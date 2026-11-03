package frc.lib.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.GyroIO.GyroIOInputs;
import frc.lib.safety.FaultSupervisor;
import frc.lib.util.FieldPositions;
import frc.lib.util.FieldPositions.FieldProfile;
import frc.lib.util.SafeAutoBuilder;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Pure AdvantageKit 250 Hz Swerve Drive subsystem (`Drive` + `GyroIO` + 4x `ModuleIO`) with
 * built-in {@link VirtualGeofence} enforcement in `SHOP_5X3` mode, dynamic CG anti-tipping
 * acceleration scaling, and {@link FaultSupervisor} stator current throttling.
 */
public class Drive extends SubsystemBase {
  private final SwerveConfig config;
  private final GyroIO gyroIo;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  private final Module[] modules = new Module[4];
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private final SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private double loopPeriodSec = 0.020;
  private double maxAccelScale = 1.0;
  private double brownoutDemandScale = 1.0;
  private int totalOdometrySamplesIntegrated = 0;
  private double lastVisionTimestampSec = -1.0;

  public Drive(SwerveConfig config, GyroIO gyroIo, ModuleIO... moduleIos) {
    if (moduleIos.length != 4) {
      throw new IllegalArgumentException("Drive requires exactly 4 ModuleIO instances");
    }
    this.config = config;
    this.gyroIo = gyroIo;

    Translation2d[] moduleTranslations = new Translation2d[4];
    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(moduleIos[i], i, config.wheelRadiusMeters());
      moduleTranslations[i] = config.corners()[i].moduleTranslation();
    }
    kinematics = new SwerveDriveKinematics(moduleTranslations);
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);
    configurePathPlanner(moduleTranslations);
  }

  private void configurePathPlanner(Translation2d[] moduleTranslations) {
    ModuleConfig ppModuleConfig =
        new ModuleConfig(
            config.wheelRadiusMeters(),
            config.maxLinearSpeedMps(),
            1.2,
            DCMotor.getKrakenX60(1),
            config.driveGearRatio(),
            config.driveStatorLimitAmps(),
            1);
    RobotConfig ppRobotConfig =
        new RobotConfig(
            58.0,
            5.8,
            ppModuleConfig,
            moduleTranslations[0],
            moduleTranslations[1],
            moduleTranslations[2],
            moduleTranslations[3]);
    if (!AutoBuilder.isConfigured()) {
      AutoBuilder.configure(
          this::getPose,
          this::setPose,
          this::getChassisSpeeds,
          (speeds, feedforwards) -> this.runVelocity(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(8.0, 0.0, 0.0), new PIDConstants(6.0, 0.0, 0.0)),
          ppRobotConfig,
          FieldPositions::isRedAlliance,
          this);
    }
  }

  /**
   * Builds a PathPlanner path-following command wrapped in {@link SafeAutoBuilder} pre-flight and
   * runtime collision/overshoot guards.
   */
  public Command followPathCommand(String pathName) {
    return SafeAutoBuilder.followPathCommand(
        pathName,
        this,
        this::getLoopPeriodSeconds,
        this::getPose,
        this::getChassisSpeeds,
        this::runVelocity);
  }

  public Optional<Pose2d> getPathStartingPose(String pathName) {
    return SafeAutoBuilder.getPathStartingPose(pathName);
  }

  public void setLoopPeriodSeconds(double loopPeriodSec) {
    this.loopPeriodSec = Math.max(0.002, loopPeriodSec);
  }

  public double getLoopPeriodSeconds() {
    return loopPeriodSec;
  }

  public void setMaxAccelerationScale(double scale) {
    maxAccelScale = Math.max(0.0, Math.min(1.0, scale));
  }

  @Override
  public void periodic() {
    double[] sampleTimestamps;
    PhoenixOdometryThread.ODOMETRY_LOCK.lock();
    try {
      PhoenixOdometryThread.getInstance().ensureSampleReady();
      gyroIo.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (Module module : modules) {
        module.periodic();
      }

      sampleTimestamps = PhoenixOdometryThread.getInstance().drainTimestamps();
      if (sampleTimestamps.length == 0) {
        sampleTimestamps = modules[0].getOdometryTimestamps();
      }
    } finally {
      PhoenixOdometryThread.ODOMETRY_LOCK.unlock();
    }

    int sampleCount = sampleTimestamps.length;
    for (int m = 0; m < 4; m++) {
      sampleCount = Math.min(sampleCount, modules[m].getOdometryPositions().length);
    }

    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int m = 0; m < 4; m++) {
        modulePositions[m] = modules[m].getOdometryPositions()[i];
        moduleDeltas[m] =
            new SwerveModulePosition(
                modulePositions[m].distanceMeters - lastModulePositions[m].distanceMeters,
                modulePositions[m].angle);
        lastModulePositions[m] = modulePositions[m];
      }

      if (gyroInputs.connected && i < gyroInputs.odometryYawPositions.length) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      totalOdometrySamplesIntegrated++;
    }

    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/ChassisSpeeds", getChassisSpeeds());
    Logger.recordOutput("Drive/TotalSupplyCurrentAmps", getTotalSupplyCurrentAmps());
    Logger.recordOutput("Drive/OdometrySamplesIntegrated", totalOdometrySamplesIntegrated);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    double effectiveScale = maxAccelScale * brownoutDemandScale;
    ChassisSpeeds scaled =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond * effectiveScale,
            speeds.vyMetersPerSecond * effectiveScale,
            speeds.omegaRadiansPerSecond * effectiveScale);
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(scaled, loopPeriodSec);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, config.maxLinearSpeedMps());

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
    Logger.recordOutput("Drive/Setpoints", setpointStates);
  }

  /**
   * Commands field-relative velocities, automatically clamping with {@link VirtualGeofence} when
   * {@link FieldProfile#SHOP_5X3}, {@link FieldProfile#HALF_FIELD_BLUE}, or {@link
   * FieldProfile#HALF_FIELD_RED} is active.
   */
  public void runFieldRelative(
      ChassisSpeeds desiredFieldSpeeds, boolean deadManHeld, double lastTagSightAgeSec) {
    ChassisSpeeds safeFieldSpeeds = desiredFieldSpeeds;
    FieldProfile profile = FieldPositions.getFieldProfile();
    if (profile != FieldProfile.FULL_FIELD) {
      Pose2d pose = getPose();
      safeFieldSpeeds =
          VirtualGeofence.clampProfileSpeeds(
              pose.getX(),
              pose.getY(),
              desiredFieldSpeeds,
              deadManHeld,
              lastTagSightAgeSec,
              FieldPositions.getCoordinateSystem(),
              profile);
    }
    ChassisSpeeds robotRel =
        ChassisSpeeds.fromFieldRelativeSpeeds(safeFieldSpeeds, getPose().getRotation());
    runVelocity(robotRel);
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings =
        new Rotation2d[] {
          Rotation2d.fromDegrees(45.0),
          Rotation2d.fromDegrees(-45.0),
          Rotation2d.fromDegrees(-45.0),
          Rotation2d.fromDegrees(45.0)
        };
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(new SwerveModuleState(0.0, headings[i]));
    }
  }

  /**
   * Applies dynamic brownout load-shedding from {@link FaultSupervisor} by scaling commanded drive
   * demand in {@link #runVelocity(ChassisSpeeds)} with zero blocking CAN configurator / EEPROM
   * writes.
   */
  public void applyBatteryProtection(FaultSupervisor supervisor) {
    double limitAmps = supervisor.activeDriveStatorLimitAmps();
    double nominalStatorAmps = Math.max(20.0, config.driveStatorLimitAmps());
    brownoutDemandScale = Math.max(0.58, Math.min(1.0, limitAmps / nominalStatorAmps));
    for (Module m : modules) {
      m.setDriveStatorCurrentLimit(limitAmps);
    }
    Logger.recordOutput("Drive/ActiveStatorLimitAmps", limitAmps);
    Logger.recordOutput("Drive/BrownoutDemandScale", brownoutDemandScale);
  }

  public double getBrownoutDemandScale() {
    return brownoutDemandScale;
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    lastVisionTimestampSec = Math.max(lastVisionTimestampSec, timestampSeconds);
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public double getLastVisionSightAgeSec() {
    if (lastVisionTimestampSec < 0.0) {
      return 999.0;
    }
    return Math.max(0.0, Timer.getFPGATimestamp() - lastVisionTimestampSec);
  }

  public double getLastVisionSightTimestampSec() {
    return lastVisionTimestampSec;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    gyroIo.setYaw(pose.getRotation());
    rawGyroRotation = pose.getRotation();
    SwerveModulePosition[] currentPositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      currentPositions[i] = modules[i].getPosition();
      lastModulePositions[i] = currentPositions[i];
    }
    poseEstimator.resetPosition(rawGyroRotation, currentPositions, pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return kinematics.toChassisSpeeds(states);
  }

  public double getTotalSupplyCurrentAmps() {
    double total = 0.0;
    for (Module m : modules) {
      total += m.getTotalSupplyCurrentAmps();
    }
    return total;
  }

  public int getOdometrySampleCount() {
    return totalOdometrySamplesIntegrated;
  }

  public double getMaxLinearSpeedMps() {
    return config.maxLinearSpeedMps();
  }

  public SwerveConfig getConfig() {
    return config;
  }
}
