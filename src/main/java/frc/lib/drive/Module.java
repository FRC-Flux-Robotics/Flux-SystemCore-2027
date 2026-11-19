package frc.lib.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.drive.ModuleIO.ModuleIOInputs;
import org.littletonrobotics.junction.Logger;

/**
 * Per-corner swerve module wrapper handling angle optimization, cosine velocity scaling, 250 Hz
 * odometry position array construction, and AdvantageKit logging.
 */
public class Module {
  public static final double STEER_DEADBAND_SPEED_MPS = 0.05;
  public static final double STEER_ANGLE_DEADBAND_RAD = Math.toRadians(0.3);

  private final ModuleIO io;
  private final ModuleIOInputs inputs = new ModuleIOInputs();
  private final int index;
  private final double wheelRadiusMeters;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[0];

  public Module(ModuleIO io, int index, double wheelRadiusMeters) {
    this.io = io;
    this.index = index;
    this.wheelRadiusMeters = wheelRadiusMeters;
  }

  /** Updates module inputs and reconstructs the 250 Hz {@link SwerveModulePosition} array. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    int sampleCount =
        Math.min(
            inputs.odometryDrivePositionsRad.length, inputs.odometryTurnPositions.length);
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double distanceMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
      odometryPositions[i] =
          new SwerveModulePosition(distanceMeters, inputs.odometryTurnPositions[i]);
    }
  }

  /** Optimizes and commands the desired state with cosine velocity scaling and 0.3 deg steer deadband. */
  public void runSetpoint(SwerveModuleState state) {
    Rotation2d currentAngle = getAngle();
    state.optimize(currentAngle);
    state.cosineScale(currentAngle);
    io.setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters);
    double angleErrRad = Math.abs(MathUtil.angleModulus(state.angle.minus(currentAngle).getRadians()));
    if (Math.abs(state.speedMetersPerSecond) < STEER_DEADBAND_SPEED_MPS
        && angleErrRad <= STEER_ANGLE_DEADBAND_RAD) {
      return;
    }
    io.setTurnPosition(state.angle);
  }

  public void setDriveStatorCurrentLimit(double statorLimitAmps) {
    io.setDriveStatorCurrentLimit(statorLimitAmps);
  }

  public void stop() {
    io.stop();
  }

  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadiusMeters;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadiusMeters;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public double getDriveStatorCurrentAmps() {
    return inputs.driveStatorCurrentAmps;
  }

  public double getTotalSupplyCurrentAmps() {
    return inputs.driveSupplyCurrentAmps + inputs.turnSupplyCurrentAmps;
  }

  public boolean isConnected() {
    return inputs.driveConnected && inputs.turnConnected;
  }
}
