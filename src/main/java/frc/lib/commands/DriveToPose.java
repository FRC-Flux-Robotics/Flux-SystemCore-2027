package frc.lib.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drive.Drive;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Field-space holonomic {@link ProfiledPIDController} alignment command (Team 3061 / 2910 pattern)
 * driving the robot smoothly to any target {@link Pose2d} using fused 250 Hz + vision odometry
 * (continuing smoothly through brief camera occlusions).
 */
public class DriveToPose extends Command {
  public static final double DEFAULT_POS_TOLERANCE_METERS = 0.03;
  public static final double DEFAULT_ANGLE_TOLERANCE_RAD = Math.toRadians(1.5);

  private final Drive drive;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final BooleanSupplier deadManHeldSupplier;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  public DriveToPose(Drive drive, Pose2d targetPose) {
    this(drive, () -> targetPose, () -> true);
  }

  public DriveToPose(
      Drive drive, Supplier<Pose2d> targetPoseSupplier, BooleanSupplier deadManHeldSupplier) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    this.deadManHeldSupplier = deadManHeldSupplier;

    double maxV = drive.getMaxLinearSpeedMps() * 0.85;
    double maxA = 3.5;
    xController =
        new ProfiledPIDController(3.2, 0.0, 0.12, new TrapezoidProfile.Constraints(maxV, maxA));
    yController =
        new ProfiledPIDController(3.2, 0.0, 0.12, new TrapezoidProfile.Constraints(maxV, maxA));
    thetaController =
        new ProfiledPIDController(
            5.0,
            0.0,
            0.20,
            new TrapezoidProfile.Constraints(
                drive.getConfig().maxAngularSpeedRadPerSec() * 0.85, 16.0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(DEFAULT_POS_TOLERANCE_METERS);
    yController.setTolerance(DEFAULT_POS_TOLERANCE_METERS);
    thetaController.setTolerance(DEFAULT_ANGLE_TOLERANCE_RAD);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d current = drive.getPose();
    xController.reset(current.getX());
    yController.reset(current.getY());
    thetaController.reset(current.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d current = drive.getPose();
    Pose2d target = targetPoseSupplier.get();

    double vx = xController.calculate(current.getX(), target.getX());
    double vy = yController.calculate(current.getY(), target.getY());
    double omega =
        thetaController.calculate(
            current.getRotation().getRadians(), target.getRotation().getRadians());

    double sightAgeSec =
        drive.getLastVisionSightTimestampSec() > 0.0
            ? drive.getLastVisionSightAgeSec()
            : 0.020;
    drive.runFieldRelative(
        new ChassisSpeeds(vx, vy, omega), deadManHeldSupplier.getAsBoolean(), sightAgeSec);

    double posErrorM = current.getTranslation().getDistance(target.getTranslation());
    double angleErrorDeg =
        Math.abs(current.getRotation().minus(target.getRotation()).getDegrees());

    Logger.recordOutput("DriveToPose/TargetPose", target);
    Logger.recordOutput("DriveToPose/PositionErrorMeters", posErrorM);
    Logger.recordOutput("DriveToPose/AngleErrorDeg", angleErrorDeg);
    Logger.recordOutput("DriveToPose/AtGoal", atGoal());
  }

  public boolean atGoal() {
    Pose2d current = drive.getPose();
    Pose2d target = targetPoseSupplier.get();
    double posErr = current.getTranslation().getDistance(target.getTranslation());
    double angErrRad = Math.abs(current.getRotation().minus(target.getRotation()).getRadians());
    return posErr <= DEFAULT_POS_TOLERANCE_METERS && angErrRad <= DEFAULT_ANGLE_TOLERANCE_RAD;
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
