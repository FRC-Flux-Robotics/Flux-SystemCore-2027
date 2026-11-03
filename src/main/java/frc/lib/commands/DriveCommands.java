package frc.lib.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.drive.Drive;
import frc.lib.util.FieldPositions;
import frc.lib.util.PiecewiseSensitivity;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Shared robot-agnostic swerve drive command factories: field-relative joystick drive with {@link
 * PiecewiseSensitivity} and {@code SHOP_5X3} geofence protection, heading-locked drive, and
 * automated wheel radius characterization.
 */
public final class DriveCommands {
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.25;
  private static final double ANGLE_MAX_VELOCITY_RAD_PER_SEC = 8.0;
  private static final double ANGLE_MAX_ACCEL_RAD_PER_SEC_SQ = 20.0;

  private DriveCommands() {}

  /** Applies the 2D radial piecewise joystick transfer curve while preserving direction. */
  public static Translation2d getLinearVelocityFromJoysticks(
      double xInput, double yInput, PiecewiseSensitivity curve) {
    double rawMag = Math.hypot(xInput, yInput);
    if (rawMag < 1e-6) {
      return Translation2d.kZero;
    }
    double curvedMag = Math.abs(curve.transfer(Math.min(1.0, rawMag)));
    Rotation2d dir = new Rotation2d(xInput, yInput);
    return new Translation2d(curvedMag * dir.getCos(), curvedMag * dir.getSin());
  }

  /**
   * Field-relative joystick drive command using a shared {@link PiecewiseSensitivity} curve,
   * alliance-aware {@link FieldPositions#operatorForward()}, and automatic {@code SHOP_5X3}
   * geofence braking.
   */
  public static Command joystickDrive(
      Drive drive,
      PiecewiseSensitivity sensitivity,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier deadManHeldSupplier) {
    return joystickDrive(
        drive,
        sensitivity,
        sensitivity,
        xSupplier,
        ySupplier,
        omegaSupplier,
        deadManHeldSupplier);
  }

  /**
   * Field-relative joystick drive command using separate translation and rotation {@link
   * PiecewiseSensitivity} curves (matching {@code New_Robot} stick response), alliance-aware {@link
   * FieldPositions#operatorForward()}, and automatic {@code SHOP_5X3} geofence braking.
   */
  public static Command joystickDrive(
      Drive drive,
      PiecewiseSensitivity linearSensitivity,
      PiecewiseSensitivity rotSensitivity,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier deadManHeldSupplier) {
    return Commands.run(
        () -> {
          Translation2d linearPct =
              getLinearVelocityFromJoysticks(
                  xSupplier.getAsDouble(), ySupplier.getAsDouble(), linearSensitivity);
          double omegaPct = rotSensitivity.transfer(omegaSupplier.getAsDouble());

          Translation2d fieldLinearPct = linearPct.rotateBy(FieldPositions.operatorForward());
          double maxLinear = drive.getMaxLinearSpeedMps();
          double maxAngular = drive.getConfig().maxAngularSpeedRadPerSec();

          ChassisSpeeds desiredFieldSpeeds =
              new ChassisSpeeds(
                  fieldLinearPct.getX() * maxLinear,
                  fieldLinearPct.getY() * maxLinear,
                  omegaPct * maxAngular);
          drive.runFieldRelative(
              desiredFieldSpeeds,
              deadManHeldSupplier.getAsBoolean(),
              drive.getLastVisionSightAgeSec());
        },
        drive);
  }

  /**
   * Field-relative joystick translation paired with closed-loop heading lock to a target {@link
   * Rotation2d} supplier (e.g., Hub target, Coral station, or cardinal snap).
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      PiecewiseSensitivity sensitivity,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> targetHeadingSupplier,
      BooleanSupplier deadManHeldSupplier) {
    return joystickDriveAtAngle(
        drive,
        sensitivity,
        xSupplier,
        ySupplier,
        targetHeadingSupplier,
        () -> 0.0,
        deadManHeldSupplier);
  }

  /**
   * Field-relative joystick translation paired with closed-loop heading lock and angular velocity
   * feedforward (`headingFeedforwardRadPerSecSupplier`) to eliminate dynamic lead lag during
   * Shoot-on-the-Move.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      PiecewiseSensitivity sensitivity,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> targetHeadingSupplier,
      DoubleSupplier headingFeedforwardRadPerSecSupplier,
      BooleanSupplier deadManHeldSupplier,
      Subsystem... extraRequirements) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(
                ANGLE_MAX_VELOCITY_RAD_PER_SEC, ANGLE_MAX_ACCEL_RAD_PER_SEC_SQ));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    Subsystem[] allRequirements = new Subsystem[1 + extraRequirements.length];
    allRequirements[0] = drive;
    System.arraycopy(extraRequirements, 0, allRequirements, 1, extraRequirements.length);

    return Commands.run(
            () -> {
              Translation2d linearPct =
                  getLinearVelocityFromJoysticks(
                      xSupplier.getAsDouble(), ySupplier.getAsDouble(), sensitivity);
              Translation2d fieldLinearPct = linearPct.rotateBy(FieldPositions.operatorForward());
              double maxLinear = drive.getMaxLinearSpeedMps();

              Rotation2d currentHeading = drive.getPose().getRotation();
              Rotation2d targetHeading = targetHeadingSupplier.get();
              double omegaCmd =
                  angleController.calculate(
                          currentHeading.getRadians(), targetHeading.getRadians())
                      + headingFeedforwardRadPerSecSupplier.getAsDouble();

              ChassisSpeeds desiredFieldSpeeds =
                  new ChassisSpeeds(
                      fieldLinearPct.getX() * maxLinear,
                      fieldLinearPct.getY() * maxLinear,
                      omegaCmd);
              drive.runFieldRelative(
                  desiredFieldSpeeds,
                  deadManHeldSupplier.getAsBoolean(),
                  drive.getLastVisionSightAgeSec());
            },
            allRequirements)
        .beforeStarting(() -> angleController.reset(drive.getPose().getRotation().getRadians()));
  }

  /**
   * Automated wheel-radius characterization command: rotates the chassis in place and logs the
   * effective wheel radius in meters to {@code Drive/WheelRadiusCharacterizationMeters}.
   */
  public static Command wheelRadiusCharacterization(Drive drive) {
    final double[] startYawRad = new double[1];
    final double[] accumTimeSec = new double[1];
    return Commands.run(
            () -> {
              double omegaCmd = 1.5;
              drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omegaCmd));
              accumTimeSec[0] += drive.getLoopPeriodSeconds();
              Pose2d pose = drive.getPose();
              double accumYaw = Math.abs(pose.getRotation().getRadians() - startYawRad[0]);
              if (accumYaw > 0.25) {
                double expectedYaw = omegaCmd * accumTimeSec[0];
                double ratio = expectedYaw > 1e-3 ? accumYaw / expectedYaw : 1.0;
                double estimatedRadius =
                    drive.getConfig().wheelRadiusMeters()
                        * Math.max(0.85, Math.min(1.15, ratio));
                Logger.recordOutput(
                    "Drive/WheelRadiusCharacterizationMeters", estimatedRadius);
              }
            },
            drive)
        .beforeStarting(
            () -> {
              startYawRad[0] = drive.getPose().getRotation().getRadians();
              accumTimeSec[0] = 0.0;
              Logger.recordOutput(
                  "Drive/WheelRadiusCharacterizationMeters", drive.getConfig().wheelRadiusMeters());
            })
        .finallyDo(drive::stop);
  }
}
