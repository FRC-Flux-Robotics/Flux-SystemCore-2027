package frc.robot.exercises;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.DriveToPose;
import frc.lib.drive.Drive;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.robot.Superstructure;
import frc.robot.Superstructure.State;
import frc.robot.autos.StudentAutoRoutines;
import frc.robot.mechanisms.ElevatorArmKinematics;

/**
 * Progressive 4-Level Hands-On Student Exercise Kit for {@code Flux-SystemCore-2027}.
 *
 * <p>Run {@code ./gradlew checkExercises} in the terminal (or VS Code Test Explorer) to grade all 4
 * exercises in under 3 seconds with zero physical robot required.
 */
public final class StudentExercises {

  public static final double TRANSLATION_DEADBAND_METERS = 0.020; // 2.0 cm
  public static final double HEADING_DEADBAND_RAD = Math.toRadians(1.0); // 1.0 deg

  /** Output record for Exercise 2 (safe coordinated Elevator + ArmPivot setpoints). */
  public record SuperstructureSetpoint(
      double elevatorTargetRot, double armTargetRot, boolean interlockActive) {}

  private StudentExercises() {}

  /**
   * Returns an intentionally broken {@link MechanismSpec} containing the 4 classic hardware config
   * bugs (odd SystemCore CAN port {@code can_s1}, {@code 120 A} stator/slip limit causing NYTR
   * brownouts, {@code 70 A} supply limit, and inverted soft limits).
   */
  public static MechanismSpec createBrokenClimberSpecExample() {
    return new MechanismSpec(
        "COMP_2027",
        "ClimberWinch",
        16,
        "can_s1", // BUG 1: Odd SystemCore CAN port shares SPI bus with can_s0
        120.0, // BUG 2: > 60A stator limit causes battery brownouts
        70.0, // BUG 3: > 40A supply limit trips breaker / sags bus
        120.0, // BUG 4: > 60A slip current overwrites stator limit on TalonFX
        "ELEVATOR_STATIC",
        2.5,
        0.0,
        0.1,
        0.20,
        0.12,
        0.01,
        0.40,
        42.0, // BUG 5: minPosRot > maxPosRot
        0.0,
        -1,
        false);
  }

  /**
   * Returns true if {@code spec} satisfies all FLUX 10413 SystemCore hardware safety rules:
   *
   * <ul>
   *   <li>Never uses odd SystemCore CAN FD ports ({@code can_s1} or {@code can_s3})
   *   <li>{@code statorLimitAmps in (0, 60.0]}, {@code supplyLimitAmps in (0, 40.0]}, {@code
   *       slipCurrentAmps in (0, 60.0]}
   *   <li>{@code minPosRot <= maxPosRot}
   * </ul>
   */
  public static boolean isMechanismSpecSafeForSystemCore(MechanismSpec spec) {
    if (spec == null || spec.canBus() == null) {
      return false;
    }
    String bus = spec.canBus().trim();
    if ("can_s1".equalsIgnoreCase(bus) || "can_s3".equalsIgnoreCase(bus)) {
      return false;
    }
    if (spec.statorLimitAmps() <= 0.0 || spec.statorLimitAmps() > 60.0) {
      return false;
    }
    if (spec.supplyLimitAmps() <= 0.0 || spec.supplyLimitAmps() > 40.0) {
      return false;
    }
    if (spec.slipCurrentAmps() <= 0.0 || spec.slipCurrentAmps() > 60.0) {
      return false;
    }
    return spec.minPosRot() <= spec.maxPosRot();
  }

  /**
   * Exercise 1 (Level 1 — 10 min): Fix a broken {@link MechanismSpec} so it passes all SystemCore
   * CAN FD and current-limit safety rules.
   */
  public static MechanismSpec exercise1_fixClimberMechanismSpec(MechanismSpec broken) {
    // TODO(student): Exercise 1 (Level 1) — Sanitize odd CAN ports (can_s1/can_s3 -> can_s2), current limits (<=60A stator/slip, <=40A supply), and soft limits, then run `./gradlew checkExercises`
    String safeBus = broken.canBus();
    if ("can_s1".equalsIgnoreCase(safeBus) || "can_s3".equalsIgnoreCase(safeBus)) {
      safeBus = "can_s2";
    }
    double safeStator = MathUtil.clamp(broken.statorLimitAmps(), 10.0, 60.0);
    double safeSupply = MathUtil.clamp(broken.supplyLimitAmps(), 10.0, 40.0);
    double safeSlip = MathUtil.clamp(broken.slipCurrentAmps(), 10.0, 60.0);
    double minRot = Math.min(broken.minPosRot(), broken.maxPosRot());
    double maxRot = Math.max(broken.minPosRot(), broken.maxPosRot());

    return new MechanismSpec(
        broken.robot(),
        broken.mechanism(),
        broken.motorId(),
        safeBus,
        safeStator,
        safeSupply,
        safeSlip,
        broken.gravityType(),
        broken.kP(),
        broken.kI(),
        broken.kD(),
        broken.kS(),
        broken.kV(),
        broken.kA(),
        broken.kG(),
        minRot,
        maxRot,
        broken.followerMotorId(),
        broken.invertFollower(),
        broken.motorInverted(),
        broken.gearRatio());
  }

  /**
   * Exercise 2 (Level 2 — 15 min): Coordinate 2027 {@code Elevator} and {@code ArmPivot} setpoints
   * so they never collide with the structural crossbar at {@link
   * ElevatorArmKinematics#CROSSBAR_HEIGHT_METERS} ({@code 0.42 m}).
   */
  public static SuperstructureSetpoint exercise2_computeSafeElevatorAndArmSetpoints(
      double currentElevatorRot,
      double currentArmRot,
      double desiredElevatorRot,
      double desiredArmRot) {
    // TODO(student): Exercise 2 (Level 2) — Coordinate 2027 Elevator & ArmPivot setpoints to prevent crossbar collisions (< 0.42 m), then run `./gradlew checkExercises`
    double currentCarriageMeters =
        currentElevatorRot * ElevatorArmKinematics.ELEVATOR_METERS_PER_ROTATION;
    double desiredCarriageMeters =
        desiredElevatorRot * ElevatorArmKinematics.ELEVATOR_METERS_PER_ROTATION;

    double desiredArmRad = desiredArmRot * 2.0 * Math.PI;
    double clampedArmRad =
        ElevatorArmKinematics.clampSafeArmAngleRad(currentCarriageMeters, desiredArmRad);
    double safeArmRot = clampedArmRad / (2.0 * Math.PI);
    boolean armClamped = Math.abs(clampedArmRad - desiredArmRad) > 1e-5;

    double currentArmRad = Math.abs(currentArmRot * 2.0 * Math.PI);
    double safeElevatorRot = desiredElevatorRot;
    boolean elevatorHeld = false;
    if (desiredCarriageMeters < ElevatorArmKinematics.CROSSBAR_HEIGHT_METERS
        && currentArmRad > ElevatorArmKinematics.SAFE_STOW_MAX_RAD) {
      safeElevatorRot = ElevatorArmKinematics.MIN_SAFE_ELEVATOR_ROTATIONS;
      elevatorHeld = true;
    }

    return new SuperstructureSetpoint(safeElevatorRot, safeArmRot, armClamped || elevatorHeld);
  }

  /**
   * Exercise 3 (Level 3 — 20 min): Compute robot-relative {@link ChassisSpeeds} to align the swerve
   * chassis from {@code currentPose} to {@code targetPose} with shortest-path angle wrapping and
   * deadbands.
   */
  public static ChassisSpeeds exercise3_computePoseAlignmentSpeeds(
      Pose2d currentPose,
      Pose2d targetPose,
      double kPTranslation,
      double kPTheta,
      double maxLinearSpeedMps,
      double maxOmegaRadPerSec) {
    // TODO(student): Exercise 3 (Level 3) — Compute swerve alignment ChassisSpeeds with 2.0 cm / 1.0 deg deadbands and `MathUtil.angleModulus` shortest-path heading wrap, then run `./gradlew checkExercises`
    double dx = targetPose.getX() - currentPose.getX();
    double dy = targetPose.getY() - currentPose.getY();
    double distMeters = Math.hypot(dx, dy);

    double vxField = 0.0;
    double vyField = 0.0;
    if (distMeters > TRANSLATION_DEADBAND_METERS) {
      double speedCmd = Math.min(maxLinearSpeedMps, distMeters * kPTranslation);
      vxField = (dx / distMeters) * speedCmd;
      vyField = (dy / distMeters) * speedCmd;
    }

    double headingErrRad =
        MathUtil.angleModulus(
            targetPose.getRotation().minus(currentPose.getRotation()).getRadians());
    double omegaCmd = 0.0;
    if (Math.abs(headingErrRad) > HEADING_DEADBAND_RAD) {
      omegaCmd = MathUtil.clamp(headingErrRad * kPTheta, -maxOmegaRadPerSec, maxOmegaRadPerSec);
    }

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        vxField, vyField, omegaCmd, currentPose.getRotation());
  }

  /**
   * Exercise 4 (Level 4 — 25 min): Build a 15-second 2-Piece Autonomous Cycle on the 2027 field and
   * visualize it in 3D via {@code ./gradlew renderAuto}.
   */
  public static Command exercise4_createTwoPieceScoringAuto(
      Drive drive, Superstructure superstructure) {
    // TODO(student): Exercise 4 (Level 4) — Customize waypoints or Superstructure states in this 15s 2-piece autonomous cycle, then run `./gradlew checkExercises` and `./gradlew renderAuto`
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
                  superstructure.requestState(State.INTAKING);
                }),
            // Piece 1: Drive to right intake station (1.8m, -0.9m, -30°) -> Score High at (0.0m, 1.0m, 180°)
            StudentAutoRoutines.withLoopTimeout(
                new DriveToPose(drive, new Pose2d(1.8, -0.9, Rotation2d.fromDegrees(-30.0))), 2.6),
            Commands.runOnce(() -> superstructure.requestState(State.SCORING_HIGH)),
            StudentAutoRoutines.withLoopTimeout(
                new DriveToPose(drive, new Pose2d(0.0, 1.0, Rotation2d.fromDegrees(180.0))), 2.8),
            Commands.runOnce(() -> superstructure.requestState(State.EJECTING)),
            // Piece 2: Lower to INTAKING, drive to left intake station (1.8m, +0.9m, +30°) -> Score High
            Commands.runOnce(() -> superstructure.requestState(State.INTAKING)),
            StudentAutoRoutines.withLoopTimeout(
                new DriveToPose(drive, new Pose2d(1.8, 0.9, Rotation2d.fromDegrees(30.0))), 2.8),
            Commands.runOnce(() -> superstructure.requestState(State.SCORING_HIGH)),
            StudentAutoRoutines.withLoopTimeout(
                new DriveToPose(drive, new Pose2d(0.0, 1.0, Rotation2d.fromDegrees(180.0))), 2.8),
            Commands.runOnce(() -> superstructure.requestState(State.EJECTING)),
            // Finish: Stow elevator and return cleanly to center (0.0, 0.0, 0°)
            Commands.runOnce(() -> superstructure.requestState(State.STOWED)),
            StudentAutoRoutines.withLoopTimeout(
                new DriveToPose(drive, new Pose2d(0.0, 0.0, Rotation2d.kZero)), 2.5))
        .withName("STUDENT EXERCISE 4: Two-Piece High Cycle");
  }
}
