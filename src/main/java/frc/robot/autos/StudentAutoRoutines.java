package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.DriveToPose;
import frc.lib.drive.Drive;
import frc.lib.util.SafeAutoBuilder;
import frc.robot.Superstructure;
import frc.robot.Superstructure.State;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Pure WPILib Autonomous {@link Command} Definitions for {@code Flux-SystemCore-2027}.
 *
 * <p>Provides:
 *
 * <ol>
 *   <li>{@link #createStudentDefaultAuto(Drive, Superstructure)} — Empty starter autonomous routine
 *       (selected by default in {@code Auto/Chooser} and rendered by {@code ./gradlew renderAuto})
 *       where students build and visualize their own autonomous routine.
 *   <li><b>Track A (Fixed Code Moves):</b> {@link #createFixedMovesExampleAuto(Drive,
 *       Superstructure)}, {@link #createScoringShowcaseAuto(Drive, Superstructure)}, and {@link
 *       #createExampleSquareAndCircleAuto(Drive, Superstructure)} demonstrating relative moves
 *       ({@link #driveRelative}), timed velocity moves ({@link #driveTimedVelocity}), loop-counted
 *       waits ({@link #waitLoopSeconds}), and field-pose PID ({@link DriveToPose}).
 *   <li><b>Track B (PathPlanner {@code .path} Moves):</b> {@link #createPathPlannerExampleAuto}
 *       demonstrating {@link #resetPoseToPathStart} and multi-path {@link
 *       Drive#followPathCommand(String)} chaining across {@code "Shop_Center_Shuttle"} and {@code
 *       "PreKickoff_2Piece_Template"}.
 * </ol>
 */
public final class StudentAutoRoutines {
  public static final String DEFAULT_STUDENT_AUTO_NAME = "STUDENT: Default Autonomous";
  public static final String FIXED_MOVES_AUTO_NAME =
      "EXAMPLE: Fixed Code Moves (Forward, Strafe, Turn)";
  public static final String PATHPLANNER_CHAIN_AUTO_NAME = "EXAMPLE: PathPlanner 2-Path Chain";
  public static final String SCORING_SHOWCASE_AUTO_NAME = "EXAMPLE: 2027 Scoring & Orbit Showcase";
  public static final String EXAMPLE_AUTO_NAME = "EXAMPLE: Center Square & Circle Orbit";

  private StudentAutoRoutines() {}

  /**
   * Default Student Autonomous Routine (selected by default in {@code Auto/Chooser} and rendered by
   * {@code ./gradlew renderAuto}).
   *
   * <p>Starts empty at field center {@code (0.0m, 0.0m, 0°)} so you can build and visualize your
   * own 15-second autonomous routine from scratch.
   */
  public static Command createStudentDefaultAuto(Drive drive, Superstructure superstructure) {
    // STUDENT: Build your own 15-second autonomous routine inside Commands.sequence(...) below, then run `./gradlew renderAuto` and open `build/auto-sim/student-auto.html`!
    // Track A — Fixed Code Moves (see createFixedMovesExampleAuto & createScoringShowcaseAuto below):
    //   • Relative move:   driveRelative(drive, 1.40, 0.0, 0.0, 2.0)        // forward +X m, left +Y m, turn +CCW deg
    //   • Turn in place:   driveRelative(drive, 0.0, 0.0, 90.0, 1.2)
    //   • Timed velocity:  driveTimedVelocity(drive, -0.80, 0.0, 0.0, 1.0)  // vx m/s, vy m/s, omega deg/s, seconds
    //   • Field pose PID:  withLoopTimeout(new DriveToPose(drive, new Pose2d(1.6, 0.0, Rotation2d.kZero)), 2.2)
    //   • Wait / timeout:  use waitLoopSeconds(0.5) & withLoopTimeout(cmd, 2.0) — NOT Commands.waitSeconds / .withTimeout!
    // Track B — PathPlanner .path Moves (see createPathPlannerExampleAuto below):
    //   • Snap start pose: resetPoseToPathStart(drive, "Shop_Center_Shuttle")
    //   • Follow .path:    withLoopTimeout(drive.followPathCommand("Shop_Center_Shuttle"), 4.5)
    // Superstructure & Orbit Helpers:
    //   • Superstructure:  Commands.runOnce(() -> superstructure.requestState(State.SCORING_HIGH))
    //   • Circle orbit:    createHolonomicCircleCommand(drive, 0.95, 4.5)
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
                  superstructure.requestState(State.STOWED);
                }))
        .withName(DEFAULT_STUDENT_AUTO_NAME);
  }

  /**
   * Track A Reference Example (Fixed Code Moves — zero PathPlanner files or field coordinates
   * needed): uses {@link #driveRelative}, {@link #waitLoopSeconds}, and {@link #driveTimedVelocity}
   * to drive forward, score low, turn 90° left, drive across, score high, back up, and return to
   * start (render via {@code ./gradlew renderAuto -Pauto="Fixed Code Moves"}).
   */
  public static Command createFixedMovesExampleAuto(Drive drive, Superstructure superstructure) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
                  superstructure.requestState(State.INTAKING);
                }),
            driveRelative(drive, 1.50, 0.0, 0.0, 2.2),
            Commands.runOnce(() -> superstructure.requestState(State.SCORING_LOW)),
            waitLoopSeconds(0.40),
            driveRelative(drive, 0.0, 0.0, 90.0, 1.2),
            driveRelative(drive, 1.10, 0.0, 0.0, 2.0),
            Commands.runOnce(() -> superstructure.requestState(State.SCORING_HIGH)),
            waitLoopSeconds(0.50),
            driveTimedVelocity(drive, -0.90, 0.0, 0.0, 1.2),
            Commands.runOnce(() -> superstructure.requestState(State.STOWED)),
            driveRelative(drive, 0.0, -1.50, -90.0, 2.4))
        .withName(FIXED_MOVES_AUTO_NAME);
  }

  /**
   * Track B Reference Example (Chaining Multiple PathPlanner {@code .path} Files): snaps to {@code
   * "Shop_Center_Shuttle.path"} start via {@link #resetPoseToPathStart}, follows {@code
   * "Shop_Center_Shuttle"}, pauses briefly, and chains {@code "PreKickoff_2Piece_Template.path"}
   * (render via {@code ./gradlew renderAuto -Pauto="PathPlanner 2-Path"}).
   */
  public static Command createPathPlannerExampleAuto(Drive drive, Superstructure superstructure) {
    return Commands.sequence(
            resetPoseToPathStart(drive, "Shop_Center_Shuttle"),
            Commands.runOnce(() -> superstructure.requestState(State.INTAKING)),
            withLoopTimeout(drive.followPathCommand("Shop_Center_Shuttle"), 4.5),
            Commands.runOnce(() -> superstructure.requestState(State.SCORING_LOW)),
            waitLoopSeconds(0.50),
            withLoopTimeout(drive.followPathCommand("PreKickoff_2Piece_Template"), 5.5),
            Commands.runOnce(() -> superstructure.requestState(State.STOWED)))
        .withName(PATHPLANNER_CHAIN_AUTO_NAME);
  }

  /**
   * Reference Example Autonomous Routine: starts at field center {@code (0, 0, 0°)}, drives a
   * {@code 1.8m x 1.2m} rectangle cycling through {@code INTAKING -> SCORING_LOW -> SCORING_HIGH ->
   * EJECTING}, and finishes with a 360° circle orbit.
   */
  public static Command createScoringShowcaseAuto(Drive drive, Superstructure superstructure) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
                  superstructure.requestState(State.INTAKING);
                }),
            withLoopTimeout(new DriveToPose(drive, new Pose2d(1.8, 0.0, Rotation2d.kZero)), 2.2),
            Commands.runOnce(() -> superstructure.requestState(State.SCORING_LOW)),
            withLoopTimeout(
                new DriveToPose(drive, new Pose2d(1.8, 1.2, Rotation2d.fromDegrees(90.0))), 2.2),
            Commands.runOnce(() -> superstructure.requestState(State.SCORING_HIGH)),
            withLoopTimeout(
                new DriveToPose(drive, new Pose2d(0.0, 1.2, Rotation2d.fromDegrees(180.0))), 2.4),
            Commands.runOnce(() -> superstructure.requestState(State.EJECTING)),
            withLoopTimeout(new DriveToPose(drive, new Pose2d(0.0, 0.0, Rotation2d.kZero)), 2.2),
            Commands.runOnce(() -> superstructure.requestState(State.STOWED)),
            createHolonomicCircleCommand(drive, 0.95, 4.6),
            withLoopTimeout(new DriveToPose(drive, new Pose2d(0.0, 0.0, Rotation2d.kZero)), 1.4))
        .withName(SCORING_SHOWCASE_AUTO_NAME);
  }

  /**
   * Reference Example Autonomous Routine: starts at field center {@code (0, 0, 0°)}, drives a
   * {@code 1.6m x 1.1m} rectangle with {@link DriveToPose}, then drives a smooth 360° circle
   * returning to {@code (0, 0)}.
   */
  public static Command createExampleSquareAndCircleAuto(
      Drive drive, Superstructure superstructure) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
                  superstructure.requestState(State.STOWED);
                }),
            withLoopTimeout(new DriveToPose(drive, new Pose2d(1.6, 0.0, Rotation2d.kZero)), 2.2),
            withLoopTimeout(
                new DriveToPose(drive, new Pose2d(1.6, 1.1, Rotation2d.fromDegrees(90.0))), 2.2),
            withLoopTimeout(
                new DriveToPose(drive, new Pose2d(0.0, 1.1, Rotation2d.fromDegrees(180.0))), 2.2),
            withLoopTimeout(new DriveToPose(drive, new Pose2d(0.0, 0.0, Rotation2d.kZero)), 2.2),
            createHolonomicCircleCommand(drive, 0.95, 4.5),
            withLoopTimeout(new DriveToPose(drive, new Pose2d(0.0, 0.0, Rotation2d.kZero)), 1.5))
        .withName(EXAMPLE_AUTO_NAME);
  }

  /**
   * Drives a continuous 360° circle of radius {@code radiusMeters} in {@code durationSeconds}
   * starting from {@code (0, 0)} while rotating the robot heading smoothly through 360°.
   */
  public static Command createHolonomicCircleCommand(
      Drive drive, double radiusMeters, double durationSeconds) {
    final int maxLoops = Math.max(1, (int) Math.round(durationSeconds / 0.020));
    final int[] loops = {0};
    final double omegaOrbit = (2.0 * Math.PI) / Math.max(0.5, durationSeconds);
    final double tangentialSpeed = radiusMeters * omegaOrbit;

    return Commands.run(
            () -> {
              double elapsedSec = loops[0] * 0.020;
              double angle = omegaOrbit * elapsedSec;
              double vxField = tangentialSpeed * Math.cos(angle);
              double vyField = -tangentialSpeed * Math.sin(angle);
              double chassisOmega = omegaOrbit;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      vxField, vyField, chassisOmega, drive.getPose().getRotation()));
              loops[0]++;
            },
            drive)
        .beforeStarting(() -> loops[0] = 0)
        .until(() -> loops[0] >= maxLoops)
        .finallyDo(drive::stop);
  }

  /**
   * Returns all named autonomous routines registered for {@code Auto/Chooser} and {@code ./gradlew
   * renderAuto -Pauto="<name>"}. Students can add additional entries here to visualize multiple
   * custom routines side-by-side without overwriting {@link #createStudentDefaultAuto}.
   */
  public static Map<String, Command> createNamedAutos(Drive drive, Superstructure superstructure) {
    Map<String, Command> autos = new LinkedHashMap<>();
    autos.put(DEFAULT_STUDENT_AUTO_NAME, createStudentDefaultAuto(drive, superstructure));
    autos.put(FIXED_MOVES_AUTO_NAME, createFixedMovesExampleAuto(drive, superstructure));
    autos.put(PATHPLANNER_CHAIN_AUTO_NAME, createPathPlannerExampleAuto(drive, superstructure));
    autos.put(SCORING_SHOWCASE_AUTO_NAME, createScoringShowcaseAuto(drive, superstructure));
    autos.put(EXAMPLE_AUTO_NAME, createExampleSquareAndCircleAuto(drive, superstructure));
    autos.put("Shop 5x3 Shuttle Path", drive.followPathCommand("Shop_Center_Shuttle"));
    autos.put(
        "Pre-Kickoff 2-Piece Template", drive.followPathCommand("PreKickoff_2Piece_Template"));
    return autos;
  }

  /**
   * Closed-loop relative swerve move from the robot's current pose at step start (forward {@code
   * +X} meters, left {@code +Y} meters, turn {@code +CCW} degrees in robot coordinates), finishing
   * when within {@code 3 cm / 1.5°} or after {@code timeoutSeconds}.
   */
  public static Command driveRelative(
      Drive drive,
      double forwardMeters,
      double leftMeters,
      double turnDegrees,
      double timeoutSeconds) {
    final Pose2d[] goalPose = {Pose2d.kZero};
    Transform2d delta =
        new Transform2d(forwardMeters, leftMeters, Rotation2d.fromDegrees(turnDegrees));
    return withLoopTimeout(
        new DriveToPose(drive, () -> goalPose[0], () -> true)
            .beforeStarting(() -> goalPose[0] = drive.getPose().plus(delta)),
        timeoutSeconds);
  }

  /**
   * Open-loop timed robot-relative velocity move ({@code vxRobotMps} forward, {@code vyRobotMps}
   * left, {@code omegaDegPerSec} counter-clockwise) for {@code durationSeconds}, stopping cleanly
   * at the end.
   */
  public static Command driveTimedVelocity(
      Drive drive,
      double vxRobotMps,
      double vyRobotMps,
      double omegaDegPerSec,
      double durationSeconds) {
    ChassisSpeeds speeds =
        new ChassisSpeeds(vxRobotMps, vyRobotMps, Math.toRadians(omegaDegPerSec));
    return withLoopTimeout(Commands.run(() -> drive.runVelocity(speeds), drive), durationSeconds)
        .finallyDo(drive::stop);
  }

  /**
   * Resets {@link Drive} odometry to the starting pose of {@code pathName} ({@code
   * src/main/deploy/pathplanner/paths/<pathName>.path}) so newly drawn PathPlanner GUI paths pass
   * {@link SafeAutoBuilder} start-pose checks without hardcoding coordinates.
   */
  public static Command resetPoseToPathStart(Drive drive, String pathName) {
    return Commands.runOnce(() -> drive.getPathStartingPose(pathName).ifPresent(drive::setPose));
  }

  /**
   * Pauses a command sequence for {@code durationSeconds} using deterministic 50 Hz (20 ms) loop
   * counting (safe in both real-robot execution and fast-forward {@code ./gradlew renderAuto}).
   */
  public static Command waitLoopSeconds(double durationSeconds) {
    return SafeAutoBuilder.waitLoopSeconds(durationSeconds);
  }

  /**
   * Wraps {@code command} with a deterministic 50 Hz (20 ms) loop-count timeout via {@link
   * SafeAutoBuilder#withLoopTimeout(Command, double)}.
   */
  public static Command withLoopTimeout(Command command, double timeoutSeconds) {
    return SafeAutoBuilder.withLoopTimeout(command, timeoutSeconds);
  }
}
