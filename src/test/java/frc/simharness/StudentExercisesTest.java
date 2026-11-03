package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.drive.Drive;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.robot.RobotContainer;
import frc.robot.RobotIdentity;
import frc.robot.Superstructure;
import frc.robot.exercises.StudentExercises;
import frc.robot.exercises.StudentExercises.SuperstructureSetpoint;
import frc.robot.mechanisms.ElevatorArmKinematics;
import java.nio.file.Paths;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

/**
 * Automated Grader for the 4-Level Student Exercise Kit ({@link StudentExercises}).
 *
 * <p>Run via {@code ./gradlew checkExercises} or {@code ./gradlew test}.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class StudentExercisesTest {

  private static int passedCount = 0;

  @BeforeAll
  public static void initHal() {
    HAL.initialize(500, 0);
    passedCount = 0;
  }

  @Test
  @Order(1)
  public void testExercise1_fixClimberMechanismSpec() {
    MechanismSpec broken = StudentExercises.createBrokenClimberSpecExample();
    assertFalse(
        StudentExercises.isMechanismSpecSafeForSystemCore(broken),
        "Broken example spec (can_s1, 120A stator, inverted soft limits) must be flagged unsafe");

    MechanismSpec fixed = StudentExercises.exercise1_fixClimberMechanismSpec(broken);
    assertTrue(
        StudentExercises.isMechanismSpecSafeForSystemCore(fixed),
        "Exercise 1 fixed MechanismSpec must pass SystemCore safety rules");
    assertEquals("can_s2", fixed.canBus(), "Odd port can_s1 must be remapped to even port can_s2");
    assertEquals(60.0, fixed.statorLimitAmps(), 1e-6, "Stator current clamped <= 60.0 A");
    assertEquals(40.0, fixed.supplyLimitAmps(), 1e-6, "Supply current clamped <= 40.0 A");
    assertEquals(60.0, fixed.slipCurrentAmps(), 1e-6, "Slip current clamped <= 60.0 A");
    assertTrue(fixed.minPosRot() <= fixed.maxPosRot(), "minPosRot must be <= maxPosRot");
    passedCount++;
  }

  @Test
  @Order(2)
  public void testExercise2_computeSafeElevatorAndArmSetpoints() {
    SuperstructureSetpoint lowCrossbar =
        StudentExercises.exercise2_computeSafeElevatorAndArmSetpoints(0.0, 0.0, 36.0, 0.25);
    assertTrue(lowCrossbar.interlockActive(), "Crossbar interlock must activate when elevator is low");
    assertEquals(
        ElevatorArmKinematics.SAFE_STOW_MAX_RAD / (2.0 * Math.PI),
        lowCrossbar.armTargetRot(),
        1e-5,
        "Arm must be clamped to SAFE_STOW_MAX_RAD while carriage < 0.42 m");

    SuperstructureSetpoint stowWhileExtended =
        StudentExercises.exercise2_computeSafeElevatorAndArmSetpoints(36.0, 0.20, 0.0, 0.0);
    assertTrue(
        stowWhileExtended.interlockActive(),
        "Elevator stow must hold above crossbar while arm is still extended");
    assertTrue(
        stowWhileExtended.elevatorTargetRot() > 10.0,
        "Elevator target must stay above crossbar until arm tucks inside safe window");
    passedCount++;
  }

  @Test
  @Order(3)
  public void testExercise3_computePoseAlignmentSpeeds() {
    Pose2d startPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(175.0));
    Pose2d targetPose = new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(-175.0));
    ChassisSpeeds cmd =
        StudentExercises.exercise3_computePoseAlignmentSpeeds(
            startPose, targetPose, 2.5, 5.0, 3.5, 6.0);
    // Shortest path from +175 deg to -175 deg is +10 deg (+0.1745 rad -> positive omegaCmd)
    assertTrue(
        cmd.omegaRadiansPerSecond > 0.0,
        "Shortest path from +175 deg to -175 deg must rotate counter-clockwise (+omega)");
    assertEquals(
        0.0,
        StudentExercises.exercise3_computePoseAlignmentSpeeds(
                targetPose,
                new Pose2d(2.01, -0.01, Rotation2d.fromDegrees(-174.5)),
                2.5,
                5.0,
                3.5,
                6.0)
            .vxMetersPerSecond,
        1e-6,
        "Pose inside 2.0 cm / 1.0 deg deadband must command 0.0 m/s");
    passedCount++;
  }

  @Test
  @Order(4)
  public void testExercise4_createTwoPieceScoringAuto() {
    RobotContainer container =
        new RobotContainer(RobotIdentity.COMP_2027, Paths.get("src/main/deploy/configs"), true);
    Drive drive = container.getDrive();
    Superstructure ss = container.getSuperstructure();
    var twoPieceAuto = StudentExercises.exercise4_createTwoPieceScoringAuto(drive, ss);

    twoPieceAuto.initialize();
    double maxX = 0.0;
    double maxY = 0.0;
    double minY = 0.0;
    int ejectLoops = 0;
    for (int loop = 0; loop < 750 && !twoPieceAuto.isFinished(); loop++) {
      twoPieceAuto.execute();
      drive.periodic();
      container.getElevator().periodic();
      container.getArmPivot().periodic();
      container.getIntakeRoller().periodic();
      ss.periodic();
      Pose2d p = drive.getPose();
      maxX = Math.max(maxX, p.getX());
      maxY = Math.max(maxY, p.getY());
      minY = Math.min(minY, p.getY());
      if (ss.getState() == Superstructure.State.EJECTING) {
        ejectLoops++;
      }
    }
    twoPieceAuto.end(false);
    assertTrue(maxX > 1.2, "Exercise 4 2-piece auto must reach intake stations past X = +1.2m");
    assertTrue(minY < -0.4 && maxY > 0.4, "Exercise 4 2-piece auto must visit both -Y and +Y sides");
    assertTrue(ejectLoops >= 1, "Exercise 4 2-piece auto must transition through EJECTING state");
    passedCount++;
  }

  @AfterAll
  public static void reportScoreboard() {
    PrecisionReporter.recordExactTarget(
        2,
        "tier2_student_exercises_passed",
        "Student Exercises Completed (1..4)",
        passedCount,
        4.0,
        0.0,
        "exercises");

    System.out.println(
        "FLUX 10413 Student Exercise Scoreboard: " + passedCount + "/4 EXERCISES PASSED");
  }
}
