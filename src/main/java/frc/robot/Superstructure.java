package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.lib.safety.FaultSupervisor;
import frc.robot.mechanisms.ElevatorArmKinematics;
import org.littletonrobotics.junction.Logger;

/**
 * Clean 2027 Pre-Kickoff {@code Superstructure} state coordinator template (Zero 2026 shooting
 * bloat).
 *
 * <p>Demonstrates game-agnostic multi-mechanism coordination across linear (`Elevator`), rotary
 * (`ArmPivot`), and roller (`IntakeRoller`) primitives with {@link ElevatorArmKinematics} 2D
 * keep-out collision interlocks, dynamic CG anti-tipping swerve acceleration scaling, {@link
 * FaultSupervisor} automatic reverse unjamming, and 3D {@link Pose3d} AdvantageScope telemetry.
 */
public class Superstructure extends SubsystemBase {

  /** Reusable 2027 Pre-Kickoff multi-mechanism states. */
  public enum State {
    STOWED,
    INTAKING,
    SCORING_LOW,
    SCORING_HIGH,
    EJECTING
  }

  public static final double ELEVATOR_STOW_ROT = 0.0;
  public static final double ELEVATOR_LOW_ROT = 16.0;
  public static final double ELEVATOR_HIGH_ROT = 36.0;
  public static final double KEEP_OUT_SWERVE_SPEED_SCALE = 0.25;
  public static final double ARM_STOW_ROT = 0.0;
  public static final double ARM_INTAKE_ROT = -0.10;
  public static final double ARM_SCORE_ROT = 0.18;
  public static final double ROLLER_INTAKE_RPS = 45.0;
  public static final double ROLLER_EJECT_RPS = -45.0;

  private final PositionMechanism elevator;
  private final PositionMechanism armPivot;
  private final VelocityMechanism intakeRoller;
  private final FaultSupervisor faultSupervisor;

  private State state = State.STOWED;

  public Superstructure(
      PositionMechanism elevator,
      PositionMechanism armPivot,
      VelocityMechanism intakeRoller,
      FaultSupervisor faultSupervisor) {
    this.elevator = elevator;
    this.armPivot = armPivot;
    this.intakeRoller = intakeRoller;
    this.faultSupervisor = faultSupervisor;
  }

  /** Requests a superstructure state transition, enforcing 2D keep-out interlocks. */
  public void requestState(State desiredState) {
    state = desiredState;
    applyStateSetpoints();
  }

  private void applyStateSetpoints() {
    double currentCarriageMeters =
        elevator.getPosition() * ElevatorArmKinematics.ELEVATOR_METERS_PER_ROTATION;

    switch (state) {
      case STOWED -> {
        armPivot.setPosition(ARM_STOW_ROT);
        double currentArmRad = Math.abs(armPivot.getPosition() * 2.0 * Math.PI);
        if (currentArmRad > ElevatorArmKinematics.SAFE_STOW_MAX_RAD) {
          elevator.setPosition(ElevatorArmKinematics.MIN_SAFE_ELEVATOR_ROTATIONS);
        } else {
          elevator.setPosition(ELEVATOR_STOW_ROT);
        }
        intakeRoller.stop();
      }
      case INTAKING -> {
        elevator.setPosition(ELEVATOR_STOW_ROT);
        double safeArmRad =
            ElevatorArmKinematics.clampSafeArmAngleRad(
                currentCarriageMeters, ARM_INTAKE_ROT * 2.0 * Math.PI);
        armPivot.setPosition(safeArmRad / (2.0 * Math.PI));

        double filteredV =
            faultSupervisor.filterMechanismCommandVoltage(
                8.0, intakeRoller.getStatorCurrentAmps(), intakeRoller.getVelocity());
        if (filteredV < 0.0) {
          intakeRoller.setVoltage(filteredV);
        } else {
          intakeRoller.setVelocity(ROLLER_INTAKE_RPS);
        }
      }
      case SCORING_LOW -> {
        elevator.setPosition(ELEVATOR_LOW_ROT);
        double safeArmRad =
            ElevatorArmKinematics.clampSafeArmAngleRad(
                currentCarriageMeters, ARM_SCORE_ROT * 2.0 * Math.PI);
        armPivot.setPosition(safeArmRad / (2.0 * Math.PI));
        intakeRoller.stop();
      }
      case SCORING_HIGH -> {
        elevator.setPosition(ELEVATOR_HIGH_ROT);
        double safeArmRad =
            ElevatorArmKinematics.clampSafeArmAngleRad(
                currentCarriageMeters, ARM_SCORE_ROT * 2.0 * Math.PI);
        armPivot.setPosition(safeArmRad / (2.0 * Math.PI));
        intakeRoller.stop();
      }
      case EJECTING -> {
        intakeRoller.setVelocity(ROLLER_EJECT_RPS);
      }
    }
  }

  @Override
  public void periodic() {
    applyStateSetpoints();

    double carriageMeters =
        elevator.getPosition() * ElevatorArmKinematics.ELEVATOR_METERS_PER_ROTATION;
    double armRad = armPivot.getPosition() * 2.0 * Math.PI;
    double stage1Z = ElevatorArmKinematics.stage1HeightMeters(carriageMeters);

    Pose3d[] mechPoses =
        new Pose3d[] {
          new Pose3d(0.0, 0.0, stage1Z, Rotation3d.kZero),
          new Pose3d(0.0, 0.0, carriageMeters, Rotation3d.kZero),
          new Pose3d(0.20, 0.0, carriageMeters + 0.25, new Rotation3d(0.0, -armRad, 0.0))
        };

    Logger.recordOutput("Superstructure/State", state.name());
    Logger.recordOutput("Superstructure/AtTarget", isAtTarget());
    Logger.recordOutput("Superstructure/MechanismPoses3d", mechPoses);
  }

  public boolean isAtTarget() {
    return elevator.atTarget() && armPivot.atTarget();
  }

  /** Sums total supply current across all 2027 template mechanisms for Ghost / battery sim. */
  public double getTotalMechanismSupplyCurrentAmps() {
    return elevator.getSupplyCurrentAmps()
        + armPivot.getSupplyCurrentAmps()
        + intakeRoller.getSupplyCurrentAmps();
  }

  /** Computes the dynamic CG anti-tipping swerve acceleration scale factor in `[0.0, 1.0]`. */
  public double getSafeSwerveAccelerationScale() {
    double carriageMeters =
        elevator.getPosition() * ElevatorArmKinematics.ELEVATOR_METERS_PER_ROTATION;
    double armDeg = Math.toDegrees(armPivot.getPosition() * 2.0 * Math.PI);
    if (!ElevatorArmKinematics.isArmSafe(carriageMeters, armDeg)) {
      return KEEP_OUT_SWERVE_SPEED_SCALE;
    }
    return ElevatorArmKinematics.maxAccelScaleFactor(carriageMeters);
  }

  public State getState() {
    return state;
  }

  public PositionMechanism getElevator() {
    return elevator;
  }

  public PositionMechanism getArmPivot() {
    return armPivot;
  }

  public VelocityMechanism getIntakeRoller() {
    return intakeRoller;
  }
}
