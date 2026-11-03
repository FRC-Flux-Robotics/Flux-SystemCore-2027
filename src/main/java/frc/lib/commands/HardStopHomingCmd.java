package frc.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.mechanism.PositionMechanism;
import org.littletonrobotics.junction.Logger;

/**
 * Reusable current-stall homing command ({@code HardStopHomingCmd}) for any linear slide, elevator,
 * arm pivot, hood, tilter, or climber.
 *
 * <p>Drives a {@link PositionMechanism} gently toward its physical hard stop (`homingVolts`, e.g.
 * {@code -1.5 V}) until stator current exceeds {@code stallCurrentThresholdAmps} while velocity
 * magnitude stays below {@code maxStallVelocityRps} for {@code requiredStallLoops} consecutive
 * cycles (`100 ms` at 50 Hz), then stops the motor and zeroes the encoder.
 */
public class HardStopHomingCmd extends Command {
  public static final double DEFAULT_HOMING_VOLTS = -1.5;
  public static final double DEFAULT_STALL_CURRENT_AMPS = 18.0;
  public static final double DEFAULT_MAX_STALL_VELOCITY_RPS = 0.25;
  public static final int DEFAULT_REQUIRED_STALL_LOOPS = 5;
  public static final int DEFAULT_MAX_TIMEOUT_LOOPS = 150;

  private final PositionMechanism mechanism;
  private final double homingVolts;
  private final double stallCurrentThresholdAmps;
  private final double maxStallVelocityRps;
  private final int requiredStallLoops;
  private final int maxTimeoutLoops;

  private int stallLoopCounter = 0;
  private int totalLoops = 0;
  private boolean homed = false;

  public HardStopHomingCmd(PositionMechanism mechanism) {
    this(
        mechanism,
        DEFAULT_HOMING_VOLTS,
        DEFAULT_STALL_CURRENT_AMPS,
        DEFAULT_MAX_STALL_VELOCITY_RPS,
        DEFAULT_REQUIRED_STALL_LOOPS,
        DEFAULT_MAX_TIMEOUT_LOOPS);
  }

  public HardStopHomingCmd(
      PositionMechanism mechanism,
      double homingVolts,
      double stallCurrentThresholdAmps,
      double maxStallVelocityRps,
      int requiredStallLoops,
      int maxTimeoutLoops) {
    this.mechanism = mechanism;
    this.homingVolts = homingVolts;
    this.stallCurrentThresholdAmps = stallCurrentThresholdAmps;
    this.maxStallVelocityRps = maxStallVelocityRps;
    this.requiredStallLoops = requiredStallLoops;
    this.maxTimeoutLoops = maxTimeoutLoops;
    addRequirements(mechanism);
  }

  @Override
  public void initialize() {
    stallLoopCounter = 0;
    totalLoops = 0;
    homed = false;
    Logger.recordOutput("HardStopHoming/" + mechanism.getSpec().mechanism() + "/Homed", false);
  }

  @Override
  public void execute() {
    totalLoops++;
    mechanism.setVoltage(homingVolts);

    boolean stalled =
        mechanism.getStatorCurrentAmps() >= stallCurrentThresholdAmps
            && Math.abs(mechanism.getVelocity()) <= maxStallVelocityRps;
    if (stalled) {
      stallLoopCounter++;
    } else {
      stallLoopCounter = 0;
    }

    if (stallLoopCounter >= requiredStallLoops) {
      homed = true;
    }
  }

  @Override
  public boolean isFinished() {
    return homed || totalLoops >= maxTimeoutLoops;
  }

  @Override
  public void end(boolean interrupted) {
    mechanism.stop();
    if (!interrupted && homed) {
      mechanism.resetPosition();
    }
    Logger.recordOutput("HardStopHoming/" + mechanism.getSpec().mechanism() + "/Homed", homed);
  }

  public boolean isHomed() {
    return homed;
  }
}
