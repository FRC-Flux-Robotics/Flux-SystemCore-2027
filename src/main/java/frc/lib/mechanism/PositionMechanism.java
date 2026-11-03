package frc.lib.mechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.mechanism.MechanismIO.MechanismIOInputs;
import frc.lib.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Generic config-driven position mechanism subsystem backed by {@link MechanismIO} with {@link
 * LoggedTunableNumber} PID/gravity gains.
 */
public class PositionMechanism extends SubsystemBase {

  private final MechanismIO io;
  private final MechanismSpec spec;
  private final MechanismIOInputs inputs = new MechanismIOInputs();

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kG;

  private double targetPosition = 0.0;
  private final double toleranceRot;

  public PositionMechanism(MechanismIO io, MechanismSpec spec) {
    this(io, spec, 0.25);
  }

  public PositionMechanism(MechanismIO io, MechanismSpec spec, double toleranceRot) {
    this.io = io;
    this.spec = spec;
    this.toleranceRot = toleranceRot;

    String prefix = spec.mechanism();
    kP = new LoggedTunableNumber(prefix + "/kP", spec.kP());
    kI = new LoggedTunableNumber(prefix + "/kI", spec.kI());
    kD = new LoggedTunableNumber(prefix + "/kD", spec.kD());
    kS = new LoggedTunableNumber(prefix + "/kS", spec.kS());
    kV = new LoggedTunableNumber(prefix + "/kV", spec.kV());
    kG = new LoggedTunableNumber(prefix + "/kG", spec.kG());
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        vals -> io.setPID(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]),
        kP,
        kI,
        kD,
        kS,
        kV,
        kG);

    io.updateInputs(inputs);
    Logger.processInputs(spec.mechanism(), inputs);
    Logger.recordOutput(spec.mechanism() + "/TargetPosition", targetPosition);
    Logger.recordOutput(spec.mechanism() + "/AtTarget", atTarget());
  }

  /** Commands the mechanism to a target position in rotations (clamped to CSV soft limits). */
  public void setPosition(double positionRotations) {
    targetPosition = MathUtil.clamp(positionRotations, spec.minPosRot(), spec.maxPosRot());
    io.setPosition(targetPosition);
  }

  /** Commands open-loop voltage directly (for current-stall homing or characterization). */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void stop() {
    io.stop();
  }

  public boolean atTarget() {
    return Math.abs(getPosition() - targetPosition) <= toleranceRot;
  }

  public double getPosition() {
    return inputs.positionRotations;
  }

  public double getVelocity() {
    return inputs.velocityRPS;
  }

  public double getStatorCurrentAmps() {
    return inputs.statorCurrentA;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentA;
  }

  public double getAppliedVoltage() {
    return inputs.appliedVoltage;
  }

  public boolean isConnected() {
    return inputs.motorConnected;
  }

  public void resetPosition() {
    io.resetPosition();
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public MechanismSpec getSpec() {
    return spec;
  }
}
