package frc.lib.mechanism;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.mechanism.MechanismIO.MechanismIOInputs;
import frc.lib.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Generic config-driven velocity mechanism subsystem backed by {@link MechanismIO} with {@link
 * LoggedTunableNumber} PID/feedforward gains.
 */
public class VelocityMechanism extends SubsystemBase {

  private final MechanismIO io;
  private final MechanismSpec spec;
  private final MechanismIOInputs inputs = new MechanismIOInputs();

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;

  private double targetVelocityRps = 0.0;
  private final double toleranceRps;

  public VelocityMechanism(MechanismIO io, MechanismSpec spec) {
    this(io, spec, 2.0);
  }

  public VelocityMechanism(MechanismIO io, MechanismSpec spec, double toleranceRps) {
    this.io = io;
    this.spec = spec;
    this.toleranceRps = toleranceRps;

    String prefix = spec.mechanism();
    kP = new LoggedTunableNumber(prefix + "/kP", spec.kP());
    kI = new LoggedTunableNumber(prefix + "/kI", spec.kI());
    kD = new LoggedTunableNumber(prefix + "/kD", spec.kD());
    kS = new LoggedTunableNumber(prefix + "/kS", spec.kS());
    kV = new LoggedTunableNumber(prefix + "/kV", spec.kV());
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        vals -> io.setPID(vals[0], vals[1], vals[2], vals[3], vals[4], 0.0),
        kP,
        kI,
        kD,
        kS,
        kV);

    io.updateInputs(inputs);
    Logger.processInputs(spec.mechanism(), inputs);
    Logger.recordOutput(spec.mechanism() + "/TargetVelocityRPS", targetVelocityRps);
    Logger.recordOutput(spec.mechanism() + "/AtTarget", atTarget());
  }

  /** Commands the mechanism to a target velocity in rotations per second (RPS). */
  public void setVelocity(double velocityRPS) {
    targetVelocityRps = velocityRPS;
    io.setVelocity(velocityRPS);
  }

  /** Commands open-loop voltage directly (e.g., for FaultSupervisor reverse dejam pulse). */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void stop() {
    targetVelocityRps = 0.0;
    io.stop();
  }

  public boolean atTarget() {
    return Math.abs(getVelocity() - targetVelocityRps) <= toleranceRps;
  }

  public double getVelocity() {
    return inputs.velocityRPS;
  }

  public double getPosition() {
    return inputs.positionRotations;
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

  public double getTargetVelocity() {
    return targetVelocityRps;
  }

  public MechanismSpec getSpec() {
    return spec;
  }
}
