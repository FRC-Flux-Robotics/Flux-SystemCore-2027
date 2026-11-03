package frc.lib.util;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Simulates loaded battery terminal voltage {@code V_bat = V_oc - I_total * R_int} and regresses
 * online internal resistance {@code R_int = -dV / dI} for desktop simulation and Ghost
 * Superstructure brownout testing on bare chassis (`CORAL` / `CHASSIS_3`).
 */
public class VirtualBatterySim {
  public static final double BROWNOUT_VOLTAGE_THRESHOLD_V = 6.80;

  private double openCircuitVoltageV;
  private double internalResistanceOhms;
  private double terminalVoltageV;
  private double minVoltageV;
  private double peakSupplyCurrentAmps = 0.0;
  private int brownoutCount = 0;

  private double prevCurrentA = 0.0;
  private double prevVoltageV;
  private double estimatedRintOhms;

  public VirtualBatterySim(double openCircuitVoltageV, double internalResistanceOhms) {
    this.openCircuitVoltageV = openCircuitVoltageV;
    this.internalResistanceOhms = Math.max(0.005, internalResistanceOhms);
    this.terminalVoltageV = openCircuitVoltageV;
    this.minVoltageV = openCircuitVoltageV;
    this.prevVoltageV = openCircuitVoltageV;
    this.estimatedRintOhms = this.internalResistanceOhms;
  }

  /**
   * Updates terminal voltage from real drivetrain supply current plus virtual Ghost mechanism
   * supply current.
   */
  public double update(double realDriveSupplyAmps, double ghostMechSupplyAmps) {
    double totalSupplyAmps = Math.max(0.0, realDriveSupplyAmps) + Math.max(0.0, ghostMechSupplyAmps);
    peakSupplyCurrentAmps = Math.max(peakSupplyCurrentAmps, totalSupplyAmps);

    double loadedV =
        MathUtil.clamp(
            openCircuitVoltageV - totalSupplyAmps * internalResistanceOhms,
            4.5,
            openCircuitVoltageV);

    double dI = totalSupplyAmps - prevCurrentA;
    double dV = loadedV - prevVoltageV;
    if (Math.abs(dI) >= 15.0) {
      double sampleRint = MathUtil.clamp(-dV / dI, 0.005, 0.080);
      estimatedRintOhms = 0.85 * estimatedRintOhms + 0.15 * sampleRint;
    }

    prevCurrentA = totalSupplyAmps;
    prevVoltageV = loadedV;
    terminalVoltageV = loadedV;
    minVoltageV = Math.min(minVoltageV, loadedV);
    if (loadedV < BROWNOUT_VOLTAGE_THRESHOLD_V) {
      brownoutCount++;
    }

    Logger.recordOutput("BatterySim/TerminalVoltageV", terminalVoltageV);
    Logger.recordOutput("BatterySim/MinVoltageV", minVoltageV);
    Logger.recordOutput("BatterySim/TotalSupplyAmps", totalSupplyAmps);
    Logger.recordOutput("BatterySim/EstimatedRintMilliOhms", estimatedRintMilliOhms());
    Logger.recordOutput("BatterySim/BrownoutCount", brownoutCount);
    return terminalVoltageV;
  }

  public double terminalVoltageV() {
    return terminalVoltageV;
  }

  public double minVoltageV() {
    return minVoltageV;
  }

  public double peakSupplyCurrentAmps() {
    return peakSupplyCurrentAmps;
  }

  public int brownoutCount() {
    return brownoutCount;
  }

  public double estimatedRintOhms() {
    return estimatedRintOhms;
  }

  public double estimatedRintMilliOhms() {
    return estimatedRintOhms * 1000.0;
  }

  public void setBatteryParameters(double openCircuitVoltageV, double internalResistanceOhms) {
    this.openCircuitVoltageV = openCircuitVoltageV;
    this.internalResistanceOhms = Math.max(0.005, internalResistanceOhms);
  }
}
