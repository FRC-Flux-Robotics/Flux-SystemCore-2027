package frc.lib.mechanism;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Hardware abstraction for a mechanism (single or dual motor). */
public interface MechanismIO {

  class MechanismIOInputs implements LoggableInputs {
    public double positionRotations = 0.0;
    public double velocityRPS = 0.0;
    public double statorCurrentA = 0.0;
    public double supplyCurrentA = 0.0;
    public double appliedVoltage = 0.0;
    public double tempCelsius = 0.0;
    public boolean motorConnected = false;

    @Override
    public void toLog(LogTable table) {
      table.put("PositionRotations", positionRotations);
      table.put("VelocityRPS", velocityRPS);
      table.put("StatorCurrentA", statorCurrentA);
      table.put("SupplyCurrentA", supplyCurrentA);
      table.put("AppliedVoltage", appliedVoltage);
      table.put("TempCelsius", tempCelsius);
      table.put("MotorConnected", motorConnected);
    }

    @Override
    public void fromLog(LogTable table) {
      positionRotations = table.get("PositionRotations", positionRotations);
      velocityRPS = table.get("VelocityRPS", velocityRPS);
      statorCurrentA = table.get("StatorCurrentA", statorCurrentA);
      supplyCurrentA = table.get("SupplyCurrentA", supplyCurrentA);
      appliedVoltage = table.get("AppliedVoltage", appliedVoltage);
      tempCelsius = table.get("TempCelsius", tempCelsius);
      motorConnected = table.get("MotorConnected", motorConnected);
    }
  }

  default void updateInputs(MechanismIOInputs inputs) {}

  default void setPosition(double positionRotations) {}

  default void setVelocity(double velocityRPS) {}

  default void setVoltage(double volts) {}

  default void setPID(double kP, double kI, double kD, double kS, double kV, double kG) {}

  default void stop() {}

  default void resetPosition() {}
}
