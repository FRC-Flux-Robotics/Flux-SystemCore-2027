package frc.lib.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Hardware abstraction for the swerve IMU / gyroscope (Pigeon 2 + SystemCore onboard IMU fallback). */
public interface GyroIO {

  /** AdvantageKit loggable inputs for the gyroscope including 250 Hz odometry yaw samples. */
  class GyroIOInputs implements LoggableInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public double yawVelocityRadPerSec = 0.0;
    public double rollRad = 0.0;
    public double pitchRad = 0.0;
    public double supplyVoltageV = 12.0;
    public boolean usingOnboardFallback = false;
    public double[] odometryYawTimestamps = new double[0];
    public Rotation2d[] odometryYawPositions = new Rotation2d[0];

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("YawPositionRad", yawPosition.getRadians());
      table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
      table.put("RollRad", rollRad);
      table.put("PitchRad", pitchRad);
      table.put("SupplyVoltageV", supplyVoltageV);
      table.put("UsingOnboardFallback", usingOnboardFallback);
      table.put("OdometryYawTimestamps", odometryYawTimestamps);
      double[] yawRads = new double[odometryYawPositions.length];
      for (int i = 0; i < odometryYawPositions.length; i++) {
        yawRads[i] = odometryYawPositions[i].getRadians();
      }
      table.put("OdometryYawPositionsRad", yawRads);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      yawPosition = new Rotation2d(table.get("YawPositionRad", yawPosition.getRadians()));
      yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
      rollRad = table.get("RollRad", rollRad);
      pitchRad = table.get("PitchRad", pitchRad);
      supplyVoltageV = table.get("SupplyVoltageV", supplyVoltageV);
      usingOnboardFallback = table.get("UsingOnboardFallback", usingOnboardFallback);
      odometryYawTimestamps = table.get("OdometryYawTimestamps", odometryYawTimestamps);
      double[] yawRads = table.get("OdometryYawPositionsRad", new double[0]);
      odometryYawPositions = new Rotation2d[yawRads.length];
      for (int i = 0; i < yawRads.length; i++) {
        odometryYawPositions[i] = new Rotation2d(yawRads[i]);
      }
    }
  }

  default void updateInputs(GyroIOInputs inputs) {}

  default void setYaw(Rotation2d yaw) {}
}
