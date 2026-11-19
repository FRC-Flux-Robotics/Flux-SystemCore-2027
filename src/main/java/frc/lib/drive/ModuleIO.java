package frc.lib.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Hardware abstraction for a single swerve corner module (drive motor, steer motor, absolute
 * encoder, and 250 Hz odometry sample arrays).
 */
public interface ModuleIO {

  /** AdvantageKit loggable inputs for a single swerve module corner. */
  class ModuleIOInputs implements LoggableInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveStatorCurrentAmps = 0.0;
    public double driveSupplyCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public boolean usingFusedCancoder = false;
    public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnStatorCurrentAmps = 0.0;
    public double turnSupplyCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[0];
    public double[] odometryDrivePositionsRad = new double[0];
    public Rotation2d[] odometryTurnPositions = new Rotation2d[0];

    @Override
    public void toLog(LogTable table) {
      table.put("DriveConnected", driveConnected);
      table.put("DrivePositionRad", drivePositionRad);
      table.put("DriveVelocityRadPerSec", driveVelocityRadPerSec);
      table.put("DriveAppliedVolts", driveAppliedVolts);
      table.put("DriveStatorCurrentAmps", driveStatorCurrentAmps);
      table.put("DriveSupplyCurrentAmps", driveSupplyCurrentAmps);

      table.put("TurnConnected", turnConnected);
      table.put("UsingFusedCancoder", usingFusedCancoder);
      table.put("TurnAbsolutePositionRad", turnAbsolutePosition.getRadians());
      table.put("TurnPositionRad", turnPosition.getRadians());
      table.put("TurnVelocityRadPerSec", turnVelocityRadPerSec);
      table.put("TurnAppliedVolts", turnAppliedVolts);
      table.put("TurnStatorCurrentAmps", turnStatorCurrentAmps);
      table.put("TurnSupplyCurrentAmps", turnSupplyCurrentAmps);

      table.put("OdometryTimestamps", odometryTimestamps);
      table.put("OdometryDrivePositionsRad", odometryDrivePositionsRad);
      double[] turnRads = new double[odometryTurnPositions.length];
      for (int i = 0; i < odometryTurnPositions.length; i++) {
        turnRads[i] = odometryTurnPositions[i].getRadians();
      }
      table.put("OdometryTurnPositionsRad", turnRads);
    }

    @Override
    public void fromLog(LogTable table) {
      driveConnected = table.get("DriveConnected", driveConnected);
      drivePositionRad = table.get("DrivePositionRad", drivePositionRad);
      driveVelocityRadPerSec = table.get("DriveVelocityRadPerSec", driveVelocityRadPerSec);
      driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
      driveStatorCurrentAmps = table.get("DriveStatorCurrentAmps", driveStatorCurrentAmps);
      driveSupplyCurrentAmps = table.get("DriveSupplyCurrentAmps", driveSupplyCurrentAmps);

      turnConnected = table.get("TurnConnected", turnConnected);
      usingFusedCancoder = table.get("UsingFusedCancoder", usingFusedCancoder);
      turnAbsolutePosition =
          new Rotation2d(table.get("TurnAbsolutePositionRad", turnAbsolutePosition.getRadians()));
      turnPosition = new Rotation2d(table.get("TurnPositionRad", turnPosition.getRadians()));
      turnVelocityRadPerSec = table.get("TurnVelocityRadPerSec", turnVelocityRadPerSec);
      turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts);
      turnStatorCurrentAmps = table.get("TurnStatorCurrentAmps", turnStatorCurrentAmps);
      turnSupplyCurrentAmps = table.get("TurnSupplyCurrentAmps", turnSupplyCurrentAmps);

      odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
      odometryDrivePositionsRad = table.get("OdometryDrivePositionsRad", odometryDrivePositionsRad);
      double[] turnRads = table.get("OdometryTurnPositionsRad", new double[0]);
      odometryTurnPositions = new Rotation2d[turnRads.length];
      for (int i = 0; i < turnRads.length; i++) {
        odometryTurnPositions[i] = new Rotation2d(turnRads[i]);
      }
    }
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void setDriveVelocity(double velocityRadPerSec) {}

  default void setTurnPosition(Rotation2d angle) {}

  default void setDriveOpenLoop(double volts) {}

  default void setTurnOpenLoop(double volts) {}

  default void setDriveStatorCurrentLimit(double statorLimitAmps) {}

  default void stop() {}
}
