package frc.lib.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * WPILib {@link DCMotorSim} drive + steer physics simulation of {@link ModuleIO} with 250 Hz
 * sub-stepping (5 x 4 ms sub-steps per 20 ms cycle) to populate high-rate odometry queues.
 */
public class ModuleIOSim implements ModuleIO {
  private static final int SUBSTEPS_PER_CYCLE = 5; // 5 * 4ms = 20ms (250 Hz)
  private static final double SUBSTEP_DT_SEC = 0.004;

  private final SwerveConfig config;
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private double driveTargetVelRadPerSec = 0.0;
  private Rotation2d turnTargetAngle = Rotation2d.kZero;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private double activeStatorLimitAmps;

  public ModuleIOSim(SwerveConfig config) {
    this.config = config;
    this.activeStatorLimitAmps =
        Math.min(config.driveStatorLimitAmps(), config.driveSlipCurrentAmps());
    DCMotor driveMotor = DCMotor.getKrakenX60Foc(1);
    DCMotor turnMotor = DCMotor.getKrakenX60Foc(1);
    this.driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotor, 0.025, config.driveGearRatio()),
            driveMotor);
    this.turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotor, 0.004, config.turnGearRatio()),
            turnMotor);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveTargetVelRadPerSec = velocityRadPerSec;
  }

  @Override
  public void setTurnPosition(Rotation2d angle) {
    turnClosedLoop = true;
    turnTargetAngle = angle;
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveClosedLoop = false;
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setTurnOpenLoop(double volts) {
    turnClosedLoop = false;
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setDriveStatorCurrentLimit(double statorLimitAmps) {
    activeStatorLimitAmps = Math.min(60.0, Math.max(20.0, statorLimitAmps));
  }

  @Override
  public void stop() {
    driveClosedLoop = false;
    turnClosedLoop = false;
    driveAppliedVolts = 0.0;
    turnAppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    double nowSec = Timer.getFPGATimestamp();
    double[] timestamps = new double[SUBSTEPS_PER_CYCLE];
    double[] drivePositions = new double[SUBSTEPS_PER_CYCLE];
    Rotation2d[] turnPositions = new Rotation2d[SUBSTEPS_PER_CYCLE];

    for (int step = 0; step < SUBSTEPS_PER_CYCLE; step++) {
      if (driveClosedLoop) {
        double velErr = driveTargetVelRadPerSec - driveSim.getAngularVelocityRadPerSec();
        double ff =
            config.driveKs() * Math.signum(driveTargetVelRadPerSec)
                + (driveTargetVelRadPerSec / (2.0 * Math.PI)) * config.driveKv();
        driveAppliedVolts = MathUtil.clamp(ff + velErr * config.driveKp(), -12.0, 12.0);
      }
      if (turnClosedLoop) {
        double angleErr =
            MathUtil.angleModulus(
                turnTargetAngle.getRadians() - turnSim.getAngularPositionRad());
        turnAppliedVolts =
            MathUtil.clamp(
                angleErr * 8.0 - turnSim.getAngularVelocityRadPerSec() * 0.15, -12.0, 12.0);
      }

      driveSim.setInputVoltage(driveAppliedVolts);
      turnSim.setInputVoltage(turnAppliedVolts);
      driveSim.update(SUBSTEP_DT_SEC);
      turnSim.update(SUBSTEP_DT_SEC);

      timestamps[step] = nowSec - (SUBSTEPS_PER_CYCLE - 1 - step) * SUBSTEP_DT_SEC;
      drivePositions[step] = driveSim.getAngularPositionRad();
      turnPositions[step] = new Rotation2d(turnSim.getAngularPositionRad());
    }

    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveStatorCurrentAmps =
        Math.min(activeStatorLimitAmps, Math.abs(driveSim.getCurrentDrawAmps()));
    inputs.driveSupplyCurrentAmps =
        Math.min(
            config.driveSupplyLimitAmps(),
            inputs.driveStatorCurrentAmps * (Math.abs(driveAppliedVolts) / 12.0));

    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnAbsolutePosition = inputs.turnPosition;
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnStatorCurrentAmps =
        Math.min(config.turnStatorLimitAmps(), Math.abs(turnSim.getCurrentDrawAmps()));
    inputs.turnSupplyCurrentAmps =
        Math.min(
            config.turnSupplyLimitAmps(),
            inputs.turnStatorCurrentAmps * (Math.abs(turnAppliedVolts) / 12.0));

    inputs.odometryTimestamps = timestamps;
    inputs.odometryDrivePositionsRad = drivePositions;
    inputs.odometryTurnPositions = turnPositions;
  }
}
