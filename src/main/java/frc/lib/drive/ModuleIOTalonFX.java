package frc.lib.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.drive.SwerveConfig.CornerConfig;
import frc.lib.util.PhoenixSignals;
import frc.lib.util.PhoenixUtil;
import java.util.Queue;

/**
 * Real CTRE Kraken X60 + CANcoder implementation of {@link ModuleIO}. Enforces <= 60A Stator / <=
 * 40A Supply / <= 60A Slip current limits (preventing the NYTR 120A SlipCurrent overwrite bug),
 * reuses pre-allocated {@link VelocityVoltage} and {@link MotionMagicExpoVoltage} requests, and
 * registers 250 Hz odometry signals with {@link PhoenixOdometryThread}.
 */
public class ModuleIOTalonFX implements ModuleIO, AutoCloseable {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;
  private final SwerveConfig swerveConfig;

  private final VelocityVoltage driveVelocityReq = new VelocityVoltage(0).withSlot(0);
  private final MotionMagicExpoVoltage turnPositionReq =
      new MotionMagicExpoVoltage(0).withSlot(0);
  private final VoltageOut driveVoltsReq = new VoltageOut(0);
  private final VoltageOut turnVoltsReq = new VoltageOut(0);
  private final NeutralOut neutralReq = new NeutralOut();

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveStatorCurrent;
  private final StatusSignal<Current> driveSupplyCurrent;

  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnStatorCurrent;
  private final StatusSignal<Current> turnSupplyCurrent;

  private final int driveSignalGroup;
  private final int turnSignalGroup;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private double activeStatorLimitAmps;

  public ModuleIOTalonFX(SwerveConfig swerveConfig, CornerConfig corner) {
    if (swerveConfig.driveStatorLimitAmps() > 60.0 || swerveConfig.driveSlipCurrentAmps() > 60.0) {
      throw new IllegalArgumentException(
          "Refusing swerve drive stator/slip current > 60.0A on corner "
              + corner.name()
              + " (stator="
              + swerveConfig.driveStatorLimitAmps()
              + "A, slip="
              + swerveConfig.driveSlipCurrentAmps()
              + "A) to prevent battery brownouts.");
    }
    if (swerveConfig.driveSupplyLimitAmps() > 40.0) {
      throw new IllegalArgumentException(
          "Refusing swerve drive supply current > 40.0A on corner " + corner.name());
    }

    this.swerveConfig = swerveConfig;
    this.activeStatorLimitAmps =
        Math.min(swerveConfig.driveStatorLimitAmps(), swerveConfig.driveSlipCurrentAmps());

    CANBus bus = new CANBus(swerveConfig.canBus());
    this.driveTalon = new TalonFX(corner.driveMotorId(), bus);
    this.turnTalon = new TalonFX(corner.turnMotorId(), bus);
    this.cancoder = new CANcoder(corner.cancoderId(), bus);

    CANcoderConfiguration cancoderCfg = new CANcoderConfiguration();
    cancoderCfg.MagnetSensor.MagnetOffset = corner.encoderOffsetRot();
    PhoenixUtil.tryUntilOk(() -> cancoder.getConfigurator().apply(cancoderCfg, 0.100), 5);

    TalonFXConfiguration driveCfg = new TalonFXConfiguration();
    driveCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    boolean invertDrive = corner.invertDrive();
    driveCfg.MotorOutput.Inverted =
        invertDrive
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveCfg.Feedback.SensorToMechanismRatio = swerveConfig.driveGearRatio();
    driveCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    driveCfg.CurrentLimits.StatorCurrentLimit = activeStatorLimitAmps;
    driveCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveCfg.CurrentLimits.SupplyCurrentLimit = swerveConfig.driveSupplyLimitAmps();
    driveCfg.Slot0.kP = swerveConfig.driveKp();
    driveCfg.Slot0.kI = swerveConfig.driveKi();
    driveCfg.Slot0.kD = swerveConfig.driveKd();
    driveCfg.Slot0.kS = swerveConfig.driveKs();
    driveCfg.Slot0.kV = swerveConfig.driveKv();
    driveCfg.Slot0.kA = swerveConfig.driveKa();
    PhoenixUtil.tryUntilOk(() -> driveTalon.getConfigurator().apply(driveCfg, 0.100), 5);
    driveTalon.getSimState().Orientation =
        invertDrive
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    driveTalon.setPosition(0.0);

    TalonFXConfiguration turnCfg = new TalonFXConfiguration();
    turnCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnCfg.MotorOutput.Inverted =
        corner.invertTurn()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnCfg.Feedback.FeedbackRemoteSensorID = corner.cancoderId();
    turnCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turnCfg.Feedback.RotorToSensorRatio = swerveConfig.turnGearRatio();
    turnCfg.ClosedLoopGeneral.ContinuousWrap = true;
    turnCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    turnCfg.CurrentLimits.StatorCurrentLimit = swerveConfig.turnStatorLimitAmps();
    turnCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnCfg.CurrentLimits.SupplyCurrentLimit = swerveConfig.turnSupplyLimitAmps();
    turnCfg.Slot0.kP = swerveConfig.turnKp();
    turnCfg.Slot0.kD = swerveConfig.turnKd();
    turnCfg.Slot0.kS = 0.1;
    turnCfg.Slot0.kV = 1.5;
    turnCfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    turnCfg.MotionMagic.MotionMagicExpo_kV = 1.5;
    turnCfg.MotionMagic.MotionMagicExpo_kA = 0.1;
    PhoenixUtil.tryUntilOk(() -> turnTalon.getConfigurator().apply(turnCfg, 0.100), 5);
    turnTalon.getSimState().Orientation =
        corner.invertTurn()
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveStatorCurrent = driveTalon.getStatorCurrent();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnStatorCurrent = turnTalon.getStatorCurrent();
    turnSupplyCurrent = turnTalon.getSupplyCurrent();

    driveSignalGroup =
        PhoenixSignals.register(
            swerveConfig.canBus(),
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveStatorCurrent,
            driveSupplyCurrent);
    turnSignalGroup =
        PhoenixSignals.register(
            swerveConfig.canBus(),
            turnAbsolutePosition,
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnStatorCurrent,
            turnSupplyCurrent);

    PhoenixOdometryThread odomThread = PhoenixOdometryThread.getInstance();
    drivePositionQueue =
        odomThread.registerLatencyCompensatedSignal(drivePosition, driveVelocity);
    turnPositionQueue =
        odomThread.registerLatencyCompensatedSignal(turnPosition, turnVelocity);
  }

  /** Reads back the applied drive motor TalonFXConfiguration registers for Tier 3 verification. */
  public TalonFXConfiguration readAppliedDriveConfig() {
    TalonFXConfiguration readback = new TalonFXConfiguration();
    PhoenixUtil.tryUntilOk(() -> driveTalon.getConfigurator().refresh(readback, 0.100), 5);
    return readback;
  }

  /** Reads back the applied steer motor TalonFXConfiguration registers for Tier 3 verification. */
  public TalonFXConfiguration readAppliedTurnConfig() {
    TalonFXConfiguration readback = new TalonFXConfiguration();
    PhoenixUtil.tryUntilOk(() -> turnTalon.getConfigurator().refresh(readback, 0.100), 5);
    return readback;
  }

  public TalonFX getDriveTalon() {
    return driveTalon;
  }

  public TalonFX getTurnTalon() {
    return turnTalon;
  }

  public CANcoder getCancoder() {
    return cancoder;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(driveVelocityReq.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setTurnPosition(Rotation2d angle) {
    turnTalon.setControl(turnPositionReq.withPosition(angle.getRotations()));
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveTalon.setControl(driveVoltsReq.withOutput(volts));
  }

  @Override
  public void setTurnOpenLoop(double volts) {
    turnTalon.setControl(turnVoltsReq.withOutput(volts));
  }

  @Override
  public void setDriveStatorCurrentLimit(double statorLimitAmps) {
    // Never call getConfigurator().apply() inside the 50 Hz / 200 Hz control loop (blocking 10 ms
    // x 4 = 40 ms CAN transaction + TalonFX flash EEPROM wear). The 60A stator / 40A supply ceiling
    // is locked in TalonFX hardware at boot, and dynamic brownout load-shedding scales velocity /
    // voltage demand in Drive.runVelocity() via brownoutDemandScale.
    activeStatorLimitAmps = Math.min(60.0, Math.max(20.0, statorLimitAmps));
  }

  public double getActiveStatorLimitAmps() {
    return activeStatorLimitAmps;
  }

  @Override
  public void stop() {
    driveTalon.setControl(neutralReq);
    turnTalon.setControl(neutralReq);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = PhoenixSignals.isOK(driveSignalGroup);
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveStatorCurrentAmps = driveStatorCurrent.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();

    inputs.turnConnected = PhoenixSignals.isOK(turnSignalGroup);
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnStatorCurrentAmps = turnStatorCurrent.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();

    PhoenixOdometryThread.ODOMETRY_LOCK.lock();
    try {
      int nDrive = drivePositionQueue.size();
      inputs.odometryDrivePositionsRad = new double[nDrive];
      for (int i = 0; i < nDrive; i++) {
        Double rot = drivePositionQueue.poll();
        inputs.odometryDrivePositionsRad[i] =
            (rot != null) ? Units.rotationsToRadians(rot) : inputs.drivePositionRad;
      }

      int nTurn = turnPositionQueue.size();
      inputs.odometryTurnPositions = new Rotation2d[nTurn];
      for (int i = 0; i < nTurn; i++) {
        Double rot = turnPositionQueue.poll();
        inputs.odometryTurnPositions[i] =
            (rot != null) ? Rotation2d.fromRotations(rot) : inputs.turnPosition;
      }
    } finally {
      PhoenixOdometryThread.ODOMETRY_LOCK.unlock();
    }
  }

  @Override
  public void close() {
    PhoenixOdometryThread.getInstance().reset();
    driveTalon.close();
    turnTalon.close();
    cancoder.close();
  }
}
