package frc.lib.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.PhoenixSignals;
import frc.lib.util.PhoenixUtil;
import java.util.Queue;

/**
 * Real CTRE {@link Pigeon2} implementation of {@link GyroIO} with 250 Hz {@link
 * PhoenixOdometryThread} registration and SystemCore onboard 6-axis IMU fallback hook.
 */
public class GyroIOPigeon2 implements GyroIO, AutoCloseable {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Voltage> supplyVoltage;
  private final int signalGroup;
  private final Queue<Double> yawPositionQueue;

  private boolean hasOnboardImuFallback = false;
  private double fallbackYawRad = 0.0;
  private double fallbackYawRateRadPerSec = 0.0;

  public GyroIOPigeon2(int pigeonId, String canBusName) {
    pigeon = new Pigeon2(pigeonId, new CANBus(canBusName));
    Pigeon2Configuration cfg = new Pigeon2Configuration();
    PhoenixUtil.tryUntilOk(() -> pigeon.getConfigurator().apply(cfg, 0.100), 5);
    pigeon.setYaw(0.0);

    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    roll = pigeon.getRoll();
    pitch = pigeon.getPitch();
    supplyVoltage = pigeon.getSupplyVoltage();

    signalGroup =
        PhoenixSignals.register(canBusName, yaw, yawVelocity, roll, pitch, supplyVoltage);
    yawPositionQueue =
        PhoenixOdometryThread.getInstance().registerLatencyCompensatedSignal(yaw, yawVelocity);
  }

  public Pigeon2 getPigeon() {
    return pigeon;
  }

  /** Feeds the SystemCore onboard 6-axis IMU reading used if the CAN Pigeon 2 disconnects. */
  public void setOnboardImuFallback(double yawRad, double yawRateRadPerSec) {
    hasOnboardImuFallback = true;
    fallbackYawRad = yawRad;
    fallbackYawRateRadPerSec = yawRateRadPerSec;
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    pigeon.setYaw(yaw.getDegrees());
    fallbackYawRad = yaw.getRadians();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    boolean canOk = PhoenixSignals.isOK(signalGroup);
    inputs.connected = canOk;

    if (canOk || !hasOnboardImuFallback) {
      inputs.usingOnboardFallback = false;
      inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
      inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
      inputs.rollRad = Units.degreesToRadians(roll.getValueAsDouble());
      inputs.pitchRad = Units.degreesToRadians(pitch.getValueAsDouble());
      inputs.supplyVoltageV = supplyVoltage.getValueAsDouble();
    } else {
      inputs.usingOnboardFallback = true;
      inputs.yawPosition = new Rotation2d(fallbackYawRad);
      inputs.yawVelocityRadPerSec = fallbackYawRateRadPerSec;
    }

    PhoenixOdometryThread.ODOMETRY_LOCK.lock();
    try {
      int n = yawPositionQueue.size();
      inputs.odometryYawPositions = new Rotation2d[n];
      for (int i = 0; i < n; i++) {
        Double rot = yawPositionQueue.poll();
        inputs.odometryYawPositions[i] =
            (rot != null) ? Rotation2d.fromRotations(rot) : inputs.yawPosition;
      }
    } finally {
      PhoenixOdometryThread.ODOMETRY_LOCK.unlock();
    }
  }

  @Override
  public void close() {
    pigeon.close();
  }
}
