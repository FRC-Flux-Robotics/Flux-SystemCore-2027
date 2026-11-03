package frc.lib.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.util.PhoenixUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * 250 Hz (4 ms) high-frequency swerve odometry thread supporting both roboRIO CANivore and
 * SystemCore native SocketCAN (`can_s0` latency-compensated {@code refreshAll} + {@code
 * getLatencyCompensatedValue}), plus deterministic {@link #sampleOnce(double)} stepping for tests.
 */
public final class PhoenixOdometryThread {
  public static final double ODOMETRY_FREQUENCY_HZ = 250.0;
  public static final Lock ODOMETRY_LOCK = new ReentrantLock();

  private static PhoenixOdometryThread instance = null;

  private final List<BaseStatusSignal> allSignals = new ArrayList<>();
  private final List<DoubleSupplier> signalSuppliers = new ArrayList<>();
  private final List<Queue<Double>> signalQueues = new ArrayList<>();
  private final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(32);
  private BaseStatusSignal[] cachedSignals = new BaseStatusSignal[0];
  private final Thread thread;

  private PhoenixOdometryThread() {
    thread =
        new Thread(
            () -> {
              while (!Thread.currentThread().isInterrupted()) {
                try {
                  Thread.sleep((long) (1000.0 / ODOMETRY_FREQUENCY_HZ));
                  if (hasRegisteredSignals()) {
                    sampleNow();
                  }
                } catch (InterruptedException e) {
                  Thread.currentThread().interrupt();
                  break;
                }
              }
            },
            "PhoenixOdometryThread-250Hz");
    thread.setDaemon(true);
    thread.start();
  }

  public static synchronized PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  /**
   * Registers a CTRE position + velocity signal pair and returns a 250 Hz queue of latency-
   * compensated position values (in rotations).
   */
  public Queue<Double> registerLatencyCompensatedSignal(
      StatusSignal<Angle> position, StatusSignal<AngularVelocity> velocity) {
    Queue<Double> queue = new ArrayBlockingQueue<>(32);
    ODOMETRY_LOCK.lock();
    try {
      PhoenixUtil.tryUntilOk(
          () -> position.setUpdateFrequency(ODOMETRY_FREQUENCY_HZ, 0.100), 5);
      PhoenixUtil.tryUntilOk(
          () -> velocity.setUpdateFrequency(ODOMETRY_FREQUENCY_HZ, 0.100), 5);
      allSignals.add(position);
      allSignals.add(velocity);
      cachedSignals = allSignals.toArray(new BaseStatusSignal[0]);
      signalSuppliers.add(
          () ->
              BaseStatusSignal.getLatencyCompensatedValue(position, velocity).in(Units.Rotations));
      signalQueues.add(queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  /** Registers an arbitrary double supplier (for simulation or fallback IMU) at 250 Hz. */
  public Queue<Double> registerGenericSignal(DoubleSupplier supplier) {
    Queue<Double> queue = new ArrayBlockingQueue<>(32);
    ODOMETRY_LOCK.lock();
    try {
      signalSuppliers.add(supplier);
      signalQueues.add(queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  public void sampleOnce(double timestampSeconds) {
    ODOMETRY_LOCK.lock();
    try {
      if (cachedSignals.length > 0) {
        BaseStatusSignal.refreshAll(cachedSignals);
      }
      if (!timestampQueue.offer(timestampSeconds)) {
        timestampQueue.poll();
        timestampQueue.offer(timestampSeconds);
      }
      for (int i = 0; i < signalSuppliers.size(); i++) {
        double val = signalSuppliers.get(i).getAsDouble();
        Queue<Double> q = signalQueues.get(i);
        if (!q.offer(val)) {
          q.poll();
          q.offer(val);
        }
      }
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }

  public void sampleNow() {
    sampleOnce(RobotController.getFPGATime() / 1.0e6);
  }

  public boolean hasRegisteredSignals() {
    ODOMETRY_LOCK.lock();
    try {
      return !signalSuppliers.isEmpty();
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }

  /**
   * Ensures at least one 250 Hz sample is queued for the current control loop cycle when hardware
   * signals are registered.
   */
  public void ensureSampleReady() {
    ODOMETRY_LOCK.lock();
    try {
      if (!signalSuppliers.isEmpty() && timestampQueue.isEmpty()) {
        sampleNow();
      }
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }

  public double[] drainTimestamps() {
    ODOMETRY_LOCK.lock();
    try {
      double[] out = new double[timestampQueue.size()];
      int i = 0;
      while (!timestampQueue.isEmpty()) {
        out[i++] = timestampQueue.poll();
      }
      return out;
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }

  public synchronized void reset() {
    ODOMETRY_LOCK.lock();
    try {
      allSignals.clear();
      signalSuppliers.clear();
      signalQueues.clear();
      timestampQueue.clear();
      cachedSignals = new BaseStatusSignal[0];
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }
}
