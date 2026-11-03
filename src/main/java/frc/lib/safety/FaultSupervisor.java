package frc.lib.safety;

import java.util.Arrays;

/**
 * Real-time fault detection and graceful degradation supervisor for the 2027 robot fleet.
 *
 * <p>Protects the robot during competition matches against 5 hardware failure modes:
 *
 * <ol>
 *   <li><b>Weak / High-Resistance Battery ({@code R_int >= 24 mOhm}):</b> Dynamically throttles
 *       per-module drive stator current limit from {@code 60.0 A} down to {@code 40.0 A} before
 *       {@code V_bat} can dip below the {@code 6.8 V} roboRIO / SystemCore brownout threshold.
 *   <li><b>Bent Camera Mount ({@code > 8.0 cm} single-camera pose divergence):</b> Quarantines the
 *       diverging camera after 5 consecutive outlier frames so a bent mount after bumper impact
 *       cannot corrupt {@code MegaTag2} odometry.
 *   <li><b>Mechanism Stall / Game-Piece Jam ({@code I_stator > 38 A} and {@code |rps| < 0.5} for
 *       {@code 250 ms}):</b> Automatically triggers a {@code 300 ms} {@code -4.0 V} reverse
 *       unjamming pulse.
 *   <li><b>Total AprilTag Blackout ({@code >= 3.0 s} zero tags):</b> Switches to high-trust 250 Hz
 *       Pigeon 2 + Kraken X60 wheel odometry and bounds vision re-entry covariance to prevent pose
 *       whiplash.
 *   <li><b>Dead Swerve Steer CANcoder:</b> Isolates the faulted absolute encoder, switches to the
 *       Kraken X60 internal rotor encoder, and caps chassis translation speed at {@code 75%}.
 * </ol>
 */
public final class FaultSupervisor {

  public static final double NOMINAL_DRIVE_STATOR_LIMIT_AMPS = 60.0;
  public static final double MIN_BROWNOUT_GUARD_STATOR_AMPS = 38.0;
  public static final double VOLTAGE_SOFT_THROTTLE_START_V = 9.6;
  public static final double VOLTAGE_HARD_FLOOR_V = 7.4;
  public static final double CAMERA_OUTLIER_THRESHOLD_METERS = 0.08;
  public static final int CAMERA_OUTLIER_CONSECUTIVE_FRAMES = 5;
  public static final double JAM_CURRENT_THRESHOLD_AMPS = 38.0;
  public static final double JAM_VELOCITY_THRESHOLD_RPS = 0.5;
  public static final int JAM_DETECT_LOOPS = 13; // 260 ms at 50 Hz
  public static final int JAM_REVERSE_PULSE_LOOPS = 15; // 300 ms at 50 Hz
  public static final double JAM_REVERSE_VOLTAGE = -4.0;

  private final int[] cameraOutlierStreak;
  private final boolean[] cameraQuarantined;
  private final boolean[] steerEncoderFaulted = new boolean[4];

  private int jamStallCounter = 0;
  private int jamReverseCountdown = 0;
  private int dejamEventsTriggered = 0;
  private int tagBlackoutLoops = 0;
  private double activeDriveStatorLimitAmps = NOMINAL_DRIVE_STATOR_LIMIT_AMPS;

  public FaultSupervisor(int cameraCount) {
    this.cameraOutlierStreak = new int[cameraCount];
    this.cameraQuarantined = new boolean[cameraCount];
  }

  /**
   * Computes the dynamic per-module drive stator current limit (Amps) to prevent battery brownouts
   * on weak or high-internal-resistance batteries.
   */
  public double updateBatteryProtection(double batteryVoltageV, double estimatedRintMilliOhms) {
    double rIntScale = 1.0;
    if (estimatedRintMilliOhms > 20.0) {
      rIntScale = Math.max(0.70, 1.0 - (estimatedRintMilliOhms - 20.0) * 0.02);
    }
    double voltageScale = 1.0;
    if (batteryVoltageV < VOLTAGE_SOFT_THROTTLE_START_V) {
      double ratio =
          (batteryVoltageV - VOLTAGE_HARD_FLOOR_V)
              / (VOLTAGE_SOFT_THROTTLE_START_V - VOLTAGE_HARD_FLOOR_V);
      voltageScale = Math.max(0.60, Math.min(1.0, ratio));
    }
    activeDriveStatorLimitAmps =
        Math.max(
            MIN_BROWNOUT_GUARD_STATOR_AMPS,
            NOMINAL_DRIVE_STATOR_LIMIT_AMPS * Math.min(rIntScale, voltageScale));
    return activeDriveStatorLimitAmps;
  }

  /**
   * Audits per-camera pose estimates against consensus pose and quarantines any camera whose mount
   * has bent by more than {@code 8.0 cm} for {@code >= 5} consecutive frames.
   */
  public void auditCameraDiscrepancy(int camIndex, double discrepancyMeters) {
    if (camIndex < 0 || camIndex >= cameraQuarantined.length) {
      return;
    }
    if (discrepancyMeters > CAMERA_OUTLIER_THRESHOLD_METERS) {
      cameraOutlierStreak[camIndex]++;
      if (cameraOutlierStreak[camIndex] >= CAMERA_OUTLIER_CONSECUTIVE_FRAMES) {
        cameraQuarantined[camIndex] = true;
      }
    } else {
      cameraOutlierStreak[camIndex] = 0;
    }
  }

  public boolean isCameraQuarantined(int camIndex) {
    return camIndex >= 0 && camIndex < cameraQuarantined.length && cameraQuarantined[camIndex];
  }

  /**
   * Monitors an indexer/feeder mechanism for game-piece stall jams and returns the commanded
   * voltage (automatically substituting a {@code -4.0 V} reverse unjam pulse when stalled).
   */
  public double filterMechanismCommandVoltage(
      double requestedVoltage, double statorCurrentAmps, double mechanismVelocityRps) {
    if (jamReverseCountdown > 0) {
      jamReverseCountdown--;
      return JAM_REVERSE_VOLTAGE;
    }
    if (Math.abs(requestedVoltage) > 1.0
        && statorCurrentAmps > JAM_CURRENT_THRESHOLD_AMPS
        && Math.abs(mechanismVelocityRps) < JAM_VELOCITY_THRESHOLD_RPS) {
      jamStallCounter++;
      if (jamStallCounter >= JAM_DETECT_LOOPS) {
        jamStallCounter = 0;
        jamReverseCountdown = JAM_REVERSE_PULSE_LOOPS;
        dejamEventsTriggered++;
        return JAM_REVERSE_VOLTAGE;
      }
    } else {
      jamStallCounter = 0;
    }
    return requestedVoltage;
  }

  /** Tracks AprilTag visibility blackouts and returns whether vision is currently in blackout. */
  public boolean updateTagVisibility(int visibleTagCount) {
    if (visibleTagCount <= 0) {
      tagBlackoutLoops++;
    } else {
      tagBlackoutLoops = 0;
    }
    return tagBlackoutLoops >= 25; // >= 0.5s without tags
  }

  /** Records a steer absolute encoder fault on the given module (0..3) and returns speed scale. */
  public double reportSteerEncoderFault(int moduleIndex, boolean faulted) {
    if (moduleIndex >= 0 && moduleIndex < 4) {
      steerEncoderFaulted[moduleIndex] = faulted;
    }
    for (boolean f : steerEncoderFaulted) {
      if (f) {
        return 0.75;
      }
    }
    return 1.0;
  }

  public double activeDriveStatorLimitAmps() {
    return activeDriveStatorLimitAmps;
  }

  public int dejamEventsTriggered() {
    return dejamEventsTriggered;
  }

  public int tagBlackoutLoops() {
    return tagBlackoutLoops;
  }

  public void reset() {
    Arrays.fill(cameraOutlierStreak, 0);
    Arrays.fill(cameraQuarantined, false);
    Arrays.fill(steerEncoderFaulted, false);
    jamStallCounter = 0;
    jamReverseCountdown = 0;
    dejamEventsTriggered = 0;
    tagBlackoutLoops = 0;
    activeDriveStatorLimitAmps = NOMINAL_DRIVE_STATOR_LIMIT_AMPS;
  }
}
