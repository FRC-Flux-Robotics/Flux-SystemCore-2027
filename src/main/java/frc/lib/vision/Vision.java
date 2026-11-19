package frc.lib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.safety.FaultSupervisor;
import frc.lib.util.FieldPositions;
import frc.lib.vision.VisionIO.ObservationType;
import frc.lib.vision.VisionIO.PoseObservation;
import frc.lib.vision.VisionIO.VisionIOInputs;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Multi-camera AprilTag vision subsystem that drains the entire unread frame queue from ALL cameras
 * every cycle (zero round-robin), computes quadratic {@code 0.01 * d^2 / N^2} covariance, enforces
 * finite-value, {@code 0.30 s} peak yaw-rate ({@code 1.6 rad/s}), {@code 5 deg} tilt, {@code 2.0
 * s} auto-start ignore, and timestamp freshness gates, checks {@link FieldPositions} bounds, and
 * quarantines bent camera mounts via {@link FaultSupervisor}.
 */
public class Vision extends SubsystemBase {
  public static final double MAX_AMBIGUITY = 0.30;
  public static final double MAX_Z_ERROR_METERS = 0.75;
  public static final double FIELD_MARGIN_METERS = 0.50;
  public static final double LINEAR_STD_DEV_BASELINE = 0.01;
  public static final double ANGULAR_STD_DEV_BASELINE = 0.06;
  public static final double QUARANTINE_STDDEV_MULTIPLIER = 5.0;

  /** Spectrum 3847 + 6328-2026 vision motion and timing rejection gates. */
  public static final double MAX_YAW_RATE_RAD_PER_SEC = 1.6;

  public static final double YAW_RATE_LOOKBACK_SEC = 0.30;
  public static final double MAX_VISION_TILT_RAD = Math.toRadians(5.0);
  public static final double MAX_ODOMETRY_TILT_RAD = Math.toRadians(25.0);
  public static final double MAX_OBS_AGE_SEC = 1.0;
  public static final double MAX_OBS_FUTURE_SEC = 0.05;
  public static final double AUTO_START_VISION_IGNORE_SEC = 2.0;

  private static final int YAW_HISTORY_CAPACITY = 64;

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  private final VisionConsumer consumer;
  private final VisionIO[] ios;
  private final VisionIOInputs[] inputs;
  private final String[] cameraLogKeys;
  private final Pose2d[] latestCameraPose;
  private final CameraSpec[] configs;
  private final FaultSupervisor faultSupervisor;

  private final double[] yawSampleTimestamps = new double[YAW_HISTORY_CAPACITY];
  private final double[] yawSampleAbsRates = new double[YAW_HISTORY_CAPACITY];
  private int yawSampleWriteIdx = 0;
  private int yawSampleCount = 0;

  private double latestTiltRad = 0.0;
  private boolean wasAutonomousEnabled = false;
  private double autoEnableStartSec = -1.0;

  private int totalFramesDrained = 0;
  private int acceptedObservations = 0;
  private int rejectedObservations = 0;
  private int rejectedByMotionGate = 0;
  private double lastTagSightTimestampSec = 0.0;
  private Pose2d latestAcceptedPose = null;

  public Vision(
      VisionConsumer consumer,
      FaultSupervisor faultSupervisor,
      CameraSpec[] configs,
      VisionIO... ios) {
    if (configs.length != ios.length) {
      throw new IllegalArgumentException("CameraSpec count must match VisionIO count");
    }
    this.consumer = consumer;
    this.faultSupervisor = faultSupervisor;
    this.configs = configs;
    this.ios = ios;
    this.inputs = new VisionIOInputs[ios.length];
    this.cameraLogKeys = new String[ios.length];
    this.latestCameraPose = new Pose2d[ios.length];
    for (int i = 0; i < ios.length; i++) {
      this.inputs[i] = new VisionIOInputs();
      this.cameraLogKeys[i] = "Vision/Camera" + i;
    }
  }

  @Override
  public void periodic() {
    double nowSec = Timer.getFPGATimestamp();
    boolean autoEnabled = DriverStation.isAutonomous() && DriverStation.isEnabled();
    if (autoEnabled && !wasAutonomousEnabled) {
      autoEnableStartSec = nowSec;
    }
    wasAutonomousEnabled = autoEnabled;
    boolean inAutoStartIgnoreWindow =
        autoEnabled
            && autoEnableStartSec >= 0.0
            && (nowSec - autoEnableStartSec) < AUTO_START_VISION_IGNORE_SEC;

    int totalVisibleTags = 0;

    for (int camIdx = 0; camIdx < ios.length; camIdx++) {
      latestCameraPose[camIdx] = null;
      ios[camIdx].updateInputs(inputs[camIdx]);
      Logger.processInputs(cameraLogKeys[camIdx], inputs[camIdx]);
      totalFramesDrained += inputs[camIdx].framesDrainedThisCycle;
      totalVisibleTags += inputs[camIdx].tagIds.length;

      int len = inputs[camIdx].poseObservations.length;
      if (len > 0) {
        Pose3d lastPose = inputs[camIdx].poseObservations[len - 1].robotPose();
        if (lastPose != null
            && Double.isFinite(lastPose.getX())
            && Double.isFinite(lastPose.getY())) {
          latestCameraPose[camIdx] = lastPose.toPose2d();
        }
      }
    }

    if (faultSupervisor != null) {
      faultSupervisor.updateTagVisibility(totalVisibleTags);
      auditInterCameraConsensus(latestCameraPose);
    }

    double peakYawRateRadPerSec = getRecentPeakYawRateRadPerSec(nowSec);
    boolean motionGateTripped =
        inAutoStartIgnoreWindow
            || peakYawRateRadPerSec > MAX_YAW_RATE_RAD_PER_SEC
            || Math.abs(latestTiltRad) > MAX_VISION_TILT_RAD;

    for (int camIdx = 0; camIdx < ios.length; camIdx++) {
      double camScalar = configs[camIdx].stdDevScalar();
      boolean quarantined =
          faultSupervisor != null && faultSupervisor.isCameraQuarantined(camIdx);
      if (quarantined) {
        camScalar *= QUARANTINE_STDDEV_MULTIPLIER;
      }

      for (PoseObservation obs : inputs[camIdx].poseObservations) {
        if (!isFinitePoseObservation(obs)) {
          rejectedObservations++;
          continue;
        }
        double obsTs = obs.timestampSec();
        boolean staleOrFuture =
            obsTs < 0.0
                || (obsTs - nowSec) > MAX_OBS_FUTURE_SEC
                || (nowSec > MAX_OBS_AGE_SEC && (nowSec - obsTs) > MAX_OBS_AGE_SEC);
        if (motionGateTripped || staleOrFuture) {
          if (motionGateTripped) {
            rejectedByMotionGate++;
          }
          rejectedObservations++;
          continue;
        }

        Pose2d pose2d = obs.robotPose().toPose2d();
        boolean reject =
            obs.tagCount() == 0
                || (obs.tagCount() == 1 && obs.ambiguity() > MAX_AMBIGUITY)
                || Math.abs(obs.robotPose().getZ()) > MAX_Z_ERROR_METERS
                || !FieldPositions.isInsideField(
                    pose2d.getX(), pose2d.getY(), FIELD_MARGIN_METERS);

        if (reject) {
          rejectedObservations++;
          continue;
        }

        acceptedObservations++;
        lastTagSightTimestampSec = Math.max(lastTagSightTimestampSec, obsTs);
        latestAcceptedPose = pose2d;

        double tagCount = Math.max(1, obs.tagCount());
        double stdDevFactor = Math.pow(obs.avgTagDistMeters(), 2.0) / (tagCount * tagCount);
        double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor * camScalar;
        double angularStdDev =
            (obs.type() == ObservationType.MEGATAG_2)
                ? Double.POSITIVE_INFINITY
                : ANGULAR_STD_DEV_BASELINE * stdDevFactor * camScalar;

        consumer.accept(
            pose2d, obsTs, VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }
    }

    Logger.recordOutput("Vision/TotalFramesDrained", totalFramesDrained);
    Logger.recordOutput("Vision/AcceptedObservations", acceptedObservations);
    Logger.recordOutput("Vision/RejectedObservations", rejectedObservations);
    Logger.recordOutput("Vision/RejectedByMotionGate", rejectedByMotionGate);
    Logger.recordOutput("Vision/RecentPeakYawRateRadPerSec", peakYawRateRadPerSec);
  }

  public static boolean isFinitePoseObservation(PoseObservation obs) {
    if (obs == null || obs.robotPose() == null) {
      return false;
    }
    Pose3d pose = obs.robotPose();
    Rotation3d rot = pose.getRotation();
    return Double.isFinite(obs.timestampSec())
        && Double.isFinite(obs.ambiguity())
        && Double.isFinite(obs.avgTagDistMeters())
        && obs.avgTagDistMeters() >= 0.0
        && Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getZ())
        && Double.isFinite(rot.getX())
        && Double.isFinite(rot.getY())
        && Double.isFinite(rot.getZ());
  }

  /**
   * Computes the 6328-2026 linear odometry trust scalar across robot tilt ({@code 1.0} up to {@code
   * 5 deg}, ramping down to {@code 0.0} at {@code >= 25 deg}).
   */
  public static double computeOdometryTiltTrustScale(double maxTiltRad) {
    if (!Double.isFinite(maxTiltRad)) {
      return 0.0;
    }
    double absTilt = Math.abs(maxTiltRad);
    if (absTilt <= MAX_VISION_TILT_RAD) {
      return 1.0;
    }
    if (absTilt >= MAX_ODOMETRY_TILT_RAD) {
      return 0.0;
    }
    return 1.0
        - (absTilt - MAX_VISION_TILT_RAD) / (MAX_ODOMETRY_TILT_RAD - MAX_VISION_TILT_RAD);
  }

  private void auditInterCameraConsensus(Pose2d[] latestCameraPose) {
    for (int i = 0; i < latestCameraPose.length; i++) {
      if (latestCameraPose[i] == null) {
        continue;
      }
      double sumDist = 0.0;
      int peers = 0;
      for (int j = 0; j < latestCameraPose.length; j++) {
        if (i != j && latestCameraPose[j] != null) {
          sumDist +=
              latestCameraPose[i].getTranslation().getDistance(latestCameraPose[j].getTranslation());
          peers++;
        }
      }
      if (peers > 0) {
        faultSupervisor.auditCameraDiscrepancy(i, sumDist / peers);
      }
    }
  }

  public int getTotalFramesDrained() {
    return totalFramesDrained;
  }

  public int getAcceptedObservationCount() {
    return acceptedObservations;
  }

  public int getRejectedObservationCount() {
    return rejectedObservations;
  }

  public int getRejectedByMotionGateCount() {
    return rejectedByMotionGate;
  }

  public double getLastTagSightTimestampSec() {
    return lastTagSightTimestampSec;
  }

  /** Feeds current gyro yaw, yaw rate, and seeded state to all camera IOs for MT1 -> MT2 handoff. */
  public void setRobotOrientation(
      Rotation2d yaw, double yawRateRadPerSec, boolean gyroSeeded) {
    setRobotOrientation(yaw, yawRateRadPerSec, gyroSeeded, 0.0);
  }

  /** Feeds current gyro yaw, yaw rate, seeded state, and tilt angle for motion-gate filtering. */
  public void setRobotOrientation(
      Rotation2d yaw, double yawRateRadPerSec, boolean gyroSeeded, double maxTiltRad) {
    recordMotionSample(yawRateRadPerSec, maxTiltRad, Timer.getFPGATimestamp());
    for (VisionIO io : ios) {
      io.setRobotOrientation(yaw, yawRateRadPerSec, gyroSeeded);
    }
  }

  /** Records a motion sample at an explicit timestamp for deterministic 0.30 s yaw-rate lookback tests. */
  public void recordMotionSample(
      double yawRateRadPerSec, double maxTiltRad, double timestampSec) {
    if (Double.isFinite(maxTiltRad)) {
      latestTiltRad = Math.abs(maxTiltRad);
    }
    if (Double.isFinite(yawRateRadPerSec) && Double.isFinite(timestampSec)) {
      yawSampleTimestamps[yawSampleWriteIdx] = timestampSec;
      yawSampleAbsRates[yawSampleWriteIdx] = Math.abs(yawRateRadPerSec);
      yawSampleWriteIdx = (yawSampleWriteIdx + 1) % YAW_HISTORY_CAPACITY;
      if (yawSampleCount < YAW_HISTORY_CAPACITY) {
        yawSampleCount++;
      }
    }
  }

  public double getRecentPeakYawRateRadPerSec(double nowSec) {
    double maxRate = 0.0;
    for (int i = 0; i < yawSampleCount; i++) {
      double age = nowSec - yawSampleTimestamps[i];
      if (age >= -0.01 && age <= YAW_RATE_LOOKBACK_SEC) {
        maxRate = Math.max(maxRate, yawSampleAbsRates[i]);
      }
    }
    return maxRate;
  }

  public Optional<Pose2d> getLatestAcceptedPose() {
    return Optional.ofNullable(latestAcceptedPose);
  }

  public VisionIO getCameraIo(int index) {
    return ios[index];
  }
}
