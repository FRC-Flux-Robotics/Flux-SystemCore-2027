package frc.lib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
 * every cycle (zero round-robin), computes dynamic distance/tag-count covariance, checks {@link
 * FieldPositions} field/shop bounds, and quarantines bent camera mounts via {@link FaultSupervisor}.
 */
public class Vision extends SubsystemBase {
  public static final double MAX_AMBIGUITY = 0.30;
  public static final double MAX_Z_ERROR_METERS = 0.75;
  public static final double FIELD_MARGIN_METERS = 0.50;
  public static final double LINEAR_STD_DEV_BASELINE = 0.02;
  public static final double ANGULAR_STD_DEV_BASELINE = 0.06;
  public static final double QUARANTINE_STDDEV_MULTIPLIER = 5.0;

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

  private int totalFramesDrained = 0;
  private int acceptedObservations = 0;
  private int rejectedObservations = 0;
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
    int totalVisibleTags = 0;

    for (int camIdx = 0; camIdx < ios.length; camIdx++) {
      latestCameraPose[camIdx] = null;
      ios[camIdx].updateInputs(inputs[camIdx]);
      Logger.processInputs(cameraLogKeys[camIdx], inputs[camIdx]);
      totalFramesDrained += inputs[camIdx].framesDrainedThisCycle;
      totalVisibleTags += inputs[camIdx].tagIds.length;

      int len = inputs[camIdx].poseObservations.length;
      if (len > 0) {
        latestCameraPose[camIdx] =
            inputs[camIdx].poseObservations[len - 1].robotPose().toPose2d();
      }
    }

    if (faultSupervisor != null) {
      faultSupervisor.updateTagVisibility(totalVisibleTags);
      auditInterCameraConsensus(latestCameraPose);
    }

    for (int camIdx = 0; camIdx < ios.length; camIdx++) {
      double camScalar = configs[camIdx].stdDevScalar();
      boolean quarantined =
          faultSupervisor != null && faultSupervisor.isCameraQuarantined(camIdx);
      if (quarantined) {
        camScalar *= QUARANTINE_STDDEV_MULTIPLIER;
      }

      for (PoseObservation obs : inputs[camIdx].poseObservations) {
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
        lastTagSightTimestampSec = Math.max(lastTagSightTimestampSec, obs.timestampSec());
        latestAcceptedPose = pose2d;

        double stdDevFactor =
            Math.pow(obs.avgTagDistMeters(), 2.0) / Math.max(1, obs.tagCount());
        double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor * camScalar;
        double angularStdDev =
            (obs.type() == ObservationType.MEGATAG_2)
                ? Double.POSITIVE_INFINITY
                : ANGULAR_STD_DEV_BASELINE * stdDevFactor * camScalar;

        consumer.accept(
            pose2d, obs.timestampSec(), VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }
    }

    Logger.recordOutput("Vision/TotalFramesDrained", totalFramesDrained);
    Logger.recordOutput("Vision/AcceptedObservations", acceptedObservations);
    Logger.recordOutput("Vision/RejectedObservations", rejectedObservations);
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

  public double getLastTagSightTimestampSec() {
    return lastTagSightTimestampSec;
  }

  /** Feeds current gyro yaw, yaw rate, and seeded state to all camera IOs for MT1 -> MT2 handoff. */
  public void setRobotOrientation(
      Rotation2d yaw, double yawRateRadPerSec, boolean gyroSeeded) {
    for (VisionIO io : ios) {
      io.setRobotOrientation(yaw, yawRateRadPerSec, gyroSeeded);
    }
  }

  public Optional<Pose2d> getLatestAcceptedPose() {
    return Optional.ofNullable(latestAcceptedPose);
  }

  public VisionIO getCameraIo(int index) {
    return ios[index];
  }
}
