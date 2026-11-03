package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Hardware abstraction for a single AprilTag camera draining the entire unread frame queue every
 * cycle into {@link PoseObservation} and {@link RawTagObservation} arrays.
 */
public interface VisionIO {

  /** Converts a {@link CameraSpec} row from {@code cameras.csv} into a 6-DOF {@link Transform3d}. */
  static Transform3d toTransform3d(CameraSpec spec) {
    return new Transform3d(
        new Translation3d(spec.xMeters(), spec.yMeters(), spec.zMeters()),
        new Rotation3d(
            Units.degreesToRadians(spec.rollDeg()),
            Units.degreesToRadians(spec.pitchDeg()),
            Units.degreesToRadians(spec.yawDeg())));
  }

  enum ObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  /** A single timestamped 3D robot pose observation from one camera frame. */
  record PoseObservation(
      double timestampSec,
      Pose3d robotPose,
      double ambiguity,
      int tagCount,
      double avgTagDistMeters,
      ObservationType type) {}

  /** A single raw fiducial tag observation in camera space (used by ZeroPriorBootstrapSolver). */
  record RawTagObservation(
      int tagId, double txRad, double tyRad, double distanceMeters, double ambiguity) {}

  /** AdvantageKit loggable inputs for a single vision camera. */
  class VisionIOInputs implements LoggableInputs {
    public boolean connected = false;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public RawTagObservation[] tagObservations = new RawTagObservation[0];
    public int[] tagIds = new int[0];
    public int framesDrainedThisCycle = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("TagIds", tagIds);
      table.put("FramesDrainedThisCycle", framesDrainedThisCycle);
      table.put("PoseObservationCount", poseObservations.length);
      for (int i = 0; i < poseObservations.length; i++) {
        PoseObservation obs = poseObservations[i];
        String p = "PoseObs" + i + "/";
        table.put(p + "TimestampSec", obs.timestampSec());
        table.put(p + "X", obs.robotPose().getX());
        table.put(p + "Y", obs.robotPose().getY());
        table.put(p + "Z", obs.robotPose().getZ());
        table.put(p + "RollRad", obs.robotPose().getRotation().getX());
        table.put(p + "PitchRad", obs.robotPose().getRotation().getY());
        table.put(p + "YawRad", obs.robotPose().getRotation().getZ());
        table.put(p + "Ambiguity", obs.ambiguity());
        table.put(p + "TagCount", obs.tagCount());
        table.put(p + "AvgTagDistMeters", obs.avgTagDistMeters());
        table.put(p + "Type", obs.type().name());
      }
      table.put("TagObservationCount", tagObservations.length);
      for (int i = 0; i < tagObservations.length; i++) {
        RawTagObservation tag = tagObservations[i];
        String p = "TagObs" + i + "/";
        table.put(p + "TagId", tag.tagId());
        table.put(p + "TxRad", tag.txRad());
        table.put(p + "TyRad", tag.tyRad());
        table.put(p + "DistanceMeters", tag.distanceMeters());
        table.put(p + "Ambiguity", tag.ambiguity());
      }
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      tagIds = table.get("TagIds", tagIds);
      framesDrainedThisCycle = table.get("FramesDrainedThisCycle", framesDrainedThisCycle);
      int poseCount = table.get("PoseObservationCount", 0);
      poseObservations = new PoseObservation[poseCount];
      for (int i = 0; i < poseCount; i++) {
        String p = "PoseObs" + i + "/";
        Pose3d pose =
            new Pose3d(
                table.get(p + "X", 0.0),
                table.get(p + "Y", 0.0),
                table.get(p + "Z", 0.0),
                new Rotation3d(
                    table.get(p + "RollRad", 0.0),
                    table.get(p + "PitchRad", 0.0),
                    table.get(p + "YawRad", 0.0)));
        ObservationType type =
            ObservationType.valueOf(table.get(p + "Type", ObservationType.PHOTONVISION.name()));
        poseObservations[i] =
            new PoseObservation(
                table.get(p + "TimestampSec", 0.0),
                pose,
                table.get(p + "Ambiguity", 0.0),
                table.get(p + "TagCount", 0),
                table.get(p + "AvgTagDistMeters", 0.0),
                type);
      }
      int tagCount = table.get("TagObservationCount", 0);
      tagObservations = new RawTagObservation[tagCount];
      for (int i = 0; i < tagCount; i++) {
        String p = "TagObs" + i + "/";
        tagObservations[i] =
            new RawTagObservation(
                table.get(p + "TagId", -1),
                table.get(p + "TxRad", 0.0),
                table.get(p + "TyRad", 0.0),
                table.get(p + "DistanceMeters", 0.0),
                table.get(p + "Ambiguity", 0.0));
      }
    }
  }

  default void updateInputs(VisionIOInputs inputs) {}

  default void setTransform(Transform3d robotToCamera) {}

  default void setRobotOrientation(
      Rotation2d yaw, double yawRateRadPerSec, boolean gyroSeeded) {}
}
