package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.util.FieldPositions;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Native SystemCore USB 3.0 Limelight OS (`limelightsc0`..`limelightsc3` on physical ports `USB
 * 0..3`) implementation of {@link VisionIO}.
 *
 * <p>Uses unconstrained 3D PnP (`MegaTag1`) while disabled/unseeded and switches to gyro-constrained
 * `MegaTag2` once gyro heading is seeded. Drains the full NT4 pose queue every cycle.
 */
public class VisionIOLimelightSystemCore implements VisionIO {
  private final CameraSpec config;
  private final DoubleArrayPublisher orientationPub;
  private final DoubleArraySubscriber mt1Sub;
  private final DoubleArraySubscriber mt2Sub;
  private final Deque<PoseObservation> injectedQueue = new ArrayDeque<>();
  private final double[] orientationBuffer = new double[6];

  private boolean gyroSeeded = false;
  private boolean seenValidPacket = false;
  private Transform3d robotToCamera;

  public VisionIOLimelightSystemCore(CameraSpec config) {
    if (config.portIndex() < 0 || config.portIndex() > 3) {
      throw new IllegalArgumentException(
          "SystemCore direct USB cameras must use root ports USB 0..3 (got portIndex="
              + config.portIndex()
              + " on "
              + config.cameraName()
              + "). Never use a USB hub.");
    }
    this.config = config;
    this.robotToCamera = VisionIO.toTransform3d(config);

    String tableName =
        config.cameraName().startsWith("limelight")
            ? config.cameraName()
            : "limelightsc" + config.portIndex();
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
    this.orientationPub = table.getDoubleArrayTopic("robot_orientation_set").publish();
    this.mt1Sub = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
    this.mt2Sub = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
  }

  public CameraSpec getConfig() {
    return config;
  }

  public Transform3d getTransform() {
    return robotToCamera;
  }

  @Override
  public void setTransform(Transform3d robotToCamera) {
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void setRobotOrientation(Rotation2d yaw, double yawRateRadPerSec, boolean gyroSeeded) {
    this.gyroSeeded = gyroSeeded;
    orientationBuffer[0] = yaw.getDegrees();
    orientationBuffer[1] = Units.radiansToDegrees(yawRateRadPerSec);
    orientationPub.set(orientationBuffer);
  }

  /** Injects synthetic queued observations for deterministic multi-frame queue draining tests. */
  public void injectObservationsForTest(PoseObservation... observations) {
    for (PoseObservation obs : observations) {
      injectedQueue.addLast(obs);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    List<PoseObservation> observations = new ArrayList<>();
    Set<Integer> tagIds = new HashSet<>();

    while (!injectedQueue.isEmpty()) {
      observations.add(injectedQueue.removeFirst());
    }

    DoubleArraySubscriber activeSub = gyroSeeded ? mt2Sub : mt1Sub;
    DoubleArraySubscriber inactiveSub = gyroSeeded ? mt1Sub : mt2Sub;
    inactiveSub.readQueue();
    ObservationType obsType =
        gyroSeeded ? ObservationType.MEGATAG_2 : ObservationType.MEGATAG_1;

    for (TimestampedDoubleArray sample : activeSub.readQueue()) {
      double[] raw = sample.value;
      if (raw.length < 11) {
        continue;
      }
      seenValidPacket = true;
      int tagCount = (int) raw[7];
      if (tagCount <= 0) {
        continue;
      }
      double latencySec = raw[6] / 1000.0;
      double timestampSec = (sample.timestamp / 1.0e6) - latencySec;
      Pose2d activePose2d =
          FieldPositions.fromWallBlue(
              new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5])));
      Pose3d pose =
          new Pose3d(
              activePose2d.getX(),
              activePose2d.getY(),
              raw[2],
              new Rotation3d(
                  Units.degreesToRadians(raw[3]),
                  Units.degreesToRadians(raw[4]),
                  activePose2d.getRotation().getRadians()));
      double avgDist = raw[9];
      double ambiguity = (raw.length >= 18 && tagCount == 1) ? raw[17] : 0.0;
      observations.add(
          new PoseObservation(timestampSec, pose, ambiguity, tagCount, avgDist, obsType));

      for (int i = 11; i + 6 < raw.length; i += 7) {
        tagIds.add((int) raw[i]);
      }
    }

    inputs.connected = !observations.isEmpty() || seenValidPacket || activeSub.getLastChange() > 0;
    inputs.framesDrainedThisCycle = observations.size();
    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
    inputs.tagObservations = new RawTagObservation[0];
    int[] sortedTags = new int[tagIds.size()];
    int idx = 0;
    for (int tagId : tagIds) {
      sortedTags[idx++] = tagId;
    }
    Arrays.sort(sortedTags);
    inputs.tagIds = sortedTags;
  }
}
