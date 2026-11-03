package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Synthetic AprilTag raycaster implementation of {@link VisionIO} coupled to simulated robot pose
 * (`Supplier<Pose2d>`). Supports configurable camera mount perturbations (for testing {@link
 * ZeroPriorBootstrapSolver} and bent-mount quarantine) and multi-frame queue bursts per cycle.
 */
public class VisionIOSim implements VisionIO {
  private final CameraSpec config;
  private final Supplier<Pose2d> robotPoseSupplier;
  private Transform3d calibratedRobotToCam;
  private Transform3d mountErrorPerturbation = new Transform3d();
  private int framesPerCycle = 2; // 90-100 FPS camera produces ~2 frames per 20ms cycle
  private int visibleTagCount = 2;
  private double avgTagDistMeters = 2.6;
  private double singleTagAmbiguity = 0.08;
  private ObservationType observationType = ObservationType.MEGATAG_2;

  private static final Translation2d[] DEFAULT_SHOP_WALL_TAGS =
      new Translation2d[] {
        new Translation2d(2.45, 0.40),
        new Translation2d(2.45, -0.40),
        new Translation2d(-2.45, 0.00),
        new Translation2d(0.00, 1.45)
      };

  public VisionIOSim(CameraSpec config, Supplier<Pose2d> robotPoseSupplier) {
    this.config = config;
    this.robotPoseSupplier = robotPoseSupplier;
    this.calibratedRobotToCam = VisionIO.toTransform3d(config);
  }

  public CameraSpec getConfig() {
    return config;
  }

  @Override
  public void setTransform(Transform3d robotToCamera) {
    calibratedRobotToCam = robotToCamera;
  }

  @Override
  public void setRobotOrientation(Rotation2d yaw, double yawRateRadPerSec, boolean gyroSeeded) {
    observationType = gyroSeeded ? ObservationType.MEGATAG_2 : ObservationType.MEGATAG_1;
  }

  /** Injects a physical mount error (e.g., +12 cm bent camera bracket) for simulation tests. */
  public void setMountErrorPerturbation(Transform3d perturbation) {
    mountErrorPerturbation = (perturbation != null) ? perturbation : new Transform3d();
  }

  public void setFramesPerCycle(int framesPerCycle) {
    this.framesPerCycle = Math.max(0, framesPerCycle);
  }

  public void setVisibleTagSimulation(
      int visibleTagCount, double avgTagDistMeters, double singleTagAmbiguity) {
    this.visibleTagCount = Math.max(0, visibleTagCount);
    this.avgTagDistMeters = Math.max(0.3, avgTagDistMeters);
    this.singleTagAmbiguity = singleTagAmbiguity;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = true;
    if (visibleTagCount <= 0 || framesPerCycle <= 0) {
      inputs.framesDrainedThisCycle = 0;
      inputs.poseObservations = new PoseObservation[0];
      inputs.tagObservations = new RawTagObservation[0];
      inputs.tagIds = new int[0];
      return;
    }

    Pose2d trueRobotPose2d = robotPoseSupplier.get();
    Pose3d trueRobotPose3d = new Pose3d(trueRobotPose2d);

    // Apply any uncalibrated physical mount perturbation to the estimated robot pose
    Translation3d errTrans = mountErrorPerturbation.getTranslation();
    Rotation3d errRot = mountErrorPerturbation.getRotation();
    Pose3d measuredRobotPose =
        new Pose3d(
            trueRobotPose3d.getTranslation().plus(errTrans),
            trueRobotPose3d.getRotation().plus(errRot));

    double nowSec = Timer.getFPGATimestamp();
    PoseObservation[] poseObs = new PoseObservation[framesPerCycle];
    double ambiguity = (visibleTagCount == 1) ? singleTagAmbiguity : 0.02;

    for (int f = 0; f < framesPerCycle; f++) {
      double frameTs = nowSec - (framesPerCycle - 1 - f) * 0.011;
      poseObs[f] =
          new PoseObservation(
              frameTs,
              measuredRobotPose,
              ambiguity,
              visibleTagCount,
              avgTagDistMeters,
              observationType);
    }

    List<RawTagObservation> rawTags = new ArrayList<>();
    int[] tagIds = new int[Math.min(visibleTagCount, DEFAULT_SHOP_WALL_TAGS.length)];
    Rotation2d camYaw =
        trueRobotPose2d
            .getRotation()
            .plus(new Rotation2d(calibratedRobotToCam.getRotation().getZ()));
    for (int i = 0; i < tagIds.length; i++) {
      tagIds[i] = i + 1;
      Translation2d rel = DEFAULT_SHOP_WALL_TAGS[i].minus(trueRobotPose2d.getTranslation());
      double dist = Math.max(0.2, rel.getNorm());
      double txRad = new Rotation2d(rel.getX(), rel.getY()).minus(camYaw).getRadians();
      rawTags.add(new RawTagObservation(tagIds[i], txRad, 0.0, dist, ambiguity));
    }

    inputs.framesDrainedThisCycle = framesPerCycle;
    inputs.poseObservations = poseObs;
    inputs.tagObservations = rawTags.toArray(new RawTagObservation[0]);
    inputs.tagIds = tagIds;
  }
}
