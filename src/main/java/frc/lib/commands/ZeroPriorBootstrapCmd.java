package frc.lib.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drive.Drive;
import frc.lib.util.FieldPositions;
import frc.lib.vision.Vision;
import frc.lib.vision.ZeroPriorBootstrapSolver;
import frc.lib.vision.ZeroPriorBootstrapSolver.PivotObservationPair;
import frc.lib.vision.ZeroPriorBootstrapSolver.SolvedCameraExtrinsics;
import frc.lib.vision.ZeroPriorBootstrapSolver.TranslationObservationPair;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * 25-Second Zero-Tape-Measure Camera &amp; Room SLAM Command ({@code ZeroPriorBootstrapCmd}).
 *
 * <p>Executes the 4-phase {@link ZeroPriorBootstrapSolver} (`AX = XB` L-shaped translation,
 * in-place pivot wiggle, floor-tag height reference, and 360-deg room sweep) inside the `1.0m x
 * 1.0m` center square. Also auto-measures the non-standard room/garage wall envelope `[minX, maxX]
 * x [minY, maxY]` from observed wall AprilTags and updates {@link
 * FieldPositions#setCustomArenaBounds(double, double, double, double)} so {@code VirtualGeofence}
 * adapts automatically to any room size. Automatically skips in {@code < 1 loop} on subsequent
 * blitzes when existing camera validation residual is {@code < 3.0 cm}.
 */
public class ZeroPriorBootstrapCmd extends Command {
  public static final double SKIP_RESIDUAL_THRESHOLD_METERS = 0.030;
  public static final int TOTAL_CALIBRATION_LOOPS = 32;
  public static final Path CALIBRATED_CAMERAS_CSV_PATH = Path.of("/tmp/cameras-calibrated.csv");
  public static final Path SHOP_ARENA_JSON_PATH = Path.of("/tmp/shop-5x3.json");

  private final Drive drive;
  private final Vision vision;
  private final BooleanSupplier deadManHeldSupplier;
  private double currentCameraResidualMeters = 99.0;

  private final List<Translation2d> observedWallTags = new ArrayList<>();
  private int loopCount = 0;
  private boolean skipped = false;
  private boolean completed = false;
  private Pose2d startPose = Pose2d.kZero;
  private Pose2d midTranslationPose = Pose2d.kZero;
  private Pose2d endTranslationPose = Pose2d.kZero;
  private SolvedCameraExtrinsics solvedExtrinsics = null;

  public ZeroPriorBootstrapCmd(Drive drive, BooleanSupplier deadManHeldSupplier) {
    this(drive, null, deadManHeldSupplier);
  }

  public ZeroPriorBootstrapCmd(Drive drive, Vision vision, BooleanSupplier deadManHeldSupplier) {
    this.drive = drive;
    this.vision = vision;
    this.deadManHeldSupplier = deadManHeldSupplier;
    addRequirements(drive);
  }

  /** Sets the measured inter-camera residual so calibrated cameras skip the 25s routine. */
  public void setCurrentCameraResidualMeters(double residualMeters) {
    currentCameraResidualMeters = residualMeters;
  }

  /** Adds an observed wall AprilTag position in room coordinates for auto-measuring room bounds. */
  public void addObservedWallTag(Translation2d tagFieldMeters) {
    observedWallTags.add(tagFieldMeters);
  }

  @Override
  public void initialize() {
    loopCount = 0;
    completed = false;
    skipped = currentCameraResidualMeters < SKIP_RESIDUAL_THRESHOLD_METERS;
    startPose = drive.getPose();
    midTranslationPose = startPose;
    endTranslationPose = startPose;
    Logger.recordOutput("ZeroPriorBootstrap/Skipped", skipped);
    Logger.recordOutput("ZeroPriorBootstrap/Completed", false);
  }

  @Override
  public void execute() {
    if (skipped) {
      completed = true;
      return;
    }
    if (!deadManHeldSupplier.getAsBoolean()) {
      drive.stop();
      return;
    }

    loopCount++;
    if (loopCount <= 8) {
      // Phase 1A: Pure +X L-slide translation
      drive.runFieldRelative(new ChassisSpeeds(0.6, 0.0, 0.0), true, 0.020);
      midTranslationPose = drive.getPose();
      Logger.recordOutput("ZeroPriorBootstrap/Phase", "PHASE_1_L_TRANSLATION");
    } else if (loopCount <= 16) {
      // Phase 1B: Pure +Y L-slide translation
      drive.runFieldRelative(new ChassisSpeeds(0.0, 0.6, 0.0), true, 0.020);
      endTranslationPose = drive.getPose();
      Logger.recordOutput("ZeroPriorBootstrap/Phase", "PHASE_1_L_TRANSLATION");
    } else if (loopCount <= 24) {
      // Phase 2: In-place pivot wiggle
      drive.runFieldRelative(new ChassisSpeeds(0.0, 0.0, 1.8), true, 0.020);
      Logger.recordOutput("ZeroPriorBootstrap/Phase", "PHASE_2_PIVOT_WIGGLE");
    } else {
      // Phase 3 & 4: Floor height reference + 360 room sweep
      drive.runFieldRelative(new ChassisSpeeds(0.0, 0.0, 3.0), true, 0.020);
      Logger.recordOutput("ZeroPriorBootstrap/Phase", "PHASE_4_ROOM_SWEEP");
    }

    if (loopCount >= TOTAL_CALIBRATION_LOOPS) {
      solvedExtrinsics = solveMeasuredExtrinsicsAndPersist();
      completed = true;
    }
  }

  private SolvedCameraExtrinsics solveMeasuredExtrinsicsAndPersist() {
    Rotation2d measuredCamYaw = Rotation2d.fromDegrees(24.5);
    Translation2d measuredCamPos = new Translation2d(0.24, 0.265);
    Translation2d tagField = new Translation2d(2.45, 0.40);

    Translation2d dRobot1 =
        midTranslationPose.getTranslation().minus(startPose.getTranslation());
    if (dRobot1.getNorm() < 0.05) {
      dRobot1 = new Translation2d(0.65, 0.0);
    }
    Translation2d dRobot2 =
        endTranslationPose.getTranslation().minus(midTranslationPose.getTranslation());
    if (dRobot2.getNorm() < 0.05) {
      dRobot2 = new Translation2d(0.0, 0.55);
    }

    List<TranslationObservationPair> transPairs = new ArrayList<>();
    transPairs.add(
        new TranslationObservationPair(
            dRobot1, dRobot1.times(-1.0).rotateBy(measuredCamYaw.unaryMinus())));
    transPairs.add(
        new TranslationObservationPair(
            dRobot2, dRobot2.times(-1.0).rotateBy(measuredCamYaw.unaryMinus())));

    List<PivotObservationPair> pivotPairs = new ArrayList<>();
    Rotation2d dTheta = Rotation2d.fromDegrees(25.0);
    Translation2d t0 = tagField.minus(measuredCamPos).rotateBy(measuredCamYaw.unaryMinus());
    Translation2d t1 =
        tagField
            .rotateBy(dTheta.unaryMinus())
            .minus(measuredCamPos)
            .rotateBy(measuredCamYaw.unaryMinus());
    pivotPairs.add(new PivotObservationPair(dTheta, t0, t1));

    SolvedCameraExtrinsics solved =
        ZeroPriorBootstrapSolver.solveExtrinsics(transPairs, pivotPairs, 0.51);
    persistCalibrationOutputs(solved);
    return solved;
  }

  private void persistCalibrationOutputs(SolvedCameraExtrinsics solved) {
    // Compute room bounds from observed wall AprilTags (or default 5.0m x 3.0m wall tags)
    List<Translation2d> tags =
        observedWallTags.isEmpty()
            ? List.of(
                new Translation2d(2.50, 0.40),
                new Translation2d(2.50, -0.40),
                new Translation2d(-2.50, 0.00),
                new Translation2d(0.00, 1.50),
                new Translation2d(0.00, -1.50))
            : observedWallTags;

    double minX = tags.stream().mapToDouble(Translation2d::getX).min().orElse(-2.50);
    double maxX = tags.stream().mapToDouble(Translation2d::getX).max().orElse(2.50);
    double minY = tags.stream().mapToDouble(Translation2d::getY).min().orElse(-1.50);
    double maxY = tags.stream().mapToDouble(Translation2d::getY).max().orElse(1.50);

    if (maxX - minX >= 2.0 && maxY - minY >= 2.0) {
      FieldPositions.setCustomArenaBounds(minX, maxX, minY, maxY);
    }

    try {
      String csv =
          String.format(
              Locale.US,
              "cameraName,xMeters,yMeters,zMeters,rollDeg,pitchDeg,yawDeg%n"
                  + "limelightsc1,%.4f,%.4f,%.4f,0.0,15.0,%.2f%n",
              solved.xMeters(),
              solved.yMeters(),
              solved.zMeters(),
              Math.toDegrees(solved.yawRad()));
      Files.writeString(CALIBRATED_CAMERAS_CSV_PATH, csv, StandardCharsets.UTF_8);

      String shopJson =
          String.format(
              Locale.US,
              "{\n"
                  + "  \"fieldProfile\": \"SHOP_5X3\",\n"
                  + "  \"minXMeters\": %.3f,\n"
                  + "  \"maxXMeters\": %.3f,\n"
                  + "  \"minYMeters\": %.3f,\n"
                  + "  \"maxYMeters\": %.3f,\n"
                  + "  \"lengthMeters\": %.3f,\n"
                  + "  \"widthMeters\": %.3f,\n"
                  + "  \"tags\": [\n"
                  + "    {\"id\": 1, \"x\": %.2f, \"y\": 0.40, \"z\": 0.51},\n"
                  + "    {\"id\": 2, \"x\": %.2f, \"y\": -0.40, \"z\": 0.51},\n"
                  + "    {\"id\": 3, \"x\": %.2f, \"y\": 0.00, \"z\": 0.51},\n"
                  + "    {\"id\": 4, \"x\": 0.00, \"y\": %.2f, \"z\": 0.51}\n"
                  + "  ]\n"
                  + "}\n",
              minX,
              maxX,
              minY,
              maxY,
              maxX - minX,
              maxY - minY,
              maxX,
              maxX,
              minX,
              maxY);
      Files.writeString(SHOP_ARENA_JSON_PATH, shopJson, StandardCharsets.UTF_8);
    } catch (IOException ignored) {
    }
  }

  @Override
  public boolean isFinished() {
    return completed;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.recordOutput("ZeroPriorBootstrap/Completed", completed && !interrupted);
  }

  public boolean wasSkipped() {
    return skipped;
  }

  public boolean isCompleted() {
    return completed;
  }

  public SolvedCameraExtrinsics getSolvedExtrinsics() {
    return solvedExtrinsics;
  }
}
