package frc.lib.util;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.FieldPositions.CoordinateSystem;
import frc.lib.util.FieldPositions.FieldProfile;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.littletonrobotics.junction.Logger;

/**
 * Pre-flight and runtime autonomous path safety validator, deterministic PathPlanner {@code .path}
 * follower, and command decorator using pure AdvantageKit {@code Logger.recordOutput("Auto/...",
 * ...)} telemetry (zero SmartDashboard).
 */
public final class SafeAutoBuilder {

  private static volatile String lastStatus = "READY";
  private static volatile String lastCancelReason = "";

  private SafeAutoBuilder() {}

  /** Resets the in-memory diagnostic state prior to simulating or running an autonomous routine. */
  public static void clearLastDiagnostic() {
    lastStatus = "READY";
    lastCancelReason = "";
  }

  /** Returns the most recent {@code Auto/Status} string recorded by {@link SafeAutoBuilder}. */
  public static String getLastStatus() {
    return lastStatus;
  }

  /**
   * Returns the most recent block/cancel reason recorded by {@link SafeAutoBuilder}, or {@code ""}
   * if all paths passed pre-flight and runtime checks.
   */
  public static String getLastCancelReason() {
    return lastCancelReason;
  }

  private record PathRotationStop(double waypointPos, Rotation2d rotation) {}

  private record PathEventStop(double waypointPos, String commandName) {}

  /** Configurable pre-flight and runtime path safety thresholds. */
  public record Limits(
      double maxVelocityMps,
      double maxPositionErrorM,
      double maxStartErrorM,
      double speedOvershootFraction,
      double minEffectiveSpeedMps,
      double collisionDecelMps2,
      double collisionMinSpeedMps,
      int collisionCyclesRequired) {

    public static final Limits DEFAULT = new Limits(4.8, 1.5, 0.50, 0.25, 0.25, 10.0, 0.5, 5);
  }

  /** Result of deterministic pre-flight path validation. */
  public record PreflightReport(
      boolean passed,
      double startErrorMeters,
      boolean insideFieldBounds,
      String statusMessage) {}

  /**
   * Audits a candidate autonomous path against starting pose alignment, velocity bounds, and active
   * {@link FieldPositions} field/shop boundaries.
   */
  public static PreflightReport auditPath(
      List<Pose2d> pathPoses,
      Pose2d currentRobotPose,
      double pathMaxVelMps,
      Limits limits,
      CoordinateSystem sys,
      FieldProfile profile) {
    if (pathPoses == null || pathPoses.isEmpty()) {
      lastCancelReason = "Empty path poses";
      lastStatus = "BLOCKED: Empty path poses";
      return new PreflightReport(false, 0.0, false, "Empty path poses");
    }

    if (pathMaxVelMps > limits.maxVelocityMps()) {
      String msg =
          String.format(
              Locale.US,
              "Path velocity %.2f m/s exceeds limit %.2f m/s",
              pathMaxVelMps,
              limits.maxVelocityMps());
      lastCancelReason = msg;
      lastStatus = "BLOCKED: " + msg;
      Logger.recordOutput("Auto/PreflightPassed", false);
      Logger.recordOutput("Auto/CancelReason", msg);
      return new PreflightReport(false, 0.0, true, msg);
    }

    if (pathMaxVelMps < limits.minEffectiveSpeedMps()) {
      String msg =
          String.format(
              Locale.US,
              "Path velocity %.2f m/s below deadband floor %.2f m/s",
              pathMaxVelMps,
              limits.minEffectiveSpeedMps());
      lastCancelReason = msg;
      lastStatus = "BLOCKED: " + msg;
      Logger.recordOutput("Auto/PreflightPassed", false);
      Logger.recordOutput("Auto/CancelReason", msg);
      return new PreflightReport(false, 0.0, true, msg);
    }

    Pose2d expectedStart = pathPoses.get(0);
    double startErrM =
        currentRobotPose.getTranslation().getDistance(expectedStart.getTranslation());
    Logger.recordOutput("Auto/StartErrorM", startErrM);
    if (startErrM > limits.maxStartErrorM()) {
      String msg =
          String.format(
              Locale.US,
              "Start pose error %.2f m exceeds limit %.2f m (robot at (%.2f, %.2f), path expects (%.2f, %.2f) — call drive.setPose(...) first)",
              startErrM,
              limits.maxStartErrorM(),
              currentRobotPose.getX(),
              currentRobotPose.getY(),
              expectedStart.getX(),
              expectedStart.getY());
      lastCancelReason = msg;
      lastStatus = "BLOCKED: " + msg;
      Logger.recordOutput("Auto/PreflightPassed", false);
      Logger.recordOutput("Auto/CancelReason", msg);
      return new PreflightReport(false, startErrM, true, msg);
    }

    for (Pose2d p : pathPoses) {
      if (!FieldPositions.isInsideField(p.getX(), p.getY(), 0.20, sys, profile)) {
        String msg =
            String.format(
                Locale.US,
                "Waypoint (%.2f, %.2f) exits %s bounds",
                p.getX(),
                p.getY(),
                profile.name());
        lastCancelReason = msg;
        lastStatus = "BLOCKED: " + msg;
        Logger.recordOutput("Auto/PreflightPassed", false);
        Logger.recordOutput("Auto/CancelReason", msg);
        return new PreflightReport(false, startErrM, false, msg);
      }
    }

    lastStatus = "READY";
    Logger.recordOutput("Auto/PreflightPassed", true);
    Logger.recordOutput("Auto/CancelReason", "");
    return new PreflightReport(true, startErrM, true, "READY");
  }

  /** Wraps an autonomous command with pre-flight and runtime path/collision monitoring. */
  public static Command wrap(
      Command innerCommand,
      List<Pose2d> pathPoses,
      double pathMaxVelMps,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> velocitySupplier,
      Consumer<ChassisSpeeds> stopConsumer,
      Limits limits) {
    return new Command() {
      private boolean cancelled = false;
      private double prevSpeed = 0.0;
      private int collisionCycles = 0;

      {
        addRequirements(innerCommand.getRequirements().toArray(new Subsystem[0]));
      }

      @Override
      public void initialize() {
        cancelled = false;
        prevSpeed = 0.0;
        collisionCycles = 0;
        PreflightReport report =
            auditPath(
                pathPoses,
                poseSupplier.get(),
                pathMaxVelMps,
                limits,
                FieldPositions.getCoordinateSystem(),
                FieldPositions.getFieldProfile());
        if (!report.passed()) {
          cancelled = true;
          lastCancelReason = report.statusMessage();
          lastStatus = "BLOCKED: " + report.statusMessage();
          DriverStation.reportWarning("[SafeAuto] " + report.statusMessage(), false);
          Logger.recordOutput("Auto/Status", "BLOCKED: " + report.statusMessage());
          return;
        }
        lastStatus = "Running";
        Logger.recordOutput("Auto/Status", "Running");
        innerCommand.initialize();
      }

      @Override
      public void execute() {
        if (cancelled) {
          return;
        }
        innerCommand.execute();

        ChassisSpeeds speeds = velocitySupplier.get();
        double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double allowedSpeed = pathMaxVelMps * (1.0 + limits.speedOvershootFraction());
        Logger.recordOutput("Auto/CurrentSpeedMps", speed);

        if (speed > allowedSpeed) {
          cancelled = true;
          lastCancelReason = "Speed overshoot";
          lastStatus = "CANCELLED: Speed overshoot";
          Logger.recordOutput("Auto/Status", "CANCELLED: Speed overshoot");
          stopConsumer.accept(new ChassisSpeeds());
          return;
        }

        double decel = (prevSpeed - speed) / 0.020;
        if (decel > limits.collisionDecelMps2() && prevSpeed > limits.collisionMinSpeedMps()) {
          collisionCycles++;
        } else {
          collisionCycles = 0;
        }
        prevSpeed = speed;
        if (collisionCycles >= limits.collisionCyclesRequired()) {
          cancelled = true;
          lastCancelReason = "Collision detected";
          lastStatus = "CANCELLED: Collision detected";
          Logger.recordOutput("Auto/Status", "CANCELLED: Collision detected");
          stopConsumer.accept(new ChassisSpeeds());
        }
      }

      @Override
      public boolean isFinished() {
        return cancelled || innerCommand.isFinished();
      }

      @Override
      public void end(boolean interrupted) {
        if (!cancelled) {
          innerCommand.end(interrupted);
          lastStatus = interrupted ? "Interrupted" : "Done";
          Logger.recordOutput("Auto/Status", interrupted ? "Interrupted" : "Done");
        } else {
          stopConsumer.accept(new ChassisSpeeds());
        }
      }
    };
  }

  /**
   * Builds a PathPlanner path-following command wrapped in pre-flight and runtime
   * collision/overshoot guards. Steps deterministically along {@link
   * PathPlannerPath#getPathPoses()} at {@code loopPeriodSecSupplier} per cycle while interpolating
   * {@code rotationTargets} and triggering {@link NamedCommands} for {@code eventMarkers}, so
   * execution is identical on the real robot and in fast-forward {@code ./gradlew renderAuto}
   * simulation.
   */
  public static Command followPathCommand(
      String pathName,
      Subsystem driveSubsystem,
      DoubleSupplier loopPeriodSecSupplier,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> velocitySupplier,
      Consumer<ChassisSpeeds> robotRelativeVelocityConsumer) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      String rawJson = readPathJson(pathName);
      boolean localOrigin = isLocalOriginPath(rawJson);

      List<Pose2d> poses = transformPathPosesForActiveSystem(path.getPathPoses(), localOrigin);
      double maxVel = path.getGlobalConstraints().maxVelocityMPS();
      double maxAccel = path.getGlobalConstraints().maxAccelerationMPSSq();
      Rotation2d endRotation =
          transformRotationForActiveSystem(path.getGoalEndState().rotation(), localOrigin);
      Rotation2d rawStartRotation =
          path.getStartingHolonomicPose().map(Pose2d::getRotation).orElse(Rotation2d.kZero);
      Rotation2d startRotation = transformRotationForActiveSystem(rawStartRotation, localOrigin);

      int numSegments = Math.max(1, countWaypoints(rawJson) - 1);
      List<PathRotationStop> rotStops =
          parseRotationStops(rawJson, numSegments, startRotation, endRotation, localOrigin);
      List<PathEventStop> eventStops = parseEventStops(rawJson);

      Command rawCmd =
          createDeterministicPathCommand(
              pathName,
              poses,
              maxVel,
              maxAccel,
              numSegments,
              rotStops,
              eventStops,
              driveSubsystem,
              loopPeriodSecSupplier,
              poseSupplier,
              robotRelativeVelocityConsumer);
      return wrap(
          rawCmd,
          poses,
          maxVel,
          poseSupplier,
          velocitySupplier,
          robotRelativeVelocityConsumer,
          Limits.DEFAULT);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner path: " + pathName, e.getStackTrace());
      Logger.recordOutput("Auto/Status", "LOAD_ERROR: " + pathName);
      return Commands.none();
    }
  }

  /** Returns the expected starting pose of a PathPlanner path if present on disk. */
  public static Optional<Pose2d> getPathStartingPose(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      boolean localOrigin = isLocalOriginPath(readPathJson(pathName));
      Optional<Pose2d> holonomicStart = path.getStartingHolonomicPose();
      if (holonomicStart.isPresent()) {
        return Optional.of(transformPoseForActiveSystem(holonomicStart.get(), localOrigin));
      }
      List<Pose2d> poses = path.getPathPoses();
      return poses.isEmpty()
          ? Optional.empty()
          : Optional.of(transformPoseForActiveSystem(poses.get(0), localOrigin));
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  /**
   * Wraps {@code command} with a deterministic 50 Hz (20 ms) loop-count timeout so autonomous
   * sequences advance identically in both real-time 50 Hz execution and fast-forward unit-test
   * simulation without mutating global HAL clock state.
   */
  public static Command withLoopTimeout(Command command, double timeoutSeconds) {
    final int maxLoops = Math.max(1, (int) Math.round(timeoutSeconds / 0.020));
    final int[] loops = {0};
    return command
        .beforeStarting(() -> loops[0] = 0)
        .until(
            () -> {
              loops[0]++;
              return loops[0] >= maxLoops;
            });
  }

  /**
   * Pauses a command sequence for {@code durationSeconds} using deterministic 50 Hz (20 ms) loop
   * counting rather than wall-clock {@code Commands.waitSeconds}, advancing identically on the real
   * robot and in fast-forward {@code ./gradlew renderAuto} simulation.
   */
  public static Command waitLoopSeconds(double durationSeconds) {
    if (durationSeconds <= 0.0) {
      return Commands.none();
    }
    return withLoopTimeout(Commands.idle(), durationSeconds);
  }

  private static Command createDeterministicPathCommand(
      String pathName,
      List<Pose2d> poses,
      double maxVel,
      double maxAccel,
      int numSegments,
      List<PathRotationStop> rotStops,
      List<PathEventStop> eventStops,
      Subsystem driveSubsystem,
      DoubleSupplier loopPeriodSecSupplier,
      Supplier<Pose2d> poseSupplier,
      Consumer<ChassisSpeeds> robotRelativeVelocityConsumer) {
    final int n = poses.size();
    final double[] cumDist = new double[n];
    for (int i = 1; i < n; i++) {
      cumDist[i] =
          cumDist[i - 1]
              + poses.get(i).getTranslation().getDistance(poses.get(i - 1).getTranslation());
    }
    final double totalLength = (n > 0) ? cumDist[n - 1] : 0.0;

    return new Command() {
      private double sMeters = 0.0;
      private double speedMps = 0.35;
      private int loops = 0;
      private final boolean[] eventFired = new boolean[eventStops.size()];

      {
        addRequirements(driveSubsystem);
      }

      @Override
      public void initialize() {
        sMeters = 0.0;
        speedMps = 0.35;
        loops = 0;
        for (int i = 0; i < eventFired.length; i++) {
          eventFired[i] = false;
        }
        Logger.recordOutput("Auto/ActivePath", pathName);
      }

      @Override
      public void execute() {
        if (n < 2 || totalLength < 1e-4) {
          robotRelativeVelocityConsumer.accept(new ChassisSpeeds());
          return;
        }
        loops++;
        double dt = loopPeriodSecSupplier.getAsDouble();

        int segIdx = 0;
        while (segIdx + 1 < n - 1 && cumDist[segIdx + 1] < sMeters) {
          segIdx++;
        }
        double segLen = Math.max(1e-5, cumDist[segIdx + 1] - cumDist[segIdx]);
        double frac = MathUtil.clamp((sMeters - cumDist[segIdx]) / segLen, 0.0, 1.0);
        double waypointRelPos = ((segIdx + frac) / (double) (n - 1)) * numSegments;

        for (int e = 0; e < eventStops.size(); e++) {
          if (!eventFired[e] && waypointRelPos >= eventStops.get(e).waypointPos() - 0.02) {
            eventFired[e] = true;
            String cmdName = eventStops.get(e).commandName();
            Logger.recordOutput("Auto/LastEventMarker", cmdName);
            if (NamedCommands.hasCommand(cmdName)) {
              Command eventCmd = NamedCommands.getCommand(cmdName);
              eventCmd.initialize();
              eventCmd.execute();
              eventCmd.end(false);
            }
          }
        }

        Translation2d p0 = poses.get(segIdx).getTranslation();
        Translation2d p1 = poses.get(segIdx + 1).getTranslation();
        Translation2d dSeg = p1.minus(p0);
        double dNorm = Math.max(1e-5, dSeg.getNorm());
        double tx = dSeg.getX() / dNorm;
        double ty = dSeg.getY() / dNorm;

        // Reduce cruise speed slightly through tight mid-path intake turns (e.g. Depot U-turn)
        double localMaxVel =
            (numSegments >= 4 && waypointRelPos >= 0.85 && waypointRelPos <= 3.60)
                ? Math.min(maxVel, 2.20)
                : maxVel;
        double remDist = Math.max(0.0, totalLength - sMeters);
        double brakeVel = Math.sqrt(2.0 * maxAccel * remDist);
        double targetSpeed = Math.min(localMaxVel, brakeVel);

        if (speedMps < targetSpeed) {
          speedMps = Math.min(targetSpeed, speedMps + maxAccel * dt);
        } else {
          speedMps = Math.max(targetSpeed, speedMps - maxAccel * dt);
        }
        sMeters = Math.min(totalLength, sMeters + Math.max(0.15, speedMps) * dt);

        int nextSeg = segIdx;
        while (nextSeg + 1 < n - 1 && cumDist[nextSeg + 1] < sMeters) {
          nextSeg++;
        }
        double nextSegLen = Math.max(1e-5, cumDist[nextSeg + 1] - cumDist[nextSeg]);
        double nextFrac = MathUtil.clamp((sMeters - cumDist[nextSeg]) / nextSegLen, 0.0, 1.0);
        Translation2d targetTranslation =
            poses
                .get(nextSeg)
                .getTranslation()
                .interpolate(poses.get(nextSeg + 1).getTranslation(), nextFrac);
        Rotation2d targetHeading = interpolateHeading(rotStops, waypointRelPos);

        Pose2d currentPose = poseSupplier.get();
        double vxCmd = tx * speedMps + 3.2 * (targetTranslation.getX() - currentPose.getX());
        double vyCmd = ty * speedMps + 3.2 * (targetTranslation.getY() - currentPose.getY());
        double cmdMag = Math.hypot(vxCmd, vyCmd);
        if (cmdMag > maxVel) {
          vxCmd *= (maxVel / cmdMag);
          vyCmd *= (maxVel / cmdMag);
        }

        double yawErr =
            MathUtil.angleModulus(targetHeading.minus(currentPose.getRotation()).getRadians());
        double omegaCmd = MathUtil.clamp(yawErr * 5.5, -5.5, 5.5);

        robotRelativeVelocityConsumer.accept(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vxCmd, vyCmd, omegaCmd, currentPose.getRotation()));
      }

      @Override
      public boolean isFinished() {
        if (n < 2) {
          return true;
        }
        double endDist =
            poseSupplier.get().getTranslation().getDistance(poses.get(n - 1).getTranslation());
        double dt = loopPeriodSecSupplier.getAsDouble();
        int maxLoops = (int) Math.ceil(((totalLength / 0.80) + 2.5) / dt);
        return (sMeters >= totalLength - 0.02 && endDist < 0.18) || loops >= maxLoops;
      }

      @Override
      public void end(boolean interrupted) {
        robotRelativeVelocityConsumer.accept(new ChassisSpeeds());
      }
    };
  }

  private static String readPathJson(String pathName) {
    try {
      Path deployPath =
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("pathplanner/paths/" + pathName + ".path");
      if (Files.exists(deployPath)) {
        return Files.readString(deployPath, StandardCharsets.UTF_8);
      }
      Path localPath = Paths.get("src/main/deploy/pathplanner/paths/" + pathName + ".path");
      if (Files.exists(localPath)) {
        return Files.readString(localPath, StandardCharsets.UTF_8);
      }
    } catch (Exception ignored) {
    }
    return "";
  }

  private static int countWaypoints(String json) {
    int count = 0;
    int idx = 0;
    while ((idx = json.indexOf("\"anchor\"", idx)) >= 0) {
      count++;
      idx += 8;
    }
    return Math.max(2, count);
  }

  private static final Pattern LOCAL_ORIGIN_FOLDER_PATTERN =
      Pattern.compile("\"folder\"\\s*:\\s*\"LocalOrigin\"");

  private static boolean isLocalOriginPath(String json) {
    return json != null && LOCAL_ORIGIN_FOLDER_PATTERN.matcher(json).find();
  }

  private static Pose2d transformPoseForActiveSystem(Pose2d wallBluePose, boolean localOrigin) {
    return localOrigin ? wallBluePose : FieldPositions.fromWallBlue(wallBluePose);
  }

  private static Rotation2d transformRotationForActiveSystem(
      Rotation2d wallBlueRot, boolean localOrigin) {
    if (localOrigin || FieldPositions.getCoordinateSystem() == CoordinateSystem.WALL_BLUE_2026) {
      return wallBlueRot;
    }
    return wallBlueRot.plus(Rotation2d.kPi);
  }

  private static List<Pose2d> transformPathPosesForActiveSystem(
      List<Pose2d> rawPoses, boolean localOrigin) {
    if (localOrigin || FieldPositions.getCoordinateSystem() == CoordinateSystem.WALL_BLUE_2026) {
      return rawPoses;
    }
    List<Pose2d> transformed = new ArrayList<>(rawPoses.size());
    for (Pose2d pose : rawPoses) {
      transformed.add(FieldPositions.fromWallBlue(pose));
    }
    return transformed;
  }

  private static List<PathRotationStop> parseRotationStops(
      String json,
      int numSegments,
      Rotation2d startRot,
      Rotation2d endRot,
      boolean localOrigin) {
    List<PathRotationStop> stops = new ArrayList<>();
    stops.add(new PathRotationStop(0.0, startRot));
    Pattern p =
        Pattern.compile(
            "\"waypointRelativePos\"\\s*:\\s*([0-9.]+)\\s*,\\s*\"rotationDegrees\"\\s*:\\s*([-0-9.]+)");
    Matcher m = p.matcher(json);
    while (m.find()) {
      double pos = Double.parseDouble(m.group(1));
      double deg = Double.parseDouble(m.group(2));
      stops.add(
          new PathRotationStop(
              pos, transformRotationForActiveSystem(Rotation2d.fromDegrees(deg), localOrigin)));
    }
    stops.add(new PathRotationStop(numSegments, endRot));
    return stops;
  }

  private static List<PathEventStop> parseEventStops(String json) {
    List<PathEventStop> events = new ArrayList<>();
    int markerIdx = json.indexOf("\"eventMarkers\"");
    if (markerIdx < 0) {
      return events;
    }
    String sub = json.substring(markerIdx);
    Pattern p =
        Pattern.compile(
            "\"name\"\\s*:\\s*\"([^\"]+)\"\\s*,\\s*\"waypointRelativePos\"\\s*:\\s*([0-9.]+)");
    Matcher m = p.matcher(sub);
    while (m.find()) {
      events.add(new PathEventStop(Double.parseDouble(m.group(2)), m.group(1)));
    }
    return events;
  }

  private static Rotation2d interpolateHeading(List<PathRotationStop> stops, double waypointPos) {
    if (stops.isEmpty()) {
      return Rotation2d.kZero;
    }
    for (int i = 0; i + 1 < stops.size(); i++) {
      PathRotationStop s0 = stops.get(i);
      PathRotationStop s1 = stops.get(i + 1);
      if (waypointPos >= s0.waypointPos() && waypointPos <= s1.waypointPos()) {
        double span = Math.max(1e-4, s1.waypointPos() - s0.waypointPos());
        double t = MathUtil.clamp((waypointPos - s0.waypointPos()) / span, 0.0, 1.0);
        return s0.rotation().interpolate(s1.rotation(), t);
      }
    }
    return stops.get(stops.size() - 1).rotation();
  }
}
