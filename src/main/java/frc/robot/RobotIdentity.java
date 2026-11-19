package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;

/**
 * Fleet robot identities for FRC Team 10413 (FLUX Robotics), supporting both SystemCore and
 * roboRIO 2.0 fallback via FluxFig, Linux device-tree serial fallback, and persistent identity tags.
 */
public enum RobotIdentity {
  COMP_2027,
  PRACTICE_2027,
  FUEL,
  CORAL,
  CHASSIS_3;

  private static final Path DEFAULT_DEVICETREE_SERIAL =
      Paths.get("/sys/firmware/devicetree/base/serial-number");
  private static final Path DEFAULT_PERSISTENT_TAG =
      Paths.get("/home/systemcore/robot-identity.txt");
  private static final String DEPLOY_TAG_FILENAME = "robot-identity.txt";

  private static volatile String lastResolutionSource = "UNINITIALIZED";

  public static RobotIdentity fromSerialNumber(String serial) {
    return fromSerialOrComments(serial, "");
  }

  public static RobotIdentity fromSerialOrComments(String serial, String comments) {
    String combined =
        ((serial == null ? "" : serial) + " " + (comments == null ? "" : comments)).trim();
    return parseExplicitToken(combined).orElse(COMP_2027);
  }

  /**
   * Resolves the active {@link RobotIdentity} across HAL serial/comments (roboRIO 2026 bridge),
   * SystemCore Linux device-tree serial, persistent on-robot tag, and deploy tag before falling back
   * to {@link #COMP_2027}.
   */
  public static RobotIdentity detect() {
    String halSerial = RobotController.getSerialNumber();
    String comments = readHalCommentsSafe();
    Path deployTag = Filesystem.getDeployDirectory().toPath().resolve(DEPLOY_TAG_FILENAME);
    return detect(halSerial, comments, DEFAULT_DEVICETREE_SERIAL, deployTag, DEFAULT_PERSISTENT_TAG);
  }

  /** Deterministic overload allowing explicit hardware paths for unit/sim tests. */
  public static RobotIdentity detect(
      String halSerial,
      String comments,
      Path devicetreeSerialPath,
      Path deployTagPath,
      Path persistentTagPath) {
    String combined =
        ((halSerial == null ? "" : halSerial) + " " + (comments == null ? "" : comments)).trim();
    Optional<RobotIdentity> fromHal = parseExplicitToken(combined);
    if (fromHal.isPresent()) {
      lastResolutionSource = "HAL_SERIAL_OR_COMMENTS";
      return fromHal.get();
    }

    Optional<RobotIdentity> fromDevTree = parseExplicitToken(readCleanText(devicetreeSerialPath));
    if (fromDevTree.isPresent()) {
      lastResolutionSource = "DEVICETREE_SERIAL";
      return fromDevTree.get();
    }

    Optional<RobotIdentity> fromPersistentTag = parseExplicitToken(readCleanText(persistentTagPath));
    if (fromPersistentTag.isPresent()) {
      lastResolutionSource = "PERSISTENT_TAG";
      return fromPersistentTag.get();
    }

    Optional<RobotIdentity> fromDeployTag = parseExplicitToken(readCleanText(deployTagPath));
    if (fromDeployTag.isPresent()) {
      lastResolutionSource = "DEPLOY_TAG";
      return fromDeployTag.get();
    }

    lastResolutionSource = "DEFAULT_COMP_2027";
    if (RobotBase.isReal() && combined.isBlank()) {
      DriverStation.reportWarning(
          "RobotIdentity: blank HAL serial and no devicetree/identity tag found; defaulting to"
              + " COMP_2027. Write 'PRACTICE_2027' to /home/systemcore/robot-identity.txt on the"
              + " practice robot to prevent silent fallback.",
          false);
    }
    return COMP_2027;
  }

  public static String getLastResolutionSource() {
    return lastResolutionSource;
  }

  static Optional<RobotIdentity> parseExplicitToken(String raw) {
    if (raw == null || raw.isBlank()) {
      return Optional.empty();
    }
    String upper = raw.replace("\0", "").trim().toUpperCase();
    if (upper.contains("CORAL")) {
      return Optional.of(CORAL);
    }
    if (upper.contains("FUEL")) {
      return Optional.of(FUEL);
    }
    if (upper.contains("CHASSIS")) {
      return Optional.of(CHASSIS_3);
    }
    if (upper.contains("PRACTICE")) {
      return Optional.of(PRACTICE_2027);
    }
    if (upper.contains("COMP_2027") || upper.equals("COMP")) {
      return Optional.of(COMP_2027);
    }
    return Optional.empty();
  }

  private static String readHalCommentsSafe() {
    try {
      // Remove getComments() after the WPILib 2027 GradleRIO cutover (deleted in allwpilib#9645).
      return RobotController.getComments();
    } catch (Throwable ignored) {
      return "";
    }
  }

  private static String readCleanText(Path path) {
    if (path == null) {
      return "";
    }
    try {
      if (!Files.exists(path)) {
        return "";
      }
      return Files.readString(path, StandardCharsets.UTF_8).replace("\0", "").trim();
    } catch (Exception ignored) {
      return "";
    }
  }
}
