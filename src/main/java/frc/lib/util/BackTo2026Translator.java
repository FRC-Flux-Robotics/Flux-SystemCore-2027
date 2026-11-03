package frc.lib.util;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Stream;

/**
 * Deterministic source-to-source translator between WPILib 2027 SystemCore ({@code org.wpilib.*},
 * JDK 25) and WPILib 2026 roboRIO ({@code edu.wpi.first.*}, JDK 17) for the shared {@code
 * frc.lib.*} core engine.
 *
 * <p>Used by {@code ./gradlew backTo2026} and verified in {@code Tier1ArchitectureHygieneTest} so
 * that {@code Flux-SystemCore-2027} and {@code Flux-RoboRIO-2027} remain in 1:1 parity across both
 * the pre-Beta bridge and the WPILib 2027 {@code org.wpilib.*} cutover.
 */
public final class BackTo2026Translator {
  private static final String[][] RULES_2027_TO_2026 = {
    {"org.wpilib.command2.", "edu.wpi.first.wpilibj2.command."},
    {"org.wpilib.vision.apriltag.Fields", "edu.wpi.first.apriltag.AprilTagFields"},
    {"org.wpilib.vision.apriltag.", "edu.wpi.first.apriltag."},
    {"org.wpilib.networktables.", "edu.wpi.first.networktables."},
    {"org.wpilib.math.", "edu.wpi.first.math."},
    {"org.wpilib.units.", "edu.wpi.first.units."},
    {"org.wpilib.hal.", "edu.wpi.first.hal."},
    {"org.wpilib.util.", "edu.wpi.first.util."},
    {"org.wpilib.", "edu.wpi.first.wpilibj."},
    {"ChassisVelocities", "ChassisSpeeds"},
    {"LoggedNetworkChooser", "LoggedDashboardChooser"},
    {"RobotState.isDisabled()", "DriverStation.isDisabled()"},
    {"RobotState.isEnabled()", "DriverStation.isEnabled()"},
    {"RobotState.isAutonomous()", "DriverStation.isAutonomous()"},
    {"MatchState.isFMSAttached()", "DriverStation.isFMSAttached()"},
    {"MatchState.getAlliance()", "DriverStation.getAlliance()"},
    {"Rotation2d.ZERO", "Rotation2d.kZero"},
    {"Rotation2d.PI", "Rotation2d.kPi"},
    {"Pose2d.ZERO", "Pose2d.kZero"},
    {"Translation2d.ZERO", "Translation2d.kZero"},
    {"Rotation3d.ZERO", "Rotation3d.kZero"},
    {"Alliance.RED", "Alliance.Red"},
    {"Alliance.BLUE", "Alliance.Blue"},
    {".faceDown()", ".a()"},
    {".faceRight()", ".b()"},
    {".faceLeft()", ".x()"},
    {".faceUp()", ".y()"}
  };

  private BackTo2026Translator() {}

  /**
   * Translates WPILib 2027 ({@code org.wpilib.*}) Java source text into WPILib 2026 ({@code
   * edu.wpi.first.*}) Java source text. Idempotent when applied to already-translated 2026 source.
   */
  public static String translate2027To2026(String javaSource) {
    String out = javaSource;
    for (String[] rule : RULES_2027_TO_2026) {
      out = out.replace(rule[0], rule[1]);
    }
    out =
        out.replaceAll(
            "(?<!AprilTag)\\bFields\\.k2026RebuiltWelded\\b", "AprilTagFields.k2026RebuiltWelded");
    // SystemCore indexed CAN bus factory: CANBus.systemCore(0) -> new CANBus("can_s0")
    out = out.replaceAll("CANBus\\.systemCore\\((\\d+)\\)", "new CANBus(\"can_s$1\")");
    return out;
  }

  /**
   * Inverse translator from WPILib 2026 ({@code edu.wpi.first.*}) Java source text to WPILib 2027
   * ({@code org.wpilib.*}) Java source text for round-trip verification and cutover migration.
   */
  public static String translate2026To2027(String javaSource) {
    String out = javaSource;
    for (String[] rule : RULES_2027_TO_2026) {
      out = out.replace(rule[1], rule[0]);
    }
    out = out.replace("AprilTagFields.k2026RebuiltWelded", "Fields.k2026RebuiltWelded");
    out = out.replaceAll("new CANBus\\(\"can_s(\\d+)\"\\)", "CANBus.systemCore($1)");
    return out;
  }

  /**
   * Recursively translates all {@code .java} files under {@code sourceRoot} from 2027 to 2026 and
   * writes them to the matching relative paths under {@code targetRoot}.
   *
   * @return number of {@code .java} files translated and staged
   */
  public static int translateDirectory2027To2026(Path sourceRoot, Path targetRoot)
      throws IOException {
    int count = 0;
    try (Stream<Path> walk = Files.walk(sourceRoot)) {
      List<Path> files = walk.filter(p -> Files.isRegularFile(p) && p.toString().endsWith(".java")).toList();
      for (Path srcFile : files) {
        Path rel = sourceRoot.relativize(srcFile);
        Path dstFile = targetRoot.resolve(rel);
        Files.createDirectories(dstFile.getParent());
        String raw = Files.readString(srcFile, StandardCharsets.UTF_8);
        String translated =
            srcFile.getFileName().toString().equals("BackTo2026Translator.java")
                ? raw
                : translate2027To2026(raw);
        Files.writeString(dstFile, translated, StandardCharsets.UTF_8);
        count++;
      }
    }
    return count;
  }

  /**
   * CLI entry point invoked by the {@code backTo2026} Gradle task: {@code BackTo2026Translator
   * <sourceLibDir> <targetLibDir>}.
   */
  public static void main(String[] args) throws IOException {
    if (args.length < 2) {
      throw new IllegalArgumentException(
          "Usage: BackTo2026Translator <sourceLibDir> <targetLibDir>");
    }
    Path src = Path.of(args[0]);
    Path dst = Path.of(args[1]);
    int count = translateDirectory2027To2026(src, dst);
    System.out.println(
        "BackTo2026: Translated and staged "
            + count
            + " shared frc.lib.* classes to "
            + dst.toAbsolutePath());
  }
}
