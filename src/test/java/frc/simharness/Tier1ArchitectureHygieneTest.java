package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.util.BackTo2026Translator;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/**
 * Tier 1 Verification Gate: Zero SmartDashboard/Preferences, Zero Per-Loop ControlRequest
 * Allocations, strict frc.lib vs. frc.robot separation for BackTo2026, and STUDENTS.md /
 * STUDENT-CHANGELOG.md synchronization.
 */
public class Tier1ArchitectureHygieneTest {

  @Test
  public void testZeroLegacyDashboardAndPreferencesCalls() throws IOException {
    int forbiddenCount = 0;
    try (Stream<Path> walk = Files.walk(Paths.get("src/main/java"))) {
      List<Path> files = walk.filter(p -> p.toString().endsWith(".java")).toList();
      for (Path p : files) {
        String content = Files.readString(p, StandardCharsets.UTF_8);
        if (content.contains("SmartDashboard.")
            || content.contains("SendableChooser")
            || content.contains("edu.wpi.first.wpilibj.Preferences")
            || content.contains("org.wpilib.preferences.Preferences")
            || content.contains("import java.awt.")
            || content.contains("import javax.imageio.")
            || content.contains("AutoSimVideoRenderer")) {
          forbiddenCount++;
        }
      }
    }
    PrecisionReporter.recordLowerIsBetter(
        1,
        "tier1_forbidden_dashboard_calls",
        "Forbidden SmartDashboard/Preferences",
        forbiddenCount,
        0.0,
        0.0,
        "calls");
    assertEquals(0, forbiddenCount, "SmartDashboard and Preferences are banned in 2027 code.");
  }

  @Test
  public void testZeroPerLoopControlRequestAllocations() throws IOException {
    int allocViolations = 0;
    try (Stream<Path> walk = Files.walk(Paths.get("src/main/java"))) {
      List<Path> files = walk.filter(p -> p.toString().endsWith(".java")).toList();
      for (Path p : files) {
        String content = Files.readString(p, StandardCharsets.UTF_8);
        if ((content.contains("new VelocityVoltage(")
                || content.contains("new PositionVoltage(")
                || content.contains("new MotionMagicExpoVoltage(")
                || content.contains("new VoltageOut("))
            && content.contains("periodic(")) {
          allocViolations++;
        }
        if (p.getFileName().toString().equals("ModuleIOTalonFX.java")
            && content.contains("new CurrentLimitsConfigs(")) {
          allocViolations++;
        }
        if (p.getFileName().toString().equals("MechanismIOTalonFX.java")
            && content.contains("setPID(")
            && content.substring(content.indexOf("setPID(")).contains("new Slot0Configs(")) {
          allocViolations++;
        }
        for (String line : content.split("\\R")) {
          if (line.matches("^\\s*//\\s*[=-]{4,}.*")
              || line.matches("^\\s*//\\s*([0-9]+\\.|Step\\s+[0-9]+)\\s+.*")
              || line.matches(".*\\b[ms]_[a-zA-Z0-9_]+\\b.*")) {
            allocViolations++;
          }
        }
      }
    }
    PrecisionReporter.recordLowerIsBetter(
        1,
        "tier1_loop_allocations",
        "Per-Loop ControlRequest Allocs",
        allocViolations,
        0.0,
        0.0,
        "allocs");
    assertEquals(0, allocViolations);
  }

  @Test
  public void testFrcLibNeverImportsFrcRobotMechanisms() throws IOException {
    int boundaryViolations = 0;
    try (Stream<Path> walk = Files.walk(Paths.get("src/main/java/frc/lib"))) {
      List<Path> files = walk.filter(p -> p.toString().endsWith(".java")).toList();
      for (Path p : files) {
        String content = Files.readString(p, StandardCharsets.UTF_8);
        if (content.contains("import frc.robot.")) {
          boundaryViolations++;
        }
      }
    }
    assertEquals(
        0,
        boundaryViolations,
        "frc.lib.* (synced by backTo2026) must never import season-specific frc.robot.* classes.");
  }

  @Test
  public void testStudentDocsAndChangelogSynced() throws IOException {
    Path agentsMd = Paths.get("AGENTS.md");
    Path studentsMd = Paths.get("STUDENTS.md");
    Path changelogMd = Paths.get("STUDENT-CHANGELOG.md");
    Path classMapSvg = Paths.get("docs/class-map.svg");
    Path classMapHtml = Paths.get("docs/class-map.html");
    assertTrue(Files.exists(agentsMd), "AGENTS.md must exist");
    assertTrue(Files.exists(studentsMd), "STUDENTS.md must exist");
    assertTrue(Files.exists(changelogMd), "STUDENT-CHANGELOG.md must exist");
    assertTrue(Files.exists(classMapSvg), "docs/class-map.svg must exist");
    assertTrue(Files.exists(classMapHtml), "docs/class-map.html must exist");

    String agentsText = Files.readString(agentsMd, StandardCharsets.UTF_8);
    assertTrue(
        !agentsText.contains("(PLAN-2027.md)")
            && !agentsText.contains("(FRC-TEAMS.md)")
            && !agentsText.contains("(New_Robot/")
            && !agentsText.contains("copies `dist/fluxfig.jar` into `../"),
        "Per-repo AGENTS.md must be 100% standalone with no broken parent-workspace links or cross-repo write instructions");

    String studentsText = Files.readString(studentsMd, StandardCharsets.UTF_8);
    String svgText = Files.readString(classMapSvg, StandardCharsets.UTF_8);
    int missingClasses = 0;
    try (Stream<Path> walk = Files.walk(Paths.get("src/main/java"))) {
      List<Path> files = walk.filter(p -> p.toString().endsWith(".java")).toList();
      for (Path p : files) {
        String name = p.getFileName().toString();
        String stem = name.substring(0, name.length() - 5);
        if (!studentsText.contains(name) || !svgText.contains(stem)) {
          missingClasses++;
        }
      }
    }
    PrecisionReporter.recordLowerIsBetter(
        1,
        "tier1_student_docs_missing",
        "Undocumented Classes in STUDENTS.md / class-map.svg",
        missingClasses,
        0.0,
        0.0,
        "classes");
    assertEquals(
        0, missingClasses, "Every class must be indexed in STUDENTS.md and docs/class-map.svg");
  }

  @Test
  public void testElasticLayoutsAndPathPlannerAssetsPresentAndValid() throws IOException {
    Path elasticComp = Paths.get("elastic-competition.json");
    Path elasticTest = Paths.get("elastic-test.json");
    Path ppSettings = Paths.get("src/main/deploy/pathplanner/settings.json");
    Path shopShuttlePath =
        Paths.get("src/main/deploy/pathplanner/paths/Shop_Center_Shuttle.path");
    Path preKickoffPath =
        Paths.get("src/main/deploy/pathplanner/paths/PreKickoff_2Piece_Template.path");

    assertTrue(Files.exists(elasticComp), "elastic-competition.json must exist");
    assertTrue(Files.exists(elasticTest), "elastic-test.json must exist");
    assertTrue(Files.exists(ppSettings), "pathplanner/settings.json must exist");
    assertTrue(Files.exists(shopShuttlePath), "Shop_Center_Shuttle.path must exist");
    assertTrue(Files.exists(preKickoffPath), "PreKickoff_2Piece_Template.path must exist");

    String compJson = Files.readString(elasticComp, StandardCharsets.UTF_8);
    String testJson = Files.readString(elasticTest, StandardCharsets.UTF_8);
    assertTrue(
        compJson.contains("/AdvantageKit/RealOutputs/PreMatch/ReadyToPlay"),
        "elastic-competition.json must bind to AdvantageKit PreMatch/ReadyToPlay");
    assertTrue(
        testJson.contains("/AdvantageKit/RealOutputs/PitCheck/Passed"),
        "elastic-test.json must bind to AdvantageKit PitCheck/Passed");
    assertTrue(
        !compJson.contains("/Preferences") && !testJson.contains("/Preferences"),
        "Elastic layouts must not reference legacy /Preferences widget");
  }

  @Test
  public void testRobotLifecycleAndAdvantageKitLoggerWired() throws IOException {
    String robotJava =
        Files.readString(Paths.get("src/main/java/frc/robot/Robot.java"), StandardCharsets.UTF_8);
    String identityJava =
        Files.readString(
            Paths.get("src/main/java/frc/robot/RobotIdentity.java"), StandardCharsets.UTF_8);
    String containerJava =
        Files.readString(
            Paths.get("src/main/java/frc/robot/RobotContainer.java"), StandardCharsets.UTF_8);

    int missingHooks = 0;
    if (!robotJava.contains("SignalLogger.enableAutoLogging(false)")) missingHooks++;
    if (!robotJava.contains("WPILOGWriter") || !robotJava.contains("NT4Publisher")) missingHooks++;
    if (!robotJava.contains("Logger.start()")) missingHooks++;
    if (!robotJava.contains("RobotIdentity.detect()")
        || !identityJava.contains("/sys/firmware/devicetree/base/serial-number")) missingHooks++;
    if (!robotJava.contains("autonomousInit(") || !robotJava.contains("teleopInit(")) missingHooks++;
    if (!containerJava.contains("reloadIfChanged()")) missingHooks++;
    if (!containerJava.contains("setRobotOrientation(")) missingHooks++;
    if (!containerJava.contains("getBatteryVoltage()")) missingHooks++;

    PrecisionReporter.recordLowerIsBetter(
        1,
        "tier1_missing_lifecycle_hooks",
        "Missing Robot/Logger/Lifecycle Hooks",
        missingHooks,
        0.0,
        0.0,
        "hooks");

    assertTrue(
        robotJava.contains("SignalLogger.enableAutoLogging(false)"),
        "Robot.java must disable CTRE .hoot auto-logging when AdvantageKit is active");
    assertTrue(
        robotJava.contains("WPILOGWriter") && robotJava.contains("NT4Publisher"),
        "Robot.java must register WPILOGWriter and NT4Publisher receivers");
    assertTrue(
        robotJava.contains("Logger.start()"),
        "Robot.java must call Logger.start() so AdvantageKit records .wpilog and NT4 outputs");
    assertTrue(
        robotJava.contains("RobotIdentity.detect()")
            && identityJava.contains("/sys/firmware/devicetree/base/serial-number"),
        "Robot.java must resolve RobotIdentity.detect() with Linux device-tree and identity-tag fallback");
    assertTrue(
        robotJava.contains("autonomousInit(") && robotJava.contains("teleopInit("),
        "Robot.java must implement autonomousInit() and teleopInit()");

    assertTrue(
        containerJava.contains("reloadIfChanged()"),
        "RobotContainer.disabledPeriodic() must poll fluxFig.reloadIfChanged()");
    assertTrue(
        containerJava.contains("setRobotOrientation("),
        "RobotContainer.periodicUpdate() must feed gyro orientation to Vision for MT1->MT2 handoff");
    assertTrue(
        containerJava.contains("getBatteryVoltage()"),
        "RobotContainer.periodicUpdate() must read RobotController.getBatteryVoltage() on real hardware");
  }

  @Test
  public void testBackTo2026SourceTranslatorAndDirectorySync() throws IOException {
    String sample2027 =
        "import com.ctre.phoenix6.CANBus;\n"
            + "import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;\n"
            + "import java.util.ArrayList;\n"
            + "import java.util.List;\n"
            + "import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;\n"
            + "import org.wpilib.command2.Command;\n"
            + "import org.wpilib.command2.button.CommandGamepad;\n"
            + "import org.wpilib.driverstation.Alliance;\n"
            + "import org.wpilib.driverstation.DriverStationErrors;\n"
            + "import org.wpilib.driverstation.MatchState;\n"
            + "import org.wpilib.driverstation.RobotState;\n"
            + "import org.wpilib.fields.Field;\n"
            + "import org.wpilib.fields.Fields;\n"
            + "import org.wpilib.framework.RobotBase;\n"
            + "import org.wpilib.hardware.bus.CANPort;\n"
            + "import org.wpilib.hardware.hal.HAL;\n"
            + "import org.wpilib.math.geometry.Pose2d;\n"
            + "import org.wpilib.math.geometry.Rotation2d;\n"
            + "import org.wpilib.math.kinematics.ChassisVelocities;\n"
            + "import org.wpilib.math.kinematics.SwerveModuleVelocity;\n"
            + "import org.wpilib.math.linalg.Matrix;\n"
            + "import org.wpilib.math.linalg.VecBuilder;\n"
            + "import org.wpilib.math.system.DCMotor;\n"
            + "import org.wpilib.math.util.MathUtil;\n"
            + "import org.wpilib.math.util.Units;\n"
            + "import org.wpilib.simulation.DCMotorSim;\n"
            + "import org.wpilib.system.Filesystem;\n"
            + "import org.wpilib.system.RobotController;\n"
            + "import org.wpilib.system.Timer;\n"
            + "public class Sample {\n"
            + "  LoggedNetworkChooser<Command> c;\n"
            + "  LoggedNetworkChooser<Command> autoChooser;\n"
            + "  CommandGamepad driverController;\n"
            + "  CANBus bus = new CANBus(CANPort.CAN_S2);\n"
            + "  SwerveDrivetrainConstants dr = new SwerveDrivetrainConstants().withNetwork(new CANBus(CANPort.CAN_S0));\n"
            + "  SwerveDrivetrainConstants dr2 = new SwerveDrivetrainConstants().withNetwork(bus);\n"
            + "  boolean d = RobotState.isDisabled() && RobotState.isFMSAttached();\n"
            + "  Object a = MatchState.getAlliance().orElse(Alliance.RED);\n"
            + "  Pose2d p = Pose2d.ZERO;\n"
            + "  Rotation2d r = Rotation2d.ZERO;\n"
            + "  SwerveModuleVelocity v = kinematics.toSwerveModuleVelocities(new ChassisVelocities())[0];\n"
            + "  Field f = Field.loadField(Fields.FRC_2026_REBUILT_WELDED);\n"
            + "  double nowSec = Timer.getTimestamp();\n"
            + "  double fpgaSec = RobotController.getTime() / 1.0e9;\n"
            + "  void bind() {\n"
            + "    List<String> items = new ArrayList<>();\n"
            + "    items.add(\"keep-list-add\");\n"
            + "    c.addDefault(\"Auto\", null);\n"
            + "    c.add(\"Other\", null);\n"
            + "    autoChooser.addDefault(\"Main\", null);\n"
            + "    autoChooser.add(\"Alt\", null);\n"
            + "    DriverStationErrors.reportWarning(\"warn\", false);\n"
            + "    driverController.faceDown(); driverController.faceUp();\n"
            + "  }\n"
            + "}\n";

    String translated2026 = BackTo2026Translator.translate2027To2026(sample2027);
    assertTrue(translated2026.contains("import edu.wpi.first.wpilibj2.command.Command;"));
    assertTrue(
        translated2026.contains(
            "import edu.wpi.first.wpilibj2.command.button.CommandXboxController;"));
    assertTrue(translated2026.contains("import edu.wpi.first.apriltag.AprilTagFieldLayout;"));
    assertTrue(translated2026.contains("import edu.wpi.first.apriltag.AprilTagFields;"));
    assertTrue(translated2026.contains("import edu.wpi.first.wpilibj.DriverStation;"));
    assertTrue(translated2026.contains("import edu.wpi.first.wpilibj.DriverStation.Alliance;"));
    assertTrue(translated2026.contains("import edu.wpi.first.wpilibj.RobotBase;"));
    assertTrue(translated2026.contains("import edu.wpi.first.hal.HAL;"));
    assertTrue(translated2026.contains("import edu.wpi.first.math.Matrix;"));
    assertTrue(translated2026.contains("import edu.wpi.first.math.VecBuilder;"));
    assertTrue(translated2026.contains("import edu.wpi.first.math.system.plant.DCMotor;"));
    assertTrue(translated2026.contains("import edu.wpi.first.math.MathUtil;"));
    assertTrue(translated2026.contains("import edu.wpi.first.math.util.Units;"));
    assertTrue(translated2026.contains("import edu.wpi.first.wpilibj.simulation.DCMotorSim;"));
    assertTrue(translated2026.contains("import edu.wpi.first.wpilibj.Filesystem;"));
    assertTrue(translated2026.contains("import edu.wpi.first.wpilibj.RobotController;"));
    assertTrue(translated2026.contains("import edu.wpi.first.wpilibj.Timer;"));
    assertTrue(translated2026.contains("LoggedDashboardChooser<Command> c;"));
    assertTrue(translated2026.contains("CommandXboxController driverController;"));
    assertTrue(translated2026.contains("new CANBus(\"can_s2\")"));
    assertTrue(translated2026.contains(".withCANBusName(\"can_s0\")"));
    assertTrue(translated2026.contains(".withCANBusName(bus.getName())"));
    assertTrue(
        translated2026.contains("DriverStation.isDisabled() && DriverStation.isFMSAttached()"));
    assertTrue(translated2026.contains("DriverStation.getAlliance().orElse(Alliance.Red)"));
    assertTrue(translated2026.contains("Pose2d.kZero") && translated2026.contains("Rotation2d.kZero"));
    assertTrue(
        translated2026.contains(
            "SwerveModuleState v = kinematics.toSwerveModuleStates(new ChassisSpeeds())[0];"));
    assertTrue(
        translated2026.contains(
            "AprilTagFieldLayout f = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);"));
    assertTrue(translated2026.contains("double nowSec = Timer.getFPGATimestamp();"));
    assertTrue(translated2026.contains("double fpgaSec = RobotController.getFPGATime() / 1.0e6;"));
    assertTrue(translated2026.contains("items.add(\"keep-list-add\");"));
    assertTrue(translated2026.contains("c.addDefaultOption(\"Auto\", null);"));
    assertTrue(translated2026.contains("c.addOption(\"Other\", null);"));
    assertTrue(translated2026.contains("autoChooser.addDefaultOption(\"Main\", null);"));
    assertTrue(translated2026.contains("autoChooser.addOption(\"Alt\", null);"));
    assertTrue(translated2026.contains("DriverStation.reportWarning(\"warn\", false);"));
    assertTrue(translated2026.contains("driverController.a(); driverController.y();"));

    String roundTrip2027 = BackTo2026Translator.translate2026To2027(translated2026);
    assertEquals(sample2027, roundTrip2027, "2027 -> 2026 -> 2027 translation must be lossless");
    assertEquals(
        translated2026,
        BackTo2026Translator.translate2027To2026(translated2026),
        "translate2027To2026 must be idempotent on already-translated 2026 source");
    assertEquals(
        roundTrip2027,
        BackTo2026Translator.translate2026To2027(roundTrip2027),
        "translate2026To2027 must be idempotent on already-translated 2027 source");

    Path stageDir = Paths.get("build/backTo2026-test/src/main/java/frc/lib");
    int stagedCount =
        BackTo2026Translator.translateDirectory2027To2026(
            Paths.get("src/main/java/frc/lib"), stageDir);
    assertTrue(stagedCount >= 41, "Expected at least 41 shared frc.lib.* classes staged");

    int untranslatedCount = 0;
    if (translated2026.contains("org.wpilib.")
        || translated2026.contains("ChassisVelocities")
        || translated2026.contains("SwerveModuleVelocity")
        || translated2026.contains("CANPort.")
        || translated2026.contains("CANBus.system")
        || translated2026.contains("LoggedNetworkChooser")
        || translated2026.contains("CommandGamepad")
        || translated2026.contains("RobotState.")
        || translated2026.contains("MatchState.")
        || translated2026.contains("DriverStationErrors.")
        || translated2026.contains("Timer.getTimestamp()")) {
      untranslatedCount++;
    }
    try (Stream<Path> walk = Files.walk(stageDir)) {
      List<Path> stagedFiles = walk.filter(p -> p.toString().endsWith(".java")).toList();
      for (Path p : stagedFiles) {
        if (p.getFileName().toString().equals("BackTo2026Translator.java")) continue;
        String text = Files.readString(p, StandardCharsets.UTF_8);
        if (text.contains("import org.wpilib.")
            || text.contains("ChassisVelocities")
            || text.contains("SwerveModuleVelocity")) {
          untranslatedCount++;
        }
        String as2027 = BackTo2026Translator.translate2026To2027(text);
        if (as2027.contains("edu.wpi.first.") || as2027.contains("Rotation2d.k")) {
          untranslatedCount++;
        }
        assertEquals(
            as2027,
            BackTo2026Translator.translate2026To2027(as2027),
            "translate2026To2027 must be idempotent for " + p.getFileName());
        String roundTripReal = BackTo2026Translator.translate2027To2026(as2027);
        assertEquals(
            text,
            roundTripReal,
            "Round-trip 2026->2027->2026 must be lossless for " + p.getFileName());
      }
    }

    PrecisionReporter.recordLowerIsBetter(
        1,
        "tier1_backto2026_untranslated",
        "BackTo2026 Untranslated 2027 Tokens",
        untranslatedCount,
        0.0,
        0.0,
        "tokens");
    assertEquals(0, untranslatedCount, "Zero untranslated 2027 tokens allowed after backTo2026");

    String buildGradle = Files.readString(Paths.get("build.gradle"), StandardCharsets.UTF_8);
    assertTrue(
        buildGradle.contains("generateFluxPkgManifest")
            && buildGradle.contains("\"targetPlatform\": \"systemcore-2027\"")
            && buildGradle.contains("gitDirty ? \"EXPERIMENTAL\"")
            && buildGradle.contains("_fluxpkg.zip")
            && buildGradle.contains("from(\"$buildDir/tmp/fluxpkg/manifest.json\")"),
        "build.gradle must generate manifest.json with targetPlatform=systemcore-2027, EXPERIMENTAL dirty channel guard, and package into _fluxpkg.zip");

    String mechanismsCsv =
        Files.readString(
            Paths.get("src/main/deploy/configs/mechanisms.csv"), StandardCharsets.UTF_8);
    assertTrue(
        mechanismsCsv.contains("followerMotorId,invertFollower,motorInverted,gearRatio")
            && mechanismsCsv.contains(
                "COMP_2027,ElevatorLeader,10,can_s2,60.0,40.0,60.0,ELEVATOR_STATIC,2.40,0.0,0.08,0.18,0.115,0.012,0.38,0.0,48.5,11,false,false,1.0")
            && mechanismsCsv.contains("COMP_2027,IntakeRoller,14,can_s2")
            && mechanismsCsv.contains("COMP_2027,SwerveDriveFL")
            && !mechanismsCsv.contains(",6.75"),
        "mechanisms.csv must use the 21-column schema with ElevatorLeader followerMotorId=11, IntakeRoller, and 6.3947 swerve gearRatio");
  }

  @Test
  public void testBackTo2026LegacyNormalizationsAndIdempotence() {
    String legacy2027 =
        "import org.wpilib.vision.apriltag.Fields;\n"
            + "import org.wpilib.vision.apriltag.AprilTagFieldLayout;\n"
            + "import org.wpilib.hal.HAL;\n"
            + "import org.wpilib.wpilibj.Timer;\n"
            + "import org.wpilib.wpilibj.simulation.DCMotorSim;\n"
            + "import org.wpilib.wpilibj2.command.Command;\n"
            + "public class Legacy {\n"
            + "  CANBus b1 = CANBus.systemcore(0);\n"
            + "  CANBus b2 = CANBus.systemCore(2);\n"
            + "  boolean fms = MatchState.isFMSAttached();\n"
            + "  Object f = Fields.k2026RebuiltWelded;\n"
            + "}\n";

    String translated = BackTo2026Translator.translate2027To2026(legacy2027);
    assertTrue(!translated.contains("org.wpilib."));
    assertTrue(!translated.contains("CANBus.system"));
    assertTrue(!translated.contains("MatchState."));
    assertTrue(translated.contains("import edu.wpi.first.apriltag.AprilTagFields;"));
    assertTrue(translated.contains("import edu.wpi.first.apriltag.AprilTagFieldLayout;"));
    assertTrue(translated.contains("import edu.wpi.first.hal.HAL;"));
    assertTrue(translated.contains("import edu.wpi.first.wpilibj.Timer;"));
    assertTrue(translated.contains("import edu.wpi.first.wpilibj.simulation.DCMotorSim;"));
    assertTrue(translated.contains("import edu.wpi.first.wpilibj2.command.Command;"));
    assertTrue(translated.contains("new CANBus(\"can_s0\")"));
    assertTrue(translated.contains("new CANBus(\"can_s2\")"));
    assertTrue(translated.contains("DriverStation.isFMSAttached()"));
    assertTrue(translated.contains("AprilTagFields.k2026RebuiltWelded"));
    assertEquals(
        translated,
        BackTo2026Translator.translate2027To2026(translated),
        "Legacy normalization output must be idempotent");
  }

  @Test
  public void testBackTo2026CommentsStringsNestedGenericsAndPlurals() {
    String edge2026 =
        "import edu.wpi.first.apriltag.AprilTagFieldLayout;\n"
            + "import edu.wpi.first.apriltag.AprilTagFields;\n"
            + "import edu.wpi.first.math.kinematics.SwerveModuleState;\n"
            + "import edu.wpi.first.wpilibj.DriverStation;\n"
            + "import java.util.ArrayList;\n"
            + "import java.util.List;\n"
            + "import java.util.function.Supplier;\n"
            + "import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;\n"
            + "import edu.wpi.first.wpilibj2.command.Command;\n"
            + "public class EdgeCases {\n"
            + "  // Field centric, robot centric\n"
            + "  // Field layout notes & DriverStation.java\n"
            + "  String msg = \"Field origin, calibrated\";\n"
            + "  LoggedDashboardChooser<Supplier<Command>> nestedPicker;\n"
            + "  List<String> itemsInChooser = new ArrayList<>();\n"
            + "  SwerveModuleState[] desiredSwerveModuleStates = new SwerveModuleState[4];\n"
            + "  public AprilTagFieldLayout getTagLayout() {\n"
            + "    return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);\n"
            + "  }\n"
            + "  void test(Object o) {\n"
            + "    AprilTagFieldLayout x = (AprilTagFieldLayout) o;\n"
            + "    AprilTagFieldLayout.OriginPosition p = AprilTagFieldLayout.OriginPosition.kBlueWall;\n"
            + "    itemsInChooser.add(\"NotAChooserCall\");\n"
            + "    nestedPicker.addOption(\"C\", () -> null);\n"
            + "    boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;\n"
            + "  }\n"
            + "}\n";

    String as2027 = BackTo2026Translator.translate2026To2027(edge2026);
    assertTrue(as2027.contains("// Field centric, robot centric"));
    assertTrue(as2027.contains("// Field layout notes & DriverStation.java"));
    assertTrue(as2027.contains("\"Field origin, calibrated\""));
    assertTrue(as2027.contains("LoggedNetworkChooser<Supplier<Command>> nestedPicker;"));
    assertTrue(as2027.contains("itemsInChooser.add(\"NotAChooserCall\");"));
    assertTrue(as2027.contains("nestedPicker.add(\"C\", () -> null);"));
    assertTrue(!as2027.contains("Velocitys"));
    assertTrue(as2027.contains("SwerveModuleVelocity[] desiredSwerveModuleVelocities"));
    assertTrue(as2027.contains("public Field getTagLayout()"));
    assertTrue(as2027.contains("(Field) o"));
    assertTrue(as2027.contains("Field.OriginPosition p = Field.OriginPosition.kBlueWall;"));
    assertTrue(!as2027.contains("DriverStation.Alliance"));
    assertEquals(
        edge2026,
        BackTo2026Translator.translate2027To2026(as2027),
        "Edge-case round-trip 2026->2027->2026 must be lossless");

    String inline2027 =
        "public class InlineDs {\n"
            + "  boolean fms = org.wpilib.driverstation.RobotState.isFMSAttached();\n"
            + "}\n";
    String inline2026 = BackTo2026Translator.translate2027To2026(inline2027);
    assertTrue(!inline2026.contains("edu.wpi.first.wpilibj.driverstation."));
    assertTrue(inline2026.contains("edu.wpi.first.wpilibj.DriverStation.isFMSAttached()"));
  }
}
