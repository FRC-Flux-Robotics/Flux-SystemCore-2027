package frc.lib.util;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Deterministic source-to-source translator between WPILib 2027 SystemCore ({@code org.wpilib.*},
 * Phoenix 6 {@code 26.70.x}, AdvantageKit {@code v27}, JDK 25) and WPILib 2026 roboRIO ({@code
 * edu.wpi.first.*}, Phoenix 6 {@code 26.1.x}, AdvantageKit {@code v26}, JDK 17) for the shared
 * {@code frc.lib.*} core engine.
 *
 * <p>Operates strictly on Java code tokens (preserving string literals, char literals, line
 * comments, and block comments) and is used by {@code ./gradlew backTo2026} and verified in {@code
 * Tier1ArchitectureHygieneTest}.
 */
public final class BackTo2026Translator {
  /**
   * Canonical bijective rules ordered from most-specific prefix/token to least-specific fallback.
   * All pairs here MUST remain exact 1:1 inverses for lossless round-tripping.
   */
  private static final String[][] RULES_2027_TO_2026 = {
    // Commands v2 & controllers (most specific first)
    {
      "org.wpilib.command2.button.CommandGamepad",
      "edu.wpi.first.wpilibj2.command.button.CommandXboxController"
    },
    {"org.wpilib.command2.", "edu.wpi.first.wpilibj2.command."},
    // WPILib 2027 alpha-8 Fields / AprilTag
    {"org.wpilib.fields.Fields", "edu.wpi.first.apriltag.AprilTagFields"},
    {"org.wpilib.fields.Field", "edu.wpi.first.apriltag.AprilTagFieldLayout"},
    // Specific math & units subpackages (before generic org.wpilib.math.)
    {"org.wpilib.math.linalg.Matrix", "edu.wpi.first.math.Matrix"},
    {"org.wpilib.math.linalg.VecBuilder", "edu.wpi.first.math.VecBuilder"},
    {"org.wpilib.math.util.MathUtil", "edu.wpi.first.math.MathUtil"},
    {"org.wpilib.math.system.DCMotor", "edu.wpi.first.math.system.plant.DCMotor"},
    {"org.wpilib.networktables.", "edu.wpi.first.networktables."},
    {"org.wpilib.math.", "edu.wpi.first.math."},
    {"org.wpilib.units.", "edu.wpi.first.units."},
    // WPILib 2027 alpha-8 HAL, simulation, util, system, framework, driverstation (before catch-all)
    {"org.wpilib.hardware.hal.", "edu.wpi.first.hal."},
    {"org.wpilib.simulation.", "edu.wpi.first.wpilibj.simulation."},
    {"org.wpilib.util.Alert", "edu.wpi.first.wpilibj.Alert"},
    {"org.wpilib.util.", "edu.wpi.first.util."},
    {"org.wpilib.driverstation.Alliance", "edu.wpi.first.wpilibj.DriverStation.Alliance"},
    {"org.wpilib.driverstation.GenericHID", "edu.wpi.first.wpilibj.GenericHID"},
    {"org.wpilib.system.Timer", "edu.wpi.first.wpilibj.Timer"},
    {"org.wpilib.system.Filesystem", "edu.wpi.first.wpilibj.Filesystem"},
    {"org.wpilib.system.RobotController", "edu.wpi.first.wpilibj.RobotController"},
    {"org.wpilib.system.Notifier", "edu.wpi.first.wpilibj.Notifier"},
    {"org.wpilib.system.Threads", "edu.wpi.first.wpilibj.Threads"},
    {"org.wpilib.system.Watchdog", "edu.wpi.first.wpilibj.Watchdog"},
    {"org.wpilib.framework.RobotBase", "edu.wpi.first.wpilibj.RobotBase"},
    {"org.wpilib.framework.IterativeRobotBase", "edu.wpi.first.wpilibj.IterativeRobotBase"},
    {"org.wpilib.framework.TimedRobot", "edu.wpi.first.wpilibj.TimedRobot"},
    // Fallback WPILib root package rule
    {"org.wpilib.", "edu.wpi.first.wpilibj."},
    // Kinematics & swerve velocity renames (longer plural/method tokens BEFORE singular!)
    {"toSwerveModuleVelocities", "toSwerveModuleStates"},
    {"desaturateWheelVelocities", "desaturateWheelSpeeds"},
    {"toChassisVelocities", "toChassisSpeeds"},
    {"SwerveModuleVelocities", "SwerveModuleStates"},
    {"SwerveModuleVelocity", "SwerveModuleState"},
    {"ChassisVelocities", "ChassisSpeeds"},
    // AdvantageKit v27 chooser rename
    {"LoggedNetworkChooser", "LoggedDashboardChooser"},
    // Controller rename (alpha-8 CommandGamepad <-> 2026 CommandXboxController)
    {"CommandGamepad", "CommandXboxController"},
    // DriverStation split into RobotState, MatchState, DriverStationErrors (alpha-8)
    {"RobotState.isDisabled()", "DriverStation.isDisabled()"},
    {"RobotState.isEnabled()", "DriverStation.isEnabled()"},
    {"RobotState.isAutonomous()", "DriverStation.isAutonomous()"},
    {"RobotState.isTeleop()", "DriverStation.isTeleop()"},
    {"RobotState.isFMSAttached()", "DriverStation.isFMSAttached()"},
    {"MatchState.getAlliance()", "DriverStation.getAlliance()"},
    {"MatchState.getMatchType()", "DriverStation.getMatchType()"},
    {"MatchState.getMatchTime()", "DriverStation.getMatchTime()"},
    {"DriverStationErrors.reportWarning(", "DriverStation.reportWarning("},
    {"DriverStationErrors.reportError(", "DriverStation.reportError("},
    // Timebase & FPGA timestamp updates (nanoseconds in 2027 vs microseconds in 2026)
    {"Timer.getTimestamp()", "Timer.getFPGATimestamp()"},
    {"RobotController.getTime() / 1.0e9", "RobotController.getFPGATime() / 1.0e6"},
    // Field constants (alpha-8)
    {"Fields.FRC_2026_REBUILT_WELDED", "AprilTagFields.k2026RebuiltWelded"},
    {"Fields.FRC_2026_REBUILT_ANDY_MARK", "AprilTagFields.k2026RebuiltAndyMark"},
    // Geometry & Alliance constants
    {"Rotation2d.ZERO", "Rotation2d.kZero"},
    {"Rotation2d.PI", "Rotation2d.kPi"},
    {"Pose2d.ZERO", "Pose2d.kZero"},
    {"Translation2d.ZERO", "Translation2d.kZero"},
    {"Rotation3d.ZERO", "Rotation3d.kZero"},
    {"Alliance.RED", "Alliance.Red"},
    {"Alliance.BLUE", "Alliance.Blue"},
    // Gamepad button renames
    {".faceDown()", ".a()"},
    {".faceRight()", ".b()"},
    {".faceLeft()", ".x()"},
    {".faceUp()", ".y()"}
  };

  /**
   * Inline / legacy 2027 -> 2026 normalizations applied ONLY in {@link #translate2027To2026} so
   * qualified alpha-8 calls (e.g. {@code org.wpilib.driverstation.RobotState.*}) and older
   * alpha-5/7 skeletons compile on 2026 without polluting canonical {@link #translate2026To2027}.
   * Ordered most-specific first, before the generic {@code org.wpilib.} fallback.
   */
  private static final String[][] LEGACY_AND_INLINE_2027_TO_2026 = {
    // Inline qualified org.wpilib.driverstation.* calls in real 2027 code (Spectrum MiniSwerve)
    {"org.wpilib.driverstation.RobotState.", "edu.wpi.first.wpilibj.DriverStation."},
    {"org.wpilib.driverstation.MatchState.", "edu.wpi.first.wpilibj.DriverStation."},
    {"org.wpilib.driverstation.DriverStationErrors.", "edu.wpi.first.wpilibj.DriverStation."},
    {"org.wpilib.driverstation.DriverStation.", "edu.wpi.first.wpilibj.DriverStation."},
    // Older alpha-5/7 skeletons
    {
      "org.wpilib.wpilibj2.command.button.CommandXboxController",
      "edu.wpi.first.wpilibj2.command.button.CommandXboxController"
    },
    {"org.wpilib.wpilibj2.command.", "edu.wpi.first.wpilibj2.command."},
    {"org.wpilib.vision.apriltag.Fields", "edu.wpi.first.apriltag.AprilTagFields"},
    {
      "org.wpilib.vision.apriltag.AprilTagFieldLayout",
      "edu.wpi.first.apriltag.AprilTagFieldLayout"
    },
    {"org.wpilib.vision.apriltag.", "edu.wpi.first.apriltag."},
    {"org.wpilib.apriltag.AprilTagFields", "edu.wpi.first.apriltag.AprilTagFields"},
    {"org.wpilib.apriltag.", "edu.wpi.first.apriltag."},
    {"org.wpilib.math.system.plant.DCMotor", "edu.wpi.first.math.system.plant.DCMotor"},
    {"org.wpilib.wpilibj.simulation.", "edu.wpi.first.wpilibj.simulation."},
    {"org.wpilib.wpilibj.", "edu.wpi.first.wpilibj."},
    {"org.wpilib.hal.", "edu.wpi.first.hal."},
    {"MatchState.isFMSAttached()", "DriverStation.isFMSAttached()"}
  };

  private static final Pattern IMPORT_CANPORT_PATTERN =
      Pattern.compile("(?m)^import\\s+org\\.wpilib\\.hardware\\.bus\\.CANPort;\\R?");
  private static final Pattern IMPORT_DS_SPLIT_PATTERN =
      Pattern.compile(
          "import\\s+org\\.wpilib\\.driverstation\\.(?:DriverStationErrors|MatchState|RobotState|DriverStation);");
  private static final Pattern IMPORT_DS_2026_PATTERN =
      Pattern.compile("import\\s+(?:edu\\.wpi\\.first\\.wpilibj|org\\.wpilib)\\.DriverStation;");
  private static final Pattern CHOOSER_TYPE_PREFIX =
      Pattern.compile("\\bLogged(?:Network|Dashboard)Chooser\\s*<");
  private static final Pattern COLLECTION_TYPE_PREFIX =
      Pattern.compile(
          "\\b(?:List|ArrayList|LinkedList|Set|HashSet|LinkedHashSet|TreeSet|Collection|Queue|Deque|ArrayDeque)\\s*<");

  private BackTo2026Translator() {}

  /**
   * Translates a single WPILib 2027 Java source file string into a valid WPILib 2026 Java source
   * string while keeping string literals and comments intact.
   */
  public static String translate2027To2026(String javaSource) {
    Set<String> chooserVars = collectGenericTypedVars(javaSource, CHOOSER_TYPE_PREFIX);
    Set<String> collectionVars = collectGenericTypedVars(javaSource, COLLECTION_TYPE_PREFIX);
    boolean hasReflectField = javaSource.contains("java.lang.reflect.Field");
    boolean hasAllianceImport =
        javaSource.contains("edu.wpi.first.wpilibj.DriverStation.Alliance")
            || javaSource.contains("import org.wpilib.driverstation.Alliance;");

    String prepared = IMPORT_CANPORT_PATTERN.matcher(javaSource).replaceAll("");
    prepared = collapseDriverStationImportsTo2026(prepared);
    boolean hasDriverStationImport =
        prepared.contains("import edu.wpi.first.wpilibj.DriverStation;");

    return transformCodeSegments(
        prepared,
        code -> translateCodeSegment2027To2026(
            code,
            chooserVars,
            collectionVars,
            hasReflectField,
            hasDriverStationImport,
            hasAllianceImport));
  }

  private static String translateCodeSegment2027To2026(
      String code,
      Set<String> chooserVars,
      Set<String> collectionVars,
      boolean hasReflectField,
      boolean hasDriverStationImport,
      boolean hasAllianceImport) {
    String out = code;
    for (String[] rule : LEGACY_AND_INLINE_2027_TO_2026) {
      out = out.replace(rule[0], rule[1]);
    }
    for (String[] rule : RULES_2027_TO_2026) {
      out = out.replace(rule[0], rule[1]);
    }
    if (hasDriverStationImport && !hasAllianceImport) {
      out =
          out.replaceAll(
              "(?<!import\\s)edu\\.wpi\\.first\\.wpilibj\\.DriverStation\\.Alliance\\b",
              "DriverStation.Alliance");
    }
    out = rewriteChooserCalls(out, chooserVars, collectionVars, true);
    if (!hasReflectField) {
      out = out.replaceAll("(?<![.\\w])Field\\b", "AprilTagFieldLayout");
    }
    out =
        out.replaceAll(
            "(?<!AprilTag)\\bFields\\.k2026RebuiltWelded\\b", "AprilTagFields.k2026RebuiltWelded");

    // Phoenix 6 26.70.0-alpha-2 CANBus(CANPort.CAN_S<n>) -> new CANBus("can_s<n>")
    out =
        out.replaceAll(
            "new\\s+CANBus\\(\\s*CANPort\\.CAN_S(\\d+)\\s*\\)", "new CANBus(\"can_s$1\")");
    // Legacy Phoenix 6 26.50.0-alpha-1 CANBus.systemcore(n) & pre-alpha CANBus.systemCore(n)
    out = out.replaceAll("CANBus\\.system[Cc]ore\\((\\d+)\\)", "new CANBus(\"can_s$1\")");
    // Phoenix 6 SwerveDrivetrainConstants.withNetwork(...) -> withCANBusName(...)
    out =
        out.replaceAll(
            "\\.withNetwork\\(\\s*new\\s+CANBus\\(\\s*(\"[^\"]+\"|\u0001\\d+\u0002)\\s*\\)\\s*\\)",
            ".withCANBusName($1)");
    out =
        out.replaceAll(
            "\\.withNetwork\\(\\s*([A-Za-z_][A-Za-z0-9_]*)\\s*\\)",
            ".withCANBusName($1.getName())");
    return out;
  }

  /**
   * Translates a single WPILib 2026 Java source file string into WPILib 2027 SystemCore source so
   * CI tests and forward migration verify lossless round-tripping across all {@code frc.lib.*}
   * classes while preserving string literals and comments.
   */
  public static String translate2026To2027(String javaSource) {
    Set<String> chooserVars = collectGenericTypedVars(javaSource, CHOOSER_TYPE_PREFIX);
    Set<String> collectionVars = collectGenericTypedVars(javaSource, COLLECTION_TYPE_PREFIX);

    String out =
        transformCodeSegments(
            javaSource,
            code -> translateCodeSegment2026To2027(code, chooserVars, collectionVars));
    if (hasCodeToken(out, "CANPort.CAN_S")) {
      out = insertImportSorted(out, "org.wpilib.hardware.bus.CANPort");
    }
    out = expandDriverStationImportTo2027(out);
    return out;
  }

  private static String translateCodeSegment2026To2027(
      String code, Set<String> chooserVars, Set<String> collectionVars) {
    String out = code;
    for (String[] rule : RULES_2027_TO_2026) {
      out = out.replace(rule[1], rule[0]);
    }
    // Unimported qualified DriverStation.Alliance in 2026 code -> org.wpilib.driverstation.Alliance
    out = out.replaceAll("(?<![.\\w])DriverStation\\.Alliance\\b", "org.wpilib.driverstation.Alliance");
    out = rewriteChooserCalls(out, chooserVars, collectionVars, false);
    // AprilTagFieldLayout -> Field (alpha-8 org.wpilib.fields.Field)
    out = out.replaceAll("(?<![.\\w])AprilTagFieldLayout\\b", "Field");
    // Phoenix 6 26.70.0-alpha-2 SwerveDrivetrainConstants & SystemCore CANPort
    out =
        out.replaceAll(
            "\\.withCANBusName\\(\\s*([A-Za-z_][A-Za-z0-9_]*)\\.getName\\(\\)\\s*\\)",
            ".withNetwork($1)");
    out =
        out.replaceAll(
            "\\.withCANBusName\\(\\s*(\"[^\"]+\"|\u0001\\d+\u0002)\\s*\\)",
            ".withNetwork(new CANBus($1))");
    out = out.replaceAll("new\\s+CANBus\\(\\s*\"can_s(\\d+)\"\\s*\\)", "new CANBus(CANPort.CAN_S$1)");
    return out;
  }

  /**
   * Tokenizes Java source into literal/comment spans (preserved verbatim) vs. code spans
   * (transformed via {@code codeTransform}). Note: string/char literals inside code transforms such
   * as {@code "can_s0"} match as delimiters + contents, so wait: string literals MUST remain
   * accessible when part of code patterns like {@code new CANBus("can_s2")} or {@code
   * .addDefault("Auto", null)}! Therefore, ONLY comments ({@code // ...}, {@code /* ... *\/}) and
   * non-code-matched standalone string literals are masked: strings ARE kept out of identifier
   * passes, EXCEPT {@code new CANBus("can_s\d+")}, {@code .withNetwork("...")}, and {@code
   * .withCANBusName("...")}.
   */
  private static String transformCodeSegments(
      String source, Function<String, String> codeTransform) {
    // Mask comments, char literals, and string literals with collision-free placeholders, EXCEPT
    // restore string literals before CANBus/withNetwork regexes would miss them? Even cleaner: mask
    // comments + char literals + standalone string literals, while keeping short safe markers!
    List<String> masked = new ArrayList<>();
    StringBuilder unmaskedCode = new StringBuilder(source.length());
    int i = 0;
    int n = source.length();
    while (i < n) {
      char c = source.charAt(i);
      if (c == '/' && i + 1 < n && source.charAt(i + 1) == '/') {
        int end = source.indexOf('\n', i + 2);
        if (end < 0) end = n;
        int idx = masked.size();
        masked.add(source.substring(i, end));
        unmaskedCode.append('\u0001').append(idx).append('\u0002');
        i = end;
      } else if (c == '/' && i + 1 < n && source.charAt(i + 1) == '*') {
        int end = source.indexOf("*/", i + 2);
        end = (end < 0) ? n : end + 2;
        int idx = masked.size();
        masked.add(source.substring(i, end));
        unmaskedCode.append('\u0001').append(idx).append('\u0002');
        i = end;
      } else if (c == '"' && i + 2 < n && source.charAt(i + 1) == '"' && source.charAt(i + 2) == '"') {
        int end = source.indexOf("\"\"\"", i + 3);
        end = (end < 0) ? n : end + 3;
        int idx = masked.size();
        masked.add(source.substring(i, end));
        unmaskedCode.append('\u0001').append(idx).append('\u0002');
        i = end;
      } else if (c == '"') {
        int j = i + 1;
        while (j < n) {
          char sc = source.charAt(j);
          if (sc == '\\') {
            j += 2;
          } else if (sc == '"') {
            j++;
            break;
          } else if (sc == '\n') {
            break;
          } else {
            j++;
          }
        }
        String lit = source.substring(i, j);
        if (lit.matches("\"can_s\\d+\"")) {
          // Expose CAN bus port literals directly to code regexes; safe from all class/pkg rules
          unmaskedCode.append(lit);
        } else {
          int idx = masked.size();
          masked.add(lit);
          unmaskedCode.append('\u0001').append(idx).append('\u0002');
        }
        i = j;
      } else if (c == '\'') {
        int j = i + 1;
        while (j < n) {
          char sc = source.charAt(j);
          if (sc == '\\') {
            j += 2;
          } else if (sc == '\'') {
            j++;
            break;
          } else if (sc == '\n') {
            break;
          } else {
            j++;
          }
        }
        int idx = masked.size();
        masked.add(source.substring(i, j));
        unmaskedCode.append('\u0001').append(idx).append('\u0002');
        i = j;
      } else {
        unmaskedCode.append(c);
        i++;
      }
    }

    String transformed = codeTransform.apply(unmaskedCode.toString());
    StringBuilder result = new StringBuilder(transformed.length() + source.length() / 4);
    int k = 0;
    int tn = transformed.length();
    while (k < tn) {
      char tc = transformed.charAt(k);
      if (tc == '\u0001') {
        int endMarker = transformed.indexOf('\u0002', k + 1);
        if (endMarker > k + 1) {
          int idx = Integer.parseInt(transformed.substring(k + 1, endMarker));
          result.append(masked.get(idx));
          k = endMarker + 1;
          continue;
        }
      }
      result.append(tc);
      k++;
    }
    return result.toString();
  }

  private static boolean hasCodeToken(String javaSource, String token) {
    final boolean[] found = {false};
    transformCodeSegments(
        javaSource,
        code -> {
          if (code.contains(token)) {
            found[0] = true;
          }
          return code;
        });
    return found[0];
  }

  /** Collects variable/field identifiers declared with generic types (supports nested {@code <...>}). */
  private static Set<String> collectGenericTypedVars(String javaSource, Pattern typePrefix) {
    Set<String> vars = new LinkedHashSet<>();
    transformCodeSegments(
        javaSource,
        code -> {
          Matcher m = typePrefix.matcher(code);
          while (m.find()) {
            int depth = 1;
            int idx = m.end();
            int len = code.length();
            while (idx < len && depth > 0) {
              char ch = code.charAt(idx);
              if (ch == '<') {
                depth++;
              } else if (ch == '>') {
                depth--;
              }
              idx++;
            }
            if (depth == 0) {
              while (idx < len && Character.isWhitespace(code.charAt(idx))) {
                idx++;
              }
              int startId = idx;
              if (idx < len && Character.isJavaIdentifierStart(code.charAt(idx))) {
                idx++;
                while (idx < len && Character.isJavaIdentifierPart(code.charAt(idx))) {
                  idx++;
                }
                vars.add(code.substring(startId, idx));
              }
            }
          }
          return code;
        });
    return vars;
  }

  /**
   * Rewrites 2-argument chooser {@code .add(k, v)} / {@code .addDefault(k, v)} calls while ignoring
   * 1-argument {@code List.add(v)} and any declared {@code Collection} variable.
   */
  private static String rewriteChooserCalls(
      String code, Set<String> chooserVars, Set<String> collectionVars, boolean to2026) {
    String methodRegex =
        to2026 ? "\\b([A-Za-z_][A-Za-z0-9_]*)\\.(addDefault|add)\\(" : "\\b([A-Za-z_][A-Za-z0-9_]*)\\.(addDefault|add)Option\\(";
    Pattern p = Pattern.compile(methodRegex);
    Matcher m = p.matcher(code);
    StringBuilder sb = new StringBuilder(code.length());
    int cursor = 0;
    while (m.find()) {
      String receiver = m.group(1);
      String baseMethod = m.group(2);
      boolean isKnownChooser =
          !collectionVars.contains(receiver)
              && (chooserVars.contains(receiver) || receiver.toLowerCase().endsWith("chooser"));
      if (!isKnownChooser) {
        continue;
      }
      int openParen = m.end() - 1;
      int argCount = countCallArguments(code, openParen);
      if (argCount != 2) {
        continue;
      }
      sb.append(code, cursor, m.start());
      sb.append(receiver).append('.').append(baseMethod);
      sb.append(to2026 ? "Option(" : "(");
      cursor = m.end();
    }
    sb.append(code, cursor, code.length());
    return sb.toString();
  }

  private static int countCallArguments(String code, int openParenIndex) {
    int depth = 1;
    int commas = 0;
    boolean hasContent = false;
    int i = openParenIndex + 1;
    int n = code.length();
    while (i < n && depth > 0) {
      char ch = code.charAt(i);
      if (ch == '(' || ch == '{' || ch == '[') {
        depth++;
        hasContent = true;
      } else if (ch == ')' || ch == '}' || ch == ']') {
        depth--;
        if (depth > 0) {
          hasContent = true;
        }
      } else if (ch == ',' && depth == 1) {
        commas++;
      } else if (!Character.isWhitespace(ch)) {
        hasContent = true;
      }
      i++;
    }
    if (depth != 0) {
      return -1;
    }
    return hasContent ? commas + 1 : 0;
  }

  private static String collapseDriverStationImportsTo2026(String source) {
    String[] lines = source.split("\n", -1);
    List<String> result = new ArrayList<>(lines.length);
    boolean emittedDriverStation = false;
    for (String line : lines) {
      String trimmed = line.trim();
      if (IMPORT_DS_SPLIT_PATTERN.matcher(trimmed).matches()) {
        if (!emittedDriverStation) {
          result.add("import edu.wpi.first.wpilibj.DriverStation;");
          emittedDriverStation = true;
        }
      } else if (IMPORT_DS_2026_PATTERN.matcher(trimmed).matches()) {
        if (!emittedDriverStation) {
          result.add(line);
          emittedDriverStation = true;
        }
      } else {
        result.add(line);
      }
    }
    return String.join("\n", result);
  }

  private static String expandDriverStationImportTo2027(String source) {
    Matcher hasImport = IMPORT_DS_2026_PATTERN.matcher(source);
    if (!hasImport.find()) {
      return source;
    }
    String body =
        source.replaceAll(
            "(?m)^import\\s+(?:edu\\.wpi\\.first\\.wpilibj|org\\.wpilib)\\.DriverStation;\\R?",
            "");
    List<String> needed = new ArrayList<>();
    if (hasCodeToken(body, "DriverStationErrors.")) {
      needed.add("import org.wpilib.driverstation.DriverStationErrors;");
    }
    if (hasCodeToken(body, "MatchState.")) {
      needed.add("import org.wpilib.driverstation.MatchState;");
    }
    if (hasCodeToken(body, "RobotState.")) {
      needed.add("import org.wpilib.driverstation.RobotState;");
    }
    if (hasCodeToken(body, "DriverStation.") || needed.isEmpty()) {
      needed.add("import org.wpilib.driverstation.DriverStation;");
    }
    Collections.sort(needed);
    String replacement = String.join("\n", needed);
    return hasImport.replaceFirst(Matcher.quoteReplacement(replacement));
  }

  private static String insertImportSorted(String source, String fqn) {
    String targetLine = "import " + fqn + ";";
    if (source.contains(targetLine)) {
      return source;
    }
    StringBuilder out = new StringBuilder();
    String[] rawLines = source.split("\n", -1);
    int insertIdx = -1;
    int lastImportIdx = -1;
    for (int i = 0; i < rawLines.length; i++) {
      String trimmed = rawLines[i].trim();
      if (trimmed.startsWith("import ") && trimmed.endsWith(";")) {
        String existing = trimmed.substring("import ".length(), trimmed.length() - 1).trim();
        lastImportIdx = i;
        if (insertIdx == -1 && existing.compareTo(fqn) > 0) {
          insertIdx = i;
        }
      }
    }
    if (lastImportIdx == -1) {
      return source;
    }
    if (insertIdx == -1) {
      insertIdx = lastImportIdx + 1;
    }
    for (int i = 0; i < rawLines.length; i++) {
      if (i == insertIdx) {
        out.append(targetLine).append('\n');
      }
      out.append(rawLines[i]);
      if (i < rawLines.length - 1) {
        out.append('\n');
      }
    }
    if (insertIdx == rawLines.length) {
      out.append('\n').append(targetLine);
    }
    return out.toString();
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
      List<Path> files =
          walk.filter(p -> Files.isRegularFile(p) && p.toString().endsWith(".java")).toList();
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
