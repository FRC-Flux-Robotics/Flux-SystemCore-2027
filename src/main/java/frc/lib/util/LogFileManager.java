package frc.lib.util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * OS-agnostic AdvantageKit & CTRE log storage manager (`/U/logs` -> operating directory `logs`),
 * recursively cleaning both old `.wpilog` and nested `.hoot` files to prevent flash exhaustion
 * and writing `run-meta.json`.
 */
public final class LogFileManager {
  private static final long MAX_STORAGE_BYTES = 500L * 1024 * 1024; // 500 MB

  private LogFileManager() {}

  /** Resolves `/U/logs` if a USB drive is mounted, else `<operatingDir>/logs`. */
  public static String getLogPath() {
    File usbDir = new File("/U/logs");
    if (usbDir.getParentFile() != null && usbDir.getParentFile().exists()) {
      return getLogPath(usbDir, MAX_STORAGE_BYTES);
    }
    File localDir = new File(Filesystem.getOperatingDirectory(), "logs");
    return getLogPath(localDir, MAX_STORAGE_BYTES);
  }

  public static String getLogPath(File logDir, long maxBytes) {
    if (!logDir.exists()) {
      logDir.mkdirs();
    }
    cleanOldLogs(logDir, maxBytes);
    return logDir.getAbsolutePath();
  }

  /** Writes per-run `run-meta.json` summary inside the active log directory for FluxTray/FluxProcessor. */
  public static Path writeRunMetaJson(
      Path logDir,
      String robotId,
      String profile,
      String configSha256,
      String batteryName,
      int loopPeriodMs)
      throws IOException {
    return writeRunMetaJson(
        logDir,
        robotId,
        profile,
        configSha256,
        batteryName,
        loopPeriodMs,
        readDeployedGitSha(Filesystem.getDeployDirectory().toPath()),
        Instant.now().toString());
  }

  /**
   * Writes per-run `run-meta.json` summary including deployed `gitSha` and stable `bootTimestampIso`.
   */
  public static Path writeRunMetaJson(
      Path logDir,
      String robotId,
      String profile,
      String configSha256,
      String batteryName,
      int loopPeriodMs,
      String gitSha,
      String bootTimestampIso)
      throws IOException {
    Files.createDirectories(logDir);
    Path metaFile = logDir.resolve("run-meta.json");
    String effectiveGitSha =
        (gitSha != null && !gitSha.isBlank()) ? gitSha : "unknown";
    String effectiveBootIso =
        (bootTimestampIso != null && !bootTimestampIso.isBlank())
            ? bootTimestampIso
            : Instant.now().toString();
    String json =
        "{\n"
            + "  \"robotId\": \""
            + escapeJson(robotId)
            + "\",\n"
            + "  \"profile\": \""
            + escapeJson(profile)
            + "\",\n"
            + "  \"gitSha\": \""
            + escapeJson(effectiveGitSha)
            + "\",\n"
            + "  \"configSha256\": \""
            + escapeJson(configSha256)
            + "\",\n"
            + "  \"batteryName\": \""
            + escapeJson(batteryName)
            + "\",\n"
            + "  \"loopPeriodMs\": "
            + loopPeriodMs
            + ",\n"
            + "  \"bootTimestampIso\": \""
            + escapeJson(effectiveBootIso)
            + "\"\n"
            + "}\n";
    Files.writeString(metaFile, json, StandardCharsets.UTF_8);
    return metaFile;
  }

  /** Reads `gitSha` from `<deployDir>/deploy-stamp.json` if present, else returns `"unknown"`. */
  public static String readDeployedGitSha(Path deployDir) {
    if (deployDir == null) {
      return "unknown";
    }
    Path stampFile = deployDir.resolve("deploy-stamp.json");
    if (!Files.isRegularFile(stampFile)) {
      return "unknown";
    }
    try {
      String raw = Files.readString(stampFile, StandardCharsets.UTF_8);
      int idx = raw.indexOf("\"gitSha\"");
      if (idx < 0) {
        return "unknown";
      }
      int colon = raw.indexOf(':', idx);
      int firstQuote = raw.indexOf('"', colon + 1);
      int secondQuote = raw.indexOf('"', firstQuote + 1);
      if (colon > 0 && firstQuote > colon && secondQuote > firstQuote) {
        String val = raw.substring(firstQuote + 1, secondQuote).trim();
        return val.isEmpty() ? "unknown" : val;
      }
    } catch (IOException ignored) {
    }
    return "unknown";
  }

  private static String escapeJson(String s) {
    if (s == null) {
      return "";
    }
    return s.replace("\\", "\\\\").replace("\"", "\\\"");
  }

  /** Recursively deletes oldest `.wpilog` and `.hoot` files until total size <= maxBytes. */
  public static void cleanOldLogs(File logDir, long maxBytes) {
    List<File> logFiles = new ArrayList<>();
    collectLogFiles(logDir, logFiles);
    if (logFiles.isEmpty()) {
      return;
    }

    logFiles.sort(Comparator.comparingLong(File::lastModified));
    long totalSize = 0;
    for (File f : logFiles) {
      totalSize += f.length();
    }

    int i = 0;
    while (totalSize > maxBytes && i < logFiles.size()) {
      File target = logFiles.get(i);
      long fileSize = target.length();
      if (target.delete()) {
        totalSize -= fileSize;
      }
      i++;
    }
  }

  private static void collectLogFiles(File dir, List<File> out) {
    File[] children = dir.listFiles();
    if (children == null) {
      return;
    }
    for (File f : children) {
      if (f.isDirectory()) {
        collectLogFiles(f, out);
      } else if (f.getName().endsWith(".wpilog") || f.getName().endsWith(".hoot")) {
        out.add(f);
      }
    }
  }
}
