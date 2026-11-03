package frc.simharness;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Quantitative precision collector for Tiers 1-5. Writes build/precision-scorecard.json,
 * build/precision-scorecard.txt, and build/precision-scorecard.html for Cloud Build and GitHub
 * Commit Status reporting.
 */
public final class PrecisionReporter {
  public record Entry(
      int tier,
      String id,
      String label,
      double measured,
      double targetBest,
      double maxTolerance,
      String unit,
      boolean lowerIsBetter,
      boolean passed,
      double precisionPct,
      String grade) {}

  private static final Map<String, Entry> ENTRIES =
      Collections.synchronizedMap(new LinkedHashMap<>());

  static {
    Runtime.getRuntime().addShutdownHook(new Thread(PrecisionReporter::flushToDisk));
  }

  private PrecisionReporter() {}

  public static void recordLowerIsBetter(
      int tier,
      String id,
      String label,
      double measured,
      double targetBest,
      double maxTolerance,
      String unit) {
    boolean passed = measured <= maxTolerance + 1e-9;
    double span = Math.max(1e-9, maxTolerance - targetBest);
    double rawPct = 100.0 * (1.0 - Math.max(0.0, measured - targetBest) / span);
    double pct = passed ? Math.max(70.0, Math.min(100.0, rawPct)) : Math.max(0.0, rawPct);
    String grade = computeGrade(passed, pct);
    ENTRIES.put(
        id,
        new Entry(
            tier, id, label, measured, targetBest, maxTolerance, unit, true, passed, pct, grade));
  }

  public static void recordExactTarget(
      int tier,
      String id,
      String label,
      double measured,
      double targetValue,
      double maxAbsError,
      String unit) {
    double err = Math.abs(measured - targetValue);
    boolean passed = err <= maxAbsError + 1e-9;
    double pct =
        passed
            ? Math.max(80.0, 100.0 * (1.0 - err / Math.max(1e-9, maxAbsError)))
            : 0.0;
    String grade = computeGrade(passed, pct);
    ENTRIES.put(
        id,
        new Entry(
            tier,
            id,
            label,
            measured,
            targetValue,
            maxAbsError,
            unit,
            false,
            passed,
            pct,
            grade));
  }

  public static void recordHigherIsBetter(
      int tier,
      String id,
      String label,
      double measured,
      double targetBest,
      double minAcceptable,
      String unit) {
    boolean passed = measured >= minAcceptable - 1e-9;
    double span = Math.max(1e-9, targetBest - minAcceptable);
    double rawPct = 100.0 * Math.min(1.0, Math.max(0.0, (measured - minAcceptable) / span));
    double pct = passed ? Math.max(75.0, Math.min(100.0, 75.0 + 25.0 * (rawPct / 100.0))) : 0.0;
    String grade = computeGrade(passed, pct);
    ENTRIES.put(
        id,
        new Entry(
            tier,
            id,
            label,
            measured,
            targetBest,
            -minAcceptable,
            unit,
            false,
            passed,
            pct,
            grade));
  }

  private static String computeGrade(boolean passed, double pct) {
    if (!passed) {
      return "FAIL";
    }
    if (pct >= 93.0) {
      return "A+";
    }
    if (pct >= 85.0) {
      return "A";
    }
    return "PASS";
  }

  public static synchronized void flushToDisk() {
    if (ENTRIES.isEmpty()) {
      return;
    }
    try {
      Path buildDir = Paths.get("build");
      Files.createDirectories(buildDir);
      List<Entry> list = new ArrayList<>(ENTRIES.values());

      writeJson(buildDir.resolve("precision-scorecard.json"), list);
      writeTextTable(buildDir.resolve("precision-scorecard.txt"), list);
      writeHtmlReport(buildDir.resolve("precision-scorecard.html"), list);
    } catch (IOException e) {
      System.err.println("PrecisionReporter failed to write scorecard: " + e.getMessage());
    }
  }

  private static void writeJson(Path out, List<Entry> list) throws IOException {
    StringBuilder sb = new StringBuilder();
    sb.append("{\n  \"entries\": [\n");
    for (int i = 0; i < list.size(); i++) {
      Entry e = list.get(i);
      sb.append(
          String.format(
              Locale.US,
              "    {\"tier\": %d, \"id\": \"%s\", \"label\": \"%s\", \"measured\": %.4f, "
                  + "\"targetBest\": %.4f, \"maxTolerance\": %.4f, \"unit\": \"%s\", "
                  + "\"lowerIsBetter\": %s, \"passed\": %s, \"precisionPct\": %.1f, \"grade\": \"%s\"}%s\n",
              e.tier(),
              e.id(),
              e.label(),
              e.measured(),
              e.targetBest(),
              e.maxTolerance(),
              e.unit(),
              e.lowerIsBetter(),
              e.passed(),
              e.precisionPct(),
              e.grade(),
              (i + 1 < list.size()) ? "," : ""));
    }
    sb.append("  ]\n}\n");
    Files.writeString(out, sb.toString(), StandardCharsets.UTF_8);
  }

  private static void writeTextTable(Path out, List<Entry> list) throws IOException {
    StringBuilder sb = new StringBuilder();
    sb.append("====================================================================================================\n");
    sb.append(" FLUX 10413 — AUTOMATED TIER 1-5 PRECISION & HARDWARE-SIMULATION SCORECARD\n");
    sb.append("====================================================================================================\n");
    sb.append(
        String.format(
            Locale.US,
            " %-6s | %-36s | %-18s | %-18s | %-10s\n",
            "TIER",
            "CHECK / SOLVER / HARDWARE GATE",
            "MEASURED VALUE",
            "GATE THRESHOLD",
            "GRADE"));
    sb.append("----------------------------------------------------------------------------------------------------\n");
    for (Entry e : list) {
      String measuredStr = String.format(Locale.US, "%.3f %s", e.measured(), e.unit());
      String gateStr =
          e.lowerIsBetter()
              ? String.format(Locale.US, "<= %.3f %s", e.maxTolerance(), e.unit())
              : (e.maxTolerance() < 0
                  ? String.format(Locale.US, ">= %.3f %s", Math.abs(e.maxTolerance()), e.unit())
                  : String.format(
                      Locale.US, "%.1f +/- %.2f %s", e.targetBest(), e.maxTolerance(), e.unit()));
      String gradeStr = String.format(Locale.US, "%s (%.0f%%)", e.grade(), e.precisionPct());
      sb.append(
          String.format(
              Locale.US,
              " Tier %d | %-36s | %-18s | %-18s | %-10s\n",
              e.tier(),
              e.label(),
              measuredStr,
              gateStr,
              gradeStr));
    }
    sb.append("====================================================================================================\n");
    Files.writeString(out, sb.toString(), StandardCharsets.UTF_8);
  }

  private static void writeHtmlReport(Path out, List<Entry> list) throws IOException {
    StringBuilder sb = new StringBuilder();
    sb.append("<!DOCTYPE html><html><head><meta charset='utf-8'>");
    sb.append("<title>FLUX 10413 — Tier 1-5 Precision Scorecard</title>");
    sb.append("<style>body{background:#212020;color:#fff;font-family:sans-serif;padding:24px;}");
    sb.append("h1{color:#2B7BFF;}table{border-collapse:collapse;width:100%;margin-top:16px;}");
    sb.append("th,td{border:1px solid #3a3a3a;padding:10px 14px;text-align:left;}");
    sb.append("th{background:#254BA0;}tr:nth-child(even){background:#2a2929;}");
    sb.append(".pass{color:#4ade80;font-weight:bold;}.fail{color:#f87171;font-weight:bold;}</style>");
    sb.append("</head><body><h1>FLUX 10413 — Automated Tier 1–5 Precision Scorecard</h1>");
    sb.append("<table><tr><th>Tier</th><th>Metric</th><th>Measured</th><th>Gate</th><th>Grade</th></tr>");
    for (Entry e : list) {
      String cls = e.passed() ? "pass" : "fail";
      sb.append(
          String.format(
              Locale.US,
              "<tr><td>Tier %d</td><td>%s</td><td>%.3f %s</td><td>%.3f %s</td><td class='%s'>%s (%.0f%%)</td></tr>",
              e.tier(),
              e.label(),
              e.measured(),
              e.unit(),
              Math.abs(e.maxTolerance()),
              e.unit(),
              cls,
              e.grade(),
              e.precisionPct()));
    }
    sb.append("</table></body></html>");
    Files.writeString(out, sb.toString(), StandardCharsets.UTF_8);
  }
}
