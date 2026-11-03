#!/usr/bin/env python3
"""
Auto-generates:
  1. docs/class-map.svg  — Vector Class & Dependency Map
  2. docs/class-map.html — Self-contained interactive HTML Class & Relationship Explorer
                           styled in Pelham / FLUX Robotics (FRC Team 10413) official brand colors & logo
  3. STUDENTS.md         — Auto-synced student guide

Parses live Java source files under src/main/java/** and src/test/java/** so the diagram never drifts out of sync.
Columns flow Left -> Right from top-level entry points down to low-level utilities:
  Col 0 (Left):  1. Robot & Container (Main, Robot, RobotContainer, RobotIdentity)
  Col 1:         2. Subsystems, Vision & Safety (PositionMechanism, VelocityMechanism, ElevatorArmKinematics, VirtualGeofence, FaultSupervisor, ZeroPriorBootstrapSolver)
  Col 2:         3. Hardware & Sim IO (*IO) (MechanismIO, MechanismIOTalonFX, MechanismIOSim, FieldPositions)
  Col 3 (Right): 4. Config, CAN & Log Utils (FluxFig, PhoenixSignals, PhoenixUtil, LogFileManager, LoggedTracer)
"""
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = ROOT / "src" / "main" / "java"
TEST_DIR = ROOT / "src" / "test" / "java"
MECH_CSV = ROOT / "src" / "main" / "deploy" / "configs" / "mechanisms.csv"
DOCS_DIR = ROOT / "docs"
SVG_OUT = DOCS_DIR / "class-map.svg"
HTML_OUT = DOCS_DIR / "class-map.html"
STUDENTS_OUT = ROOT / "STUDENTS.md"
GITHUB_BASE = "https://github.com/FRC-Flux-Robotics/Flux-SystemCore-2027/blob/main"
LOGO_URL = "https://images.squarespace-cdn.com/content/v1/678bfe582830c94f0965d1b8/c581de14-67ff-442b-b36b-1959857c428e/Caitlin+B%26W+Logo+JPEG.jpg?format=1500w"

# Explicit vertical sort priority within each column so top-level classes appear at the top
SORT_ORDER = {
    "Main": 1,
    "Robot": 2,
    "RobotContainer": 3,
    "RobotIdentity": 4,
    "Superstructure": 5,
    "StudentAutoRoutines": 6,
    "StudentExercises": 7,
    "DriveCommands": 8,
    "DriveToPose": 9,
    "PreMatchCalibrator": 10,
    "PitSystemCheckCmd": 11,
    "HardStopHomingCmd": 12,
    "ZeroPriorBootstrapCmd": 13,
    "ShopStressSuiteCmd": 14,
    "Blitz15MinCmd": 15,
    "Drive": 20,
    "Module": 21,
    "SwerveConfig": 22,
    "PhoenixOdometryThread": 23,
    "Vision": 24,
    "PositionMechanism": 26,
    "VelocityMechanism": 27,
    "ElevatorArmKinematics": 28,
    "VirtualGeofence": 29,
    "FaultSupervisor": 30,
    "ZeroPriorBootstrapSolver": 31,
    "ModuleIO": 40,
    "ModuleIOTalonFX": 41,
    "ModuleIOSim": 42,
    "GyroIO": 43,
    "GyroIOPigeon2": 44,
    "VisionIO": 45,
    "VisionIOLimelightSystemCore": 46,
    "VisionIOPhotonVision": 47,
    "VisionIOSim": 48,
    "MechanismIO": 51,
    "MechanismIOTalonFX": 52,
    "MechanismIOSim": 53,
    "BatteryIO": 54,
    "FluxFig": 60,
    "FieldPositions": 61,
    "LoggedTunableNumber": 62,
    "PiecewiseSensitivity": 63,
    "SafeAutoBuilder": 64,
    "VirtualBatterySim": 65,
    "PhoenixSignals": 66,
    "PhoenixUtil": 67,
    "LogFileManager": 68,
    "LoggedTracer": 69,
}


def extract_javadoc_summary(text: str, default_summary: str = "Flux 2027 module") -> str:
    doc_match = re.search(r"/\*\*(.*?)\*/", text, re.DOTALL)
    if not doc_match:
        return default_summary
    raw_doc = doc_match.group(1)
    cleaned = []
    for dl in raw_doc.splitlines():
        dl = re.sub(r"^\s*\*\s?", "", dl).strip()
        if dl.startswith("@") or dl.startswith("<p>") or dl.startswith("<ol>"):
            break
        if dl:
            cleaned.append(dl)
    if not cleaned:
        return default_summary
    joined = " ".join(cleaned)
    joined = re.sub(r"\{@link\s+#?([\w\.]+)\}", r"\1", joined)
    joined = re.sub(r"\{@code\s+([^}]+)\}", r"`\1`", joined)
    return joined


def assign_column(class_name: str, pkg: str) -> int:
    if (
        class_name
        in (
            "Main",
            "Robot",
            "RobotContainer",
            "RobotIdentity",
            "Superstructure",
            "StudentAutoRoutines",
            "StudentExercises",
        )
        or pkg in ("frc.robot", "frc.robot.autos", "frc.robot.exercises", "frc.lib.commands")
    ):
        return 0
    if class_name in (
        "ModuleIO",
        "ModuleIOTalonFX",
        "ModuleIOSim",
        "GyroIO",
        "GyroIOPigeon2",
        "VisionIO",
        "VisionIOLimelightSystemCore",
        "VisionIOPhotonVision",
        "VisionIOSim",
        "MechanismIO",
        "MechanismIOTalonFX",
        "MechanismIOSim",
        "BatteryIO",
    ):
        return 2
    if (
        class_name
        in (
            "Drive",
            "Module",
            "SwerveConfig",
            "PhoenixOdometryThread",
            "Vision",
            "PositionMechanism",
            "VelocityMechanism",
            "ElevatorArmKinematics",
            "VirtualGeofence",
            "FaultSupervisor",
            "ZeroPriorBootstrapSolver",
        )
        or pkg in ("frc.lib.drive", "frc.lib.safety", "frc.lib.vision", "frc.robot.mechanisms")
    ):
        return 1
    return 3


def parse_java_files():
    java_files = sorted(SRC_DIR.rglob("*.java"))
    test_files = sorted(TEST_DIR.rglob("*.java")) if TEST_DIR.exists() else []
    classes = []
    tests = []
    todos = []

    for tf in test_files:
        rel_path = tf.relative_to(ROOT).as_posix()
        text = tf.read_text(encoding="utf-8")
        lines = text.splitlines()
        tests.append(
            {
                "id": tf.stem,
                "name": tf.name,
                "relPath": rel_path,
                "lines": len(lines),
                "summary": extract_javadoc_summary(text, "Verification gate test"),
                "rawText": text,
            }
        )

    for f in java_files:
        rel_path = f.relative_to(ROOT).as_posix()
        text = f.read_text(encoding="utf-8")
        lines = text.splitlines()
        line_count = len(lines)
        class_name = f.stem

        pkg_match = re.search(r"^\s*package\s+([\w\.]+)\s*;", text, re.MULTILINE)
        pkg = pkg_match.group(1) if pkg_match else "frc.robot"
        summary = extract_javadoc_summary(text)

        methods = []
        for line in lines:
            s = line.strip()
            if (
                (s.startswith("public ") or s.startswith("default "))
                and "(" in s
                and not s.startswith("public class ")
                and not s.startswith("public final class ")
                and not s.startswith("public interface ")
                and not s.startswith("public enum ")
                and not s.startswith("public record ")
            ):
                sig = s.split("{")[0].strip().rstrip(";")
                sig = re.sub(r"^public\s+(static\s+)?(final\s+)?", "", sig)
                sig = re.sub(r"^default\s+", "", sig)
                if len(sig) < 78:
                    methods.append(sig)

        for idx, line in enumerate(lines):
            if "TODO(student)" in line:
                msg = line.split("TODO(student)", 1)[1].lstrip(": -").strip()
                todos.append((f"[`{f.name}:{idx + 1}`]({rel_path}#L{idx + 1})", msg))
            elif "// STUDENT:" in line:
                msg = line.split("// STUDENT:", 1)[1].strip()
                todos.append((f"[`{f.name}:{idx + 1}`]({rel_path}#L{idx + 1})", f"**STUDENT:** {msg}"))

        col = assign_column(class_name, pkg)

        tested_by = [
            t["id"]
            for t in tests
            if re.search(r"\b" + re.escape(class_name) + r"\b", t["rawText"])
        ]

        classes.append(
            {
                "id": class_name,
                "name": f.name,
                "pkg": pkg,
                "relPath": rel_path,
                "ghUrl": f"{GITHUB_BASE}/{rel_path}",
                "lines": line_count,
                "summary": summary,
                "methods": methods[:8],
                "testedBy": tested_by,
                "col": col,
                "rawText": text,
            }
        )

    class_ids = {c["id"] for c in classes}
    edges = []
    for caller in classes:
        for dep_id in sorted(class_ids):
            if dep_id == caller["id"]:
                continue
            if re.search(r"\b" + re.escape(dep_id) + r"\b", caller["rawText"]):
                edges.append((caller["id"], dep_id))

    fluxfig_tested_by = [
        t["id"]
        for t in tests
        if "FluxFig" in t["rawText"] or "MechanismSpec" in t["rawText"]
    ]
    fluxfig_node = {
        "id": "FluxFig",
        "name": "fluxfig-1.0.0.jar",
        "pkg": "frc.lib.fluxfig (JAR)",
        "relPath": "libs/fluxfig-1.0.0.jar",
        "ghUrl": "https://github.com/FRC-Flux-Robotics/FluxFig/blob/main/src/main/java/frc/lib/fluxfig/FluxFig.java",
        "lines": 340,
        "summary": "Standalone pure-Java 17 CSV config engine (parameters.csv, mechanisms.csv, cameras.csv) providing MechanismSpec.",
        "methods": [
            "getMechanismSpec(robot, name)",
            "getTable(tab, Record.class)",
            "validateStrict()",
            "getConfigSha256()",
        ],
        "testedBy": fluxfig_tested_by,
        "col": 3,
    }
    for caller in classes:
        if "FluxFig" in caller["rawText"] or "MechanismSpec" in caller["rawText"]:
            edges.append((caller["id"], "FluxFig"))

    for c in classes:
        del c["rawText"]
    for t in tests:
        t["ghUrl"] = f"{GITHUB_BASE}/{t['relPath']}"
        del t["rawText"]

    all_nodes = sorted(
        classes + [fluxfig_node],
        key=lambda n: (n["col"], SORT_ORDER.get(n["id"], 99), n["id"]),
    )
    return all_nodes, edges, tests, todos


def generate_svg(nodes, edges):
    DOCS_DIR.mkdir(parents=True, exist_ok=True)
    col_buckets = {0: [], 1: [], 2: [], 3: []}
    for n in nodes:
        col_buckets[n["col"]].append(n)

    max_rows = max(len(v) for v in col_buckets.values())
    width = 1040
    height = max(430, 96 + max_rows * 68)
    col_x = [24, 280, 536, 792]
    box_w = 224
    box_h = 52

    pos = {}
    for c_idx, bucket in col_buckets.items():
        for r_idx, n in enumerate(bucket):
            pos[n["id"]] = (col_x[c_idx], 76 + r_idx * 66)

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="100%">',
        "  <defs>",
        '    <marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">',
        '      <path d="M 0 1 L 10 5 L 0 9 z" fill="#2B7BFF" opacity="0.9"/>',
        "    </marker>",
        "  </defs>",
        f'  <rect width="{width}" height="{height}" rx="10" fill="#212020" stroke="#254BA0" stroke-width="2"/>',
        '  <text x="24" y="30" fill="#FFFFFF" font-family="system-ui, -apple-system, sans-serif" font-size="16" font-weight="700">FLUX 10413 — Flux-SystemCore-2027 Live Class &amp; Dependency Map</text>',
        '  <text x="580" y="30" fill="#DEDEDE" font-family="system-ui, -apple-system, sans-serif" font-size="12">🔵 Pelham Royal (frc.robot.*)   ⬛ Dark Slate (frc.lib.*)   🟢 FluxFig JAR</text>',
        '  <line x1="20" y1="42" x2="1020" y2="42" stroke="#254BA0" stroke-width="1.5"/>',
    ]

    headers = [
        "1. ROBOT & CONTAINER (frc.robot.*)",
        "2. SUBSYSTEMS, VISION & SAFETY",
        "3. HARDWARE & SIM IO (*IO)",
        "4. CONFIG, CAN & LOG UTILS",
    ]
    for c_idx, h_text in enumerate(headers):
        svg.append(
            f'  <text x="{col_x[c_idx] + 4}" y="62" fill="#2B7BFF" font-family="system-ui, -apple-system, sans-serif" font-size="11.5" font-weight="700">{h_text}</text>'
        )

    for u, v in edges:
        if u not in pos or v not in pos:
            continue
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        if x1 == x2:
            sx, sy = x1 + box_w * 0.5, y1 + box_h if y1 < y2 else y1
            ex, ey = x2 + box_w * 0.5, y2 if y1 < y2 else y2 + box_h
            d = f"M {sx} {sy} L {ex} {ey}"
        else:
            sx, sy = (x1 + box_w, y1 + box_h * 0.5) if x1 < x2 else (x1, y1 + box_h * 0.5)
            ex, ey = (x2, y2 + box_h * 0.5) if x1 < x2 else (x2 + box_w, y2 + box_h * 0.5)
            mx = (sx + ex) * 0.5
            d = f"M {sx} {sy} C {mx} {sy}, {mx} {ey}, {ex} {ey}"
        svg.append(
            f'  <path d="{d}" fill="none" stroke="#2B7BFF" stroke-width="1.6" stroke-opacity="0.55" marker-end="url(#arrow)"/>'
        )

    for n in nodes:
        x, y = pos[n["id"]]
        if n["id"] == "FluxFig":
            fill, stroke, badge = "#0D6E52", "#10B981", "#A7F3D0"
        elif n["pkg"].startswith("frc.robot"):
            fill, stroke, badge = "#254BA0", "#2B7BFF", "#DEDEDE"
        else:
            fill, stroke, badge = "#2B303A", "#BFBFBF", "#DEDEDE"

        svg.append(f'  <g transform="translate({x}, {y})">')
        svg.append(
            f'    <rect width="{box_w}" height="{box_h}" rx="7" fill="{fill}" stroke="{stroke}" stroke-width="1.6"/>'
        )
        svg.append(
            f'    <text x="12" y="21" fill="#FFFFFF" font-family="ui-monospace, SFMono-Regular, monospace" font-size="12.5" font-weight="700">{n["id"]}</text>'
        )
        svg.append(
            f'    <text x="12" y="39" fill="{badge}" font-family="system-ui, -apple-system, sans-serif" font-size="11">{n["pkg"]} · {n["lines"]}L</text>'
        )
        svg.append("  </g>")

    svg.append("</svg>")
    SVG_OUT.write_text("\n".join(svg) + "\n", encoding="utf-8")


def generate_html(nodes, edges, tests):
    DOCS_DIR.mkdir(parents=True, exist_ok=True)
    nodes_json = json.dumps(nodes, indent=2)
    edges_json = json.dumps(edges)
    tests_json = json.dumps(tests, indent=2)

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <link rel="icon" type="image/svg+xml" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 64 64'%3E%3Crect width='64' height='64' rx='14' fill='%23212020' stroke='%232B7BFF' stroke-width='4'/%3E%3Cpolygon points='36,8 16,35 30,35 26,56 48,28 34,28' fill='%232B7BFF'/%3E%3C/svg%3E" />
  <title>FLUX Robotics (FRC 10413) — Flux-SystemCore-2027 Interactive Class &amp; Dependency Map</title>
  <link href="https://fonts.googleapis.com/css2?family=Archivo+Black&family=Space+Grotesk:wght@400;500;600;700&display=swap" rel="stylesheet">
  <style>
    :root {{
      --black: #212020;
      --royal: #254BA0;
      --flux-blue: #2B7BFF;
      --silver: #BFBFBF;
      --light-silver: #DEDEDE;
      --bg: #F2F3F5;
      --card: #FFFFFF;
      --muted: #5A606E;
      --green: #0D6E52;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      padding: 20px 24px 44px 24px;
      background: var(--bg);
      color: var(--black);
      font-family: 'Space Grotesk', system-ui, -apple-system, sans-serif;
    }}
    a {{ color: var(--royal); text-decoration: none; font-weight: 600; }}
    a:hover {{ color: var(--flux-blue); text-decoration: underline; }}
    .container {{ max-width: 1480px; margin: 0 auto; display: flex; flex-direction: column; gap: 18px; }}
    .header {{
      background: var(--card);
      border-top: 6px solid var(--royal);
      border-radius: 10px;
      padding: 16px 22px;
      display: flex;
      flex-wrap: wrap;
      justify-content: space-between;
      align-items: center;
      gap: 16px;
      box-shadow: 0 2px 8px rgba(33, 32, 32, 0.08);
    }}
    .header-left {{
      display: flex;
      align-items: center;
      gap: 18px;
    }}
    .logo {{
      height: 58px;
      width: auto;
      border-radius: 6px;
      border: 1px solid var(--light-silver);
    }}
    .brand-title {{
      font-family: 'Archivo Black', 'Space Grotesk', sans-serif;
      font-size: 21px;
      letter-spacing: 0.02em;
      color: var(--black);
      margin: 4px 0 3px 0;
    }}
    .badge {{
      display: inline-block;
      padding: 3px 10px;
      border-radius: 5px;
      font-size: 11.5px;
      font-weight: 700;
      letter-spacing: 0.03em;
      text-transform: uppercase;
    }}
    .btn {{
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 7px 13px;
      border-radius: 7px;
      font-size: 12.5px;
      font-weight: 700;
      font-family: 'Space Grotesk', sans-serif;
      cursor: pointer;
      border: 1.5px solid var(--light-silver);
      background: var(--card);
      color: var(--black);
      transition: all 0.15s ease;
    }}
    .btn:hover {{
      border-color: var(--flux-blue);
      color: var(--royal);
      background: #F8FAFF;
      text-decoration: none;
    }}
    .btn.active {{
      background: var(--royal);
      border-color: var(--royal);
      color: #FFFFFF;
    }}
    .layout {{
      display: grid;
      grid-template-columns: 1.88fr 1fr;
      gap: 16px;
      align-items: start;
    }}
    @media (max-width: 1080px) {{
      .layout {{ grid-template-columns: 1fr; }}
    }}
    .panel {{
      background: var(--card);
      border: 1px solid var(--light-silver);
      border-radius: 10px;
      padding: 16px 18px;
      box-shadow: 0 2px 6px rgba(33, 32, 32, 0.05);
    }}
    .graph-panel {{
      background: var(--black);
      border-top: 5px solid var(--flux-blue);
      border-radius: 10px;
      padding: 14px;
      box-shadow: 0 3px 10px rgba(33, 32, 32, 0.14);
      overflow-x: auto;
    }}
    .node-box {{ cursor: pointer; transition: opacity 0.15s ease; }}
    .chip {{
      display: inline-block;
      margin: 2px;
      padding: 4px 9px;
      border-radius: 5px;
      background: var(--bg);
      color: var(--black);
      border: 1px solid var(--silver);
      font-size: 12px;
      font-weight: 600;
      font-family: ui-monospace, SFMono-Regular, monospace;
      cursor: pointer;
    }}
    .chip:hover {{
      border-color: var(--flux-blue);
      background: #EBF2FF;
      color: var(--royal);
    }}
    .chip-test {{
      border-color: var(--royal);
      background: rgba(37, 75, 160, 0.09);
      color: var(--royal);
    }}
    .chip-test:hover {{
      background: var(--royal);
      color: #FFFFFF;
      text-decoration: none;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }}
    th, td {{
      text-align: left;
      padding: 10px 12px;
      border-bottom: 1px solid var(--light-silver);
      vertical-align: top;
    }}
    th {{
      background: var(--black);
      color: #FFFFFF;
      font-size: 11.5px;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      font-weight: 700;
    }}
    tr.row-item {{ cursor: pointer; transition: background 0.12s ease; }}
    tr.row-item:nth-child(even) td {{ background: #F9FAFB; }}
    tr.row-item:hover td {{ background: #EBF2FF; }}
    tr.row-item.selected td {{ background: #DCE8FF; border-top: 1px solid var(--flux-blue); border-bottom: 1px solid var(--flux-blue); }}
    .search-input {{
      background: var(--bg);
      border: 1.5px solid var(--silver);
      color: var(--black);
      padding: 7px 12px;
      border-radius: 7px;
      font-size: 13px;
      font-family: 'Space Grotesk', sans-serif;
      min-width: 240px;
    }}
    .search-input:focus {{
      outline: none;
      border-color: var(--flux-blue);
      background: #FFFFFF;
    }}
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <div class="header-left">
        <img class="logo" src="{LOGO_URL}" alt="FLUX Robotics Team 10413 Logo" />
        <div>
          <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap;">
            <span class="badge" style="background:var(--black);color:#FFFFFF;">FRC Team 10413 · FLUX Robotics</span>
            <span class="badge" style="background:var(--royal);color:#FFFFFF;">Auto-Synced via ./gradlew updateStudentDocs</span>
          </div>
          <div class="brand-title">FLUX-SYSTEMCORE-2027 — CLASS &amp; DEPENDENCY EXPLORER</div>
          <div style="color:var(--muted);font-size:13px;">
            Flows <b>Left &rarr; Right</b> from <b>1. Robot &amp; Container</b> (`Main`, `Robot`, `RobotContainer`) &rarr; <b>2. Subsystems, Vision &amp; Safety</b> &rarr; <b>3. Hardware/Sim IO</b> &rarr; <b>4. Config, CAN &amp; Log Utils</b>.
          </div>
        </div>
      </div>
      <div style="display:flex;flex-direction:column;align-items:flex-end;gap:8px;">
        <div style="display:flex;gap:14px;font-size:12.5px;color:var(--black);font-weight:600;flex-wrap:wrap;">
          <span><span style="display:inline-block;width:11px;height:11px;border-radius:3px;background:#254BA0;border:1.5px solid #2B7BFF;vertical-align:middle;margin-right:4px;"></span><b>frc.robot.*</b> (Entry &amp; 2027 Specific)</span>
          <span><span style="display:inline-block;width:11px;height:11px;border-radius:3px;background:#2B303A;border:1.5px solid #BFBFBF;vertical-align:middle;margin-right:4px;"></span><b>frc.lib.*</b> (Synced via <code>backTo2026</code>)</span>
          <span><span style="display:inline-block;width:11px;height:11px;border-radius:3px;background:#0D6E52;border:1.5px solid #10B981;vertical-align:middle;margin-right:4px;"></span><b>FluxFig JAR</b></span>
        </div>
        <div style="display:flex;gap:8px;flex-wrap:wrap;">
          <button class="btn" id="btn-show-all" onclick="toggleShowAll()">Show All Connections</button>
          <a class="btn" href="https://github.com/FRC-Flux-Robotics/Flux-SystemCore-2027" target="_blank" rel="noopener">GitHub Repo ↗</a>
        </div>
      </div>
    </div>

    <div class="layout">
      <div class="graph-panel">
        <svg id="graph-svg" viewBox="0 0 980 480" width="100%"></svg>
      </div>
      <div class="panel" id="inspector" style="position:sticky;top:16px;border-top:5px solid var(--royal);"></div>
    </div>

    <div class="panel" style="border-top:5px solid var(--black);">
      <div style="display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:12px;margin-bottom:14px;">
        <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap;" id="filter-pills">
          <button class="btn active" data-filter="ALL" onclick="setFilter('ALL')">All Classes</button>
          <button class="btn" data-filter="ROBOT" onclick="setFilter('ROBOT')">1. Robot &amp; Container (frc.robot)</button>
          <button class="btn" data-filter="SUBSYS" onclick="setFilter('SUBSYS')">2. Subsystems, Vision &amp; Safety</button>
          <button class="btn" data-filter="IO" onclick="setFilter('IO')">3. Hardware &amp; Sim IO (*IO)</button>
          <button class="btn" data-filter="UTIL" onclick="setFilter('UTIL')">4. Config, CAN &amp; Log Utils</button>
          <button class="btn" data-filter="TESTS" onclick="setFilter('TESTS')">5-Tier Verification Tests</button>
        </div>
        <input type="text" id="search-box" class="search-input" placeholder="Search class, package, or method..." oninput="renderTable()" />
      </div>
      <div style="overflow-x:auto;">
        <table>
          <thead>
            <tr>
              <th style="width:190px;">Class</th>
              <th style="width:185px;">Package &amp; Scope</th>
              <th style="width:70px;">Lines</th>
              <th>Purpose &amp; Public API</th>
              <th style="width:220px;">Relations &amp; Tests</th>
            </tr>
          </thead>
          <tbody id="class-table-body"></tbody>
        </table>
      </div>
    </div>
  </div>

  <script>
    const NODES = {nodes_json};
    const EDGES = {edges_json};
    const TESTS = {tests_json};
    let selectedId = "RobotContainer";
    let showAllEdges = false;
    let activeFilter = "ALL";

    function selectNode(id) {{
      selectedId = id;
      showAllEdges = false;
      document.getElementById("btn-show-all").classList.remove("active");
      render();
      renderTable();
    }}

    function toggleShowAll() {{
      showAllEdges = !showAllEdges;
      document.getElementById("btn-show-all").classList.toggle("active", showAllEdges);
      render();
    }}

    function setFilter(f) {{
      activeFilter = f;
      document.querySelectorAll("#filter-pills .btn").forEach(b => {{
        b.classList.toggle("active", b.getAttribute("data-filter") === f);
      }});
      renderTable();
    }}

    function render() {{
      const svg = document.getElementById("graph-svg");
      const cols = {{0: [], 1: [], 2: [], 3: []}};
      NODES.forEach(n => cols[n.col].push(n));
      const maxRows = Math.max(...Object.values(cols).map(c => c.length));
      svg.setAttribute("viewBox", "0 0 980 " + Math.max(480, 56 + maxRows * 66));
      const colX = [16, 260, 504, 748];
      const boxW = 216;
      const boxH = 50;
      const pos = {{}};
      Object.keys(cols).forEach(c => {{
        cols[c].forEach((n, r) => {{
          pos[n.id] = {{ x: colX[c], y: 40 + r * 66 }};
        }});
      }});

      const connNodes = new Set([selectedId]);
      const connEdges = new Set();
      EDGES.forEach(([u, v], idx) => {{
        if (showAllEdges || u === selectedId || v === selectedId) {{
          connNodes.add(u);
          connNodes.add(v);
          connEdges.add(idx);
        }}
      }});

      let out = `<defs>
        <marker id="arr-hi" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
          <path d="M 0 1 L 10 5 L 0 9 z" fill="#2B7BFF"/>
        </marker>
        <marker id="arr-lo" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
          <path d="M 0 1 L 10 5 L 0 9 z" fill="#BFBFBF"/>
        </marker>
      </defs>`;

      const headers = [
        "1. ROBOT & CONTAINER",
        "2. SUBSYSTEMS, VISION & SAFETY",
        "3. HARDWARE & SIM IO (*IO)",
        "4. CONFIG, CAN & LOG UTILS"
      ];
      headers.forEach((h, i) => {{
        out += `<text x="${{colX[i] + 4}}" y="22" fill="#2B7BFF" font-family="'Space Grotesk', sans-serif" font-size="11.5" font-weight="700" letter-spacing="0.05em">${{h}}</text>`;
      }});

      EDGES.forEach(([u, v], idx) => {{
        if (!pos[u] || !pos[v]) return;
        const p1 = pos[u], p2 = pos[v];
        const act = connEdges.has(idx);
        let d = "";
        if (p1.x === p2.x) {{
          const sy = p1.y < p2.y ? p1.y + boxH : p1.y;
          const ey = p1.y < p2.y ? p2.y : p2.y + boxH;
          d = `M ${{p1.x + boxW/2}} ${{sy}} L ${{p2.x + boxW/2}} ${{ey}}`;
        }} else {{
          const sx = p1.x < p2.x ? p1.x + boxW : p1.x;
          const ex = p1.x < p2.x ? p2.x : p2.x + boxW;
          const sy = p1.y + boxH/2, ey = p2.y + boxH/2;
          const mx = (sx + ex) / 2;
          d = `M ${{sx}} ${{sy}} C ${{mx}} ${{sy}}, ${{mx}} ${{ey}}, ${{ex}} ${{ey}}`;
        }}
        out += `<path d="${{d}}" fill="none" stroke="${{act ? '#2B7BFF' : '#BFBFBF'}}" stroke-width="${{act ? 2.3 : 1.1}}" opacity="${{act ? 0.98 : 0.18}}" marker-end="url(#${{act ? 'arr-hi' : 'arr-lo'}})"/>`;
      }});

      NODES.forEach(n => {{
        const p = pos[n.id];
        const sel = n.id === selectedId;
        const conn = showAllEdges || connNodes.has(n.id);
        let fill = "#2B303A", stroke = sel ? "#2B7BFF" : "#BFBFBF", subColor = "#DEDEDE";
        if (n.id === "FluxFig") {{
          fill = "#0D6E52";
          stroke = sel ? "#FFFFFF" : "#10B981";
          subColor = "#A7F3D0";
        }} else if (n.pkg.startsWith("frc.robot")) {{
          fill = "#254BA0";
          stroke = sel ? "#FFFFFF" : "#2B7BFF";
          subColor = "#DEDEDE";
        }}
        out += `<g class="node-box" transform="translate(${{p.x}}, ${{p.y}})" opacity="${{conn ? 1.0 : 0.28}}" onclick="selectNode('${{n.id}}')">
          <rect width="${{boxW}}" height="${{boxH}}" rx="7" fill="${{fill}}" stroke="${{stroke}}" stroke-width="${{sel ? 2.8 : 1.4}}"/>
          <text x="12" y="20" fill="#FFFFFF" font-family="ui-monospace, SFMono-Regular, monospace" font-size="12.5" font-weight="700">${{n.id}}</text>
          <text x="12" y="38" fill="${{subColor}}" font-family="'Space Grotesk', sans-serif" font-size="11">${{n.pkg}} · ${{n.lines}}L</text>
        </g>`;
      }});

      svg.innerHTML = out;

      const n = NODES.find(x => x.id === selectedId) || NODES[0];
      const dependsOn = EDGES.filter(([u, v]) => u === n.id).map(([u, v]) => v);
      const usedBy = EDGES.filter(([u, v]) => v === n.id).map(([u, v]) => u);
      const scopeTag = n.id === "FluxFig"
        ? `<span class="badge" style="background:var(--green);color:#FFFFFF;">Shared JAR</span>`
        : n.pkg.startsWith("frc.robot")
          ? `<span class="badge" style="background:var(--royal);color:#FFFFFF;">Separate per Repo (2027 vs 2026)</span>`
          : `<span class="badge" style="background:var(--black);color:#FFFFFF;">Synced 1:1 via backTo2026</span>`;

      const testChips = (n.testedBy || []).map(tid => {{
        const tObj = TESTS.find(t => t.id === tid);
        const url = tObj ? tObj.ghUrl : "#";
        return `<a class="chip chip-test" href="${{url}}" target="_blank" rel="noopener" title="Open test on GitHub">${{tid}} ↗</a>`;
      }}).join("");

      document.getElementById("inspector").innerHTML = `
        <div style="display:flex;justify-content:space-between;align-items:center;gap:8px;flex-wrap:wrap;">
          <h2 style="margin:0;font-size:19px;font-family:ui-monospace,monospace;color:var(--black);">${{n.id}}</h2>
          ${{scopeTag}}
        </div>
        <div style="margin-top:6px;display:flex;justify-content:space-between;align-items:center;gap:8px;flex-wrap:wrap;">
          <span style="font-size:12px;color:var(--muted);font-family:ui-monospace,monospace;">${{n.relPath}} (${{n.lines}}L)</span>
          <a class="btn" style="padding:4px 10px;font-size:11.5px;background:var(--royal);color:#FFFFFF;border-color:var(--royal);" href="${{n.ghUrl}}" target="_blank" rel="noopener">View Source on GitHub ↗</a>
        </div>
        <p style="margin:12px 0;padding:12px;background:var(--bg);border-left:4px solid var(--flux-blue);border-radius:6px;font-size:13px;line-height:1.55;color:var(--black);">${{n.summary}}</p>
        <div style="font-size:11.5px;font-weight:700;color:var(--muted);text-transform:uppercase;letter-spacing:0.05em;">Public Constructors &amp; Methods</div>
        <ul style="margin:6px 0 14px 0;padding-left:18px;font-family:ui-monospace,monospace;font-size:12px;color:var(--royal);line-height:1.65;font-weight:600;">
          ${{n.methods.length ? n.methods.map(m => `<li>${{m}}</li>`).join('') : '<li>(Entry / Data Class)</li>'}}
        </ul>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;border-top:1px solid var(--light-silver);padding-top:12px;font-size:12px;">
          <div>
            <div style="color:var(--muted);font-weight:700;margin-bottom:4px;">Uses / Calls &rarr; (${{dependsOn.length}})</div>
            ${{dependsOn.length ? dependsOn.map(id => `<span class="chip" onclick="selectNode('${{id}}')">${{id}}</span>`).join('') : '<span style="color:var(--muted)">Zero internal deps</span>'}}
          </div>
          <div>
            <div style="color:var(--muted);font-weight:700;margin-bottom:4px;">&larr; Called / Used By (${{usedBy.length}})</div>
            ${{usedBy.length ? usedBy.map(id => `<span class="chip" onclick="selectNode('${{id}}')">${{id}}</span>`).join('') : '<span style="color:var(--muted)">JVM Entry Point</span>'}}
          </div>
        </div>
        <div style="border-top:1px solid var(--light-silver);margin-top:12px;padding-top:10px;font-size:12px;">
          <div style="color:var(--muted);font-weight:700;margin-bottom:4px;">Verified By JUnit 5 Gate Tests (${{(n.testedBy || []).length}})</div>
          ${{testChips || '<span style="color:var(--muted)">Covered by Tier 1 Architecture Hygiene Gate</span>'}}
        </div>
      `;
    }}

    function renderTable() {{
      const q = (document.getElementById("search-box").value || "").toLowerCase().trim();
      const tbody = document.getElementById("class-table-body");

      if (activeFilter === "TESTS") {{
        const rows = TESTS.filter(t => !q || t.id.toLowerCase().includes(q) || t.summary.toLowerCase().includes(q));
        tbody.innerHTML = rows.map(t => `
          <tr class="row-item" onclick="window.open('${{t.ghUrl}}', '_blank')">
            <td style="font-family:ui-monospace,monospace;font-weight:700;">
              <a href="${{t.ghUrl}}" target="_blank" rel="noopener">${{t.name}} ↗</a>
            </td>
            <td><span class="badge" style="background:var(--royal);color:#FFFFFF;">frc.simharness (Test)</span></td>
            <td style="font-family:ui-monospace,monospace;font-weight:600;">${{t.lines}}L</td>
            <td>${{t.summary}}</td>
            <td style="color:var(--muted);font-size:12px;">Click row to open JUnit 5 test source on GitHub</td>
          </tr>
        `).join("");
        return;
      }}

      const filtered = NODES.filter(n => {{
        if (activeFilter === "ROBOT" && n.col !== 0) return false;
        if (activeFilter === "SUBSYS" && n.col !== 1) return false;
        if (activeFilter === "IO" && n.col !== 2) return false;
        if (activeFilter === "UTIL" && n.col !== 3) return false;
        if (q) {{
          const hay = (n.id + " " + n.pkg + " " + n.summary + " " + n.methods.join(" ")).toLowerCase();
          if (!hay.includes(q)) return false;
        }}
        return true;
      }});

      tbody.innerHTML = filtered.map(n => {{
        const dependsOn = EDGES.filter(([u, v]) => u === n.id).map(([u, v]) => v);
        const usedBy = EDGES.filter(([u, v]) => v === n.id).map(([u, v]) => u);
        const scopeBadge = n.id === "FluxFig"
          ? `<span class="badge" style="background:var(--green);color:#FFFFFF;">JAR</span>`
          : n.pkg.startsWith("frc.robot")
            ? `<span class="badge" style="background:var(--royal);color:#FFFFFF;">2027 Only</span>`
            : `<span class="badge" style="background:var(--black);color:#FFFFFF;">backTo2026</span>`;
        return `
          <tr class="row-item ${{n.id === selectedId ? 'selected' : ''}}" onclick="selectNode('${{n.id}}')">
            <td style="font-family:ui-monospace,monospace;font-weight:700;">
              <div>${{n.id}}</div>
              <div style="margin-top:4px;font-size:11.5px;font-weight:600;">
                <a href="${{n.ghUrl}}" target="_blank" rel="noopener" onclick="event.stopPropagation()">GitHub Source ↗</a>
              </div>
            </td>
            <td>
              <div style="font-family:ui-monospace,monospace;font-size:12px;color:var(--muted);margin-bottom:4px;">${{n.pkg}}</div>
              ${{scopeBadge}}
            </td>
            <td style="font-family:ui-monospace,monospace;font-weight:600;">${{n.lines}}L</td>
            <td>
              <div style="margin-bottom:6px;line-height:1.45;">${{n.summary}}</div>
              <div style="font-family:ui-monospace,monospace;font-size:11.5px;color:var(--royal);font-weight:600;">
                ${{n.methods.slice(0, 4).join(" · ")}}
              </div>
            </td>
            <td style="font-size:12px;">
              <div><b style="color:var(--muted);">Uses &rarr;:</b> ${{dependsOn.length ? dependsOn.join(", ") : "—"}}</div>
              <div style="margin-top:3px;"><b style="color:var(--muted);">&larr; Used by:</b> ${{usedBy.length ? usedBy.join(", ") : "—"}}</div>
              <div style="margin-top:3px;color:var(--royal);font-weight:600;"><b style="color:var(--muted);">Tests:</b> ${{(n.testedBy || []).join(", ") || "Tier1Hygiene"}}</div>
            </td>
          </tr>
        `;
      }}).join("");
    }}

    render();
    renderTable();
  </script>
</body>
</html>
"""
    HTML_OUT.write_text(html, encoding="utf-8")


def generate_students_md(nodes, todos):
    mech_rows = []
    if MECH_CSV.exists():
        for line in MECH_CSV.read_text(encoding="utf-8").splitlines():
            if line.strip():
                mech_rows.append([c.strip() for c in line.split(",")])

    md = [
        "# FLUX Robotics (Team 10413) — 2027 Student Engineering Guide (`STUDENTS.md`)",
        "",
        "> **Auto-Generated by `./gradlew updateStudentDocs` (`scripts/generate_class_map.py`).**",
        "> Whenever you add or edit a Java class, `FluxFig` CSV config, or `TODO(student)` task, run `./gradlew updateStudentDocs` and add a short entry to [`STUDENT-CHANGELOG.md`](STUDENT-CHANGELOG.md).",
        "",
        "---",
        "",
        "## 1. Visual Class & Dependency Map",
        "",
        "![Flux-SystemCore-2027 Class & Dependency Map](docs/class-map.svg)",
        "",
        "---",
        "",
        "## 2. Student Workstation Setup (Zero Cloud Access Needed) & Operating Modes",
        "",
        "### A. Required Installs & Autonomous 3D Video Pipeline (`./gradlew renderAuto`)",
        "1. **WPILib 2026 VS Code Installer (includes JDK 17 & GradleRIO)** — required.",
        "2. **Git** — required.",
        "3. **PathPlanner GUI** (`winget install MJansen4857.PathPlanner` on Windows or GitHub Releases) — optional; open the repository root (`File -> Open Project`) to draw/edit splines under **Paths** (`src/main/deploy/pathplanner/paths/*.path`), then call `resetPoseToPathStart(drive, \"My Path\")` + `withLoopTimeout(drive.followPathCommand(\"My Path\"), 4.5)` in `StudentAutoRoutines.java` *(note: `renderAuto` runs `.path` files wired in `StudentAutoRoutines.java`, not GUI `.auto` files)*.",
        "4. **AdvantageScope** — optional; recommended for live 3D telemetry (`./gradlew simulateJava`).",
        "5. **`ffmpeg`** (`winget install Gyan.FFmpeg` on Windows or `brew install ffmpeg` on Mac) — optional; enables `.mp4` encoding alongside the interactive 3D `.html` viewer.",
        "- **Autonomous 3D Sandbox (`Track A` Fixed Moves & `Track B` PathPlanner `.path`):** Edit `createStudentDefaultAuto(...)` in [`src/main/java/frc/robot/autos/StudentAutoRoutines.java`](src/main/java/frc/robot/autos/StudentAutoRoutines.java) using either **Track A** (`driveRelative`, `driveTimedVelocity`, `DriveToPose`, `waitLoopSeconds` — see `createFixedMovesExampleAuto`) or **Track B** (`resetPoseToPathStart` + `drive.followPathCommand` — see `createPathPlannerExampleAuto`), then run **`./gradlew renderAuto`** to generate `build/auto-sim/student-auto.html` + `build/auto-sim/student-auto.mp4` (if `../FluxProcessor` is not cloned locally, Gradle automatically shallow-clones and builds public `FRC-Flux-Robotics/FluxProcessor` into gitignored `build/tools/FluxProcessor`).",
        "- **4-Level Student Exercise Kit (`./gradlew checkExercises`):** Open [`src/main/java/frc/robot/exercises/StudentExercises.java`](src/main/java/frc/robot/exercises/StudentExercises.java) and run **`./gradlew checkExercises`** to grade Exercises 1–4 (`FluxFig` safety rules, 2D crossbar keep-out interlocks, swerve pose alignment math, and 2-piece autonomous cycles) in under 3 seconds.",
        "",
        "### B. Two Physical Robot Operating Modes",
        "",
        "- **Mode A — Student Live Sandbox & Spot-Deploy (Default whenever a robot is available $\\ge 20\\text{ min}$):**",
        "  Edit Java or `FluxFig` CSVs in VS Code and run **`./gradlew deploy`** (~6 seconds) to test on the physical robot right away. `frc.lib.*` hardware guardrails (`60 A` stator / `40 A` supply limits, soft limits, and $5\\text{ m} \\times 3\\text{ m}$ `VirtualGeofence`) protect the robot automatically, and 1 click in `FluxTray.jar` (**\"Restore STABLE\"**) restores the baseline build in 4 seconds.",
        "- **Mode B — 15-Minute Black-Box Blitz (Contingency Only when mechanical grants $\\le 15\\text{ min}$):**",
        "  Deploy a pre-built `.fluxpkg.zip` in 4 seconds via `FluxTray.jar` and record a 9-minute flight log for offline `Mode.REPLAY`.",
        "",
        "---",
        "",
        "## 3. Repository Package & Class Map (`frc.robot.*` -> `frc.lib.*`)",
        "",
        "| Class | Package | Sync Scope | Lines | Summary |",
        "| :--- | :--- | :--- | :---: | :--- |",
    ]

    for n in nodes:
        if n["id"] == "FluxFig":
            continue
        scope = (
            "🟣 **Separate per Repo** (`2027` vs `2026`)"
            if n["pkg"].startswith("frc.robot")
            else "🔵 **Synced 1:1** (`./gradlew backTo2026`)"
        )
        md.append(
            f"| [`{n['name']}`]({n['relPath']}) | `{n['pkg']}` | {scope} | `{n['lines']}` | {n['summary']} |"
        )

    md.extend(
        [
            "",
            "---",
            "",
            "## 4. `FluxFig` Mechanism Config Cheat-Sheet (`src/main/deploy/configs/mechanisms.csv`)",
            "",
        ]
    )

    if mech_rows:
        header = mech_rows[0]
        md.append("| " + " | ".join(header) + " |")
        md.append("| " + " | ".join([":---"] * len(header)) + " |")
        for row in mech_rows[1:]:
            md.append("| " + " | ".join(f"`{c}`" for c in row) + " |")

    md.extend(
        [
            "",
            "---",
            "",
            "## 5. Active Student Task Board (`TODO(student)`)",
            "",
            "| Location | Student Task Description |",
            "| :--- | :--- |",
        ]
    )

    if todos:
        for loc, desc in todos:
            md.append(f"| {loc} | {desc} |")
    else:
        md.append(
            "| *(All current tasks complete)* | Ready for next sprint `TODO(student)` scaffolding |"
        )

    STUDENTS_OUT.write_text("\n".join(md) + "\n", encoding="utf-8")


def main():
    nodes, edges, tests, todos = parse_java_files()
    generate_svg(nodes, edges)
    generate_html(nodes, edges, tests)
    generate_students_md(nodes, todos)
    print(
        f"Generated {SVG_OUT.relative_to(ROOT)}, {HTML_OUT.relative_to(ROOT)}, and {STUDENTS_OUT.relative_to(ROOT)} ({len(nodes)} nodes, {len(edges)} edges, {len(tests)} test suites)."
    )


if __name__ == "__main__":
    main()
