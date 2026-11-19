#!/usr/bin/env python3
"""
Reads build/precision-scorecard.json, prints the ASCII precision table, generates SVG badges,
and posts per-tier quantitative precision statuses back to GitHub Commit Status API.
Works identically for manual `gcloud builds submit` (via .git/HEAD) and GitHub triggers.
"""
import json
import os
import pathlib
import re
import urllib.error
import urllib.request


def resolve_git_metadata():
    commit_sha = os.environ.get("COMMIT_SHA", "").strip()
    branch = os.environ.get("BRANCH_NAME", "").strip()
    gh_repo = os.environ.get("GH_REPO", "").strip()

    git_dir = pathlib.Path(".git")
    head_file = git_dir / "HEAD"
    if not commit_sha and head_file.exists():
        head_val = head_file.read_text().strip()
        if head_val.startswith("ref: "):
            ref_path = head_val[5:].strip()
            if not branch and ref_path.startswith("refs/heads/"):
                branch = ref_path[len("refs/heads/") :]
            ref_file = git_dir / ref_path
            if ref_file.exists():
                commit_sha = ref_file.read_text().strip()
            else:
                packed = git_dir / "packed-refs"
                if packed.exists():
                    for line in packed.read_text().splitlines():
                        if line.endswith(" " + ref_path):
                            commit_sha = line.split()[0]
                            break
                reftable_dir = git_dir / "reftable"
                if (not commit_sha or branch == ".invalid") and reftable_dir.exists():
                    for rf in sorted(reftable_dir.glob("*.ref")):
                        raw = rf.read_bytes()
                        m_br = re.search(rb"refs/heads/([A-Za-z0-9_.-]+)\x00\x01(.{20})", raw, re.DOTALL)
                        if m_br:
                            branch = m_br.group(1).decode("utf-8", errors="ignore")
                            commit_sha = m_br.group(2).hex()
        else:
            commit_sha = head_val

    if not gh_repo:
        cfg_file = git_dir / "config"
        if cfg_file.exists():
            m = re.search(
                r"github\.com[:/]([A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+?)(?:\.git)?\s*$",
                cfg_file.read_text(),
                re.MULTILINE,
            )
            if m:
                gh_repo = m.group(1)

    if not commit_sha:
        commit_sha = "local-dev"
    if not branch:
        branch = "main"
    if not gh_repo:
        gh_repo = "FRC-Flux-Robotics/Flux-SystemCore-2027"

    return commit_sha, branch, gh_repo


def write_svg_badge(out_path: pathlib.Path, label: str, value: str, passed: bool, update_docs: bool = True):
    color = "#16a34a" if passed else "#dc2626"
    left_w = max(96, int(len(label) * 7.2 + 24))
    right_w = max(104, int(len(value) * 7.6 + 26))
    total_w = left_w + right_w
    left_cx = left_w / 2.0
    right_cx = left_w + (right_w / 2.0)
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{total_w}" height="22" viewBox="0 0 {total_w} 22" role="img" aria-label="{label}: {value}">
  <clipPath id="r">
    <rect width="{total_w}" height="22" rx="4" fill="#fff"/>
  </clipPath>
  <g clip-path="url(#r)">
    <rect width="{left_w}" height="22" fill="#212020"/>
    <rect x="{left_w}" width="{right_w}" height="22" fill="{color}"/>
  </g>
  <g fill="#fff" text-anchor="middle" font-family="DejaVu Sans,Verdana,Geneva,sans-serif" font-size="11">
    <text x="{left_cx:.1f}" y="15">{label}</text>
    <text x="{right_cx:.1f}" y="15" font-weight="bold">{value}</text>
  </g>
</svg>
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(svg, encoding="utf-8")
    if update_docs:
        docs_badge = pathlib.Path("docs/badges") / out_path.name
        docs_badge.parent.mkdir(parents=True, exist_ok=True)
        docs_badge.write_text(svg, encoding="utf-8")


def post_github_status(gh_repo: str, sha: str, token: str, context: str, state: str, desc: str, target_url: str):
    if not token or len(sha) != 40:
        return "SKIPPED (no token or local non-40-char SHA)"
    url = f"https://api.github.com/repos/{gh_repo}/statuses/{sha}"
    payload = json.dumps({
        "state": state,
        "context": context,
        "description": desc[:139],
        "target_url": target_url,
    }).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=payload,
        headers={
            "Authorization": f"Bearer {token}",
            "Accept": "application/vnd.github+json",
            "Content-Type": "application/json",
            "User-Agent": "Flux-10413-CloudBuild-Precision-Gate",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return f"HTTP {resp.status} OK"
    except urllib.error.HTTPError as e:
        if e.code == 422:
            return "SKIPPED (422: commit SHA not pushed to GitHub yet)"
        return f"HTTP {e.code} ({e.reason})"
    except Exception as e:
        return f"WARN ({e})"


def fail_desc(tier_num: int, entries: list[dict], gradle_rc: int) -> str:
    if not entries:
        return f"FAIL: Gradle/JUnit aborted (rc={gradle_rc}) with 0 Tier {tier_num} metrics"
    failed = [e for e in entries if not e.get("passed")]
    if not failed:
        return f"FAIL: Gradle/JUnit exited rc={gradle_rc} during Tier {tier_num} suite"
    sample = ", ".join(f"{e['id']}={e['measured']:.2f}" for e in failed[:2])
    return f"FAIL ({len(failed)}/{len(entries)}): {sample}"


def main():
    sha, branch, gh_repo = resolve_git_metadata()
    short_sha = sha[:8]
    gh_run_id = os.environ.get("GITHUB_RUN_ID", "").strip()

    rc_file = pathlib.Path("build/gradle-exit-code.txt")
    gradle_rc = 0
    if rc_file.exists():
        try:
            gradle_rc = int(rc_file.read_text(encoding="utf-8").strip())
        except ValueError:
            gradle_rc = 1

    pkg_files = sorted(pathlib.Path("build/packages").glob("*systemcore-2027_fluxpkg.zip"))
    pkg_name = pkg_files[0].name if pkg_files else "STABLE_v2027.1.0_systemcore-2027_fluxpkg.zip"

    if gh_run_id:
        scorecard_url = f"https://github.com/{gh_repo}/actions/runs/{gh_run_id}"
        pkg_url = f"https://github.com/{gh_repo}/actions/runs/{gh_run_id}"
    else:
        gcs_base_url = f"https://storage.cloud.google.com/btvaroska/frc-10413/builds/{short_sha}"
        scorecard_url = f"{gcs_base_url}/precision-scorecard.html"
        pkg_url = f"{gcs_base_url}/{pkg_name}"

    txt_path = pathlib.Path("build/precision-scorecard.txt")
    if txt_path.exists():
        print("\n" + txt_path.read_text(encoding="utf-8"))

    json_path = pathlib.Path("build/precision-scorecard.json")
    if json_path.exists():
        data = json.loads(json_path.read_text(encoding="utf-8"))
        entries = data.get("entries", [])
    else:
        print(f"ERROR: build/precision-scorecard.json missing (gradle_rc={gradle_rc}); posting FAIL statuses.")
        entries = []
        if gradle_rc == 0:
            gradle_rc = 1
    by_id = {e["id"]: e for e in entries}

    badges_dir = pathlib.Path("build/badges")
    badges_dir.mkdir(parents=True, exist_ok=True)

    t1_entries = [e for e in entries if e["tier"] == 1]
    t2_entries = [e for e in entries if e["tier"] == 2]
    t3_entries = [e for e in entries if e["tier"] == 3]
    t4_entries = [e for e in entries if e["tier"] == 4]
    t5_entries = [e for e in entries if e["tier"] == 5]

    t1_pass = bool(t1_entries) and all(e["passed"] for e in t1_entries)
    t2_pass = bool(t2_entries) and all(e["passed"] for e in t2_entries)
    t3_pass = bool(t3_entries) and all(e["passed"] for e in t3_entries)
    t4_pass = bool(t4_entries) and all(e["passed"] for e in t4_entries)
    t5_pass = bool(t5_entries) and all(e["passed"] for e in t5_entries)

    ax_pos = by_id.get("tier2_axxb_pos_error_mm", {}).get("measured", 0.0)
    ax_yaw = by_id.get("tier2_axxb_yaw_error_deg", {}).get("measured", 0.0)
    stator = by_id.get("tier3_talonfx_stator_limit_amps", {}).get("measured", 0.0)
    supply = by_id.get("tier3_talonfx_supply_limit_amps", {}).get("measured", 0.0)
    vel_err = by_id.get("tier3_kraken_velocity_tracking_err_rps", {}).get("measured", 0.0)
    chaos_vbat = by_id.get("tier4_chaos_min_vbat", {}).get("measured", 0.0)
    cam_q_ms = by_id.get("tier4_cam_quarantine_ms", {}).get("measured", 0.0)
    nytr_dropouts = by_id.get("tier5_nytr_replay_dropouts", {}).get("measured", 0.0)
    nytr_vbat = by_id.get("tier5_nytr_replay_min_vbat", {}).get("measured", 0.0)

    update_docs = gradle_rc == 0
    write_svg_badge(badges_dir / "tier1-hygiene.svg", "Tier 1 Hygiene", "100% PASS" if t1_pass else "FAIL", t1_pass, update_docs)
    write_svg_badge(badges_dir / "tier2-solvers.svg", "AX=XB Solver", f"±{ax_pos:.2f}mm / ±{ax_yaw:.2f}°" if t2_pass else "FAIL", t2_pass, update_docs)
    write_svg_badge(badges_dir / "tier3-ctre-sim.svg", "CTRE TalonFX", f"{stator:.0f}A/{supply:.0f}A | ±{vel_err:.2f}RPS" if t3_pass else "FAIL", t3_pass, update_docs)
    write_svg_badge(badges_dir / "tier4-chaos-match.svg", "150s Chaos Match", f"Vmin {chaos_vbat:.2f}V | {cam_q_ms:.0f}ms Q" if t4_pass else "FAIL", t4_pass, update_docs)
    write_svg_badge(badges_dir / "tier5-nytr-replay.svg", "NYTR Replay", f"{nytr_dropouts:.0f} Dropouts | {nytr_vbat:.2f}V" if t5_pass else "FAIL", t5_pass, update_docs)

    all_pass = (gradle_rc == 0) and t1_pass and t2_pass and t3_pass and t4_pass and t5_pass
    statuses = [
        (
            "flux/tier-1-hygiene",
            "success" if t1_pass else "failure",
            "PASS (100%): 0 SmartDashboard, 0 loop allocs, BackTo2026 & STUDENTS.md synced"
            if t1_pass
            else fail_desc(1, t1_entries, gradle_rc),
            scorecard_url,
        ),
        (
            "flux/tier-2-solvers",
            "success" if t2_pass else "failure",
            f"A+: AX=XB ±{ax_pos:.2f}mm / ±{ax_yaw:.2f}° (<5mm); Geofence 10k/10k 0 breaches"
            if t2_pass
            else fail_desc(2, t2_entries, gradle_rc),
            scorecard_url,
        ),
        (
            "flux/tier-3-ctre-sim",
            "success" if t3_pass else "failure",
            f"PASS: TalonFX readback {stator:.0f}A Stator/{supply:.0f}A Supply; Kraken ±{vel_err:.2f} RPS"
            if t3_pass
            else fail_desc(3, t3_entries, gradle_rc),
            scorecard_url,
        ),
        (
            "flux/tier-4-chaos-match",
            "success" if t4_pass else "failure",
            f"PASS: 150s 50x Match Vmin={chaos_vbat:.2f}V (28mOhm), Cam1 Quarantine={cam_q_ms:.0f}ms, 2/2 Dejams"
            if t4_pass
            else fail_desc(4, t4_entries, gradle_rc),
            scorecard_url,
        ),
        (
            "flux/tier-5-nytr-replay",
            "success" if t5_pass else "failure",
            f"A+: NYTR Qual 53 .wpilog Replay -> {nytr_dropouts:.0f} dropouts (vs 15 legacy), Vmin={nytr_vbat:.2f}V"
            if t5_pass
            else fail_desc(5, t5_entries, gradle_rc),
            scorecard_url,
        ),
        (
            "flux/packages",
            "success" if all_pass else "failure",
            f"READY: {pkg_name} ({short_sha})"
            if all_pass
            else f"BLOCKED: verification failed (rc={gradle_rc}); package not promoted",
            pkg_url,
        ),
    ]

    gh_token = os.environ.get("GITHUB_TOKEN", "").strip()
    print(f"Git Metadata Resolved -> Repo: {gh_repo} | Branch: {branch} | SHA: {sha}")
    print("----------------------------------------------------------------------------------------------------")
    for ctx, state, desc, url in statuses:
        res = post_github_status(gh_repo, sha, gh_token, ctx, state, desc, url)
        print(f" [GitHub Status] {ctx:<22} | {state.upper():<7} | {desc} -> {res}")
    print("----------------------------------------------------------------------------------------------------")

    meta_out = pathlib.Path("build/git-resolved.env")
    meta_out.parent.mkdir(parents=True, exist_ok=True)
    meta_out.write_text(f"SHORT_SHA={short_sha}\nFULL_SHA={sha}\nBRANCH={branch}\nGH_REPO={gh_repo}\n", encoding="utf-8")
    if not all_pass:
        raise SystemExit(gradle_rc if gradle_rc != 0 else 1)


if __name__ == "__main__":
    main()
