#!/usr/bin/env python3
"""Run G1 locomotion calibration across many seeds and aggregate safety gates."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from html import escape
from pathlib import Path
from typing import Any, Iterable
import argparse
import csv
import json
import math
import random
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sim.locomotion_calibration import G1LocomotionCalibrationRunner, load_calibration_suite, write_json  # noqa: E402
from sim.run_context import allocate_run, finalize_manifest, write_manifest  # noqa: E402


GROUP_PREFIXES = {
    "straight": "straight",
    "pure_rotation": "rotation",
    "arc": "arc",
    "reverse_recovery": "reverse",
}

SEED_METRIC_FIELDS = (
    "index",
    "seed",
    "profile",
    "friction_scale",
    "status",
    "suite_completed",
    "reached_goal",
    "success",
    "failure_reason",
    "total_commands",
    "stable_commands",
    "unstable_commands",
    "stable_rate",
    "fall_count",
    "stuck_count",
    "contact_count",
    "non_floor_contact_count",
    "collision_count",
    "max_safe_vx",
    "max_safe_wz",
    "selected_max_forward_mps",
    "avg_stability_score",
    "min_stability_score",
    "max_stability_score",
    "elapsed_wall_s",
    "goal_time_s",
    "simulated_time_s",
    "command_time_s",
    "straight_total",
    "straight_stable",
    "straight_failed",
    "straight_falls",
    "straight_stuck",
    "straight_non_floor_contacts",
    "rotation_total",
    "rotation_stable",
    "rotation_failed",
    "rotation_falls",
    "rotation_stuck",
    "rotation_non_floor_contacts",
    "arc_total",
    "arc_stable",
    "arc_failed",
    "arc_falls",
    "arc_stuck",
    "arc_non_floor_contacts",
    "reverse_total",
    "reverse_stable",
    "reverse_failed",
    "reverse_falls",
    "reverse_stuck",
    "reverse_non_floor_contacts",
    "failure_reason_counts_json",
    "run_dir",
    "summary_path",
)


@dataclass(frozen=True)
class GateSettings:
    min_stable_rate: float
    max_fall_count: int
    max_stuck_count: int
    max_non_floor_contact_count: int
    min_safe_vx: float
    min_safe_wz: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "min_stable_rate": self.min_stable_rate,
            "max_fall_count": self.max_fall_count,
            "max_stuck_count": self.max_stuck_count,
            "max_non_floor_contact_count": self.max_non_floor_contact_count,
            "min_safe_vx": self.min_safe_vx,
            "min_safe_wz": self.min_safe_wz,
        }


def parse_seed_values(values: list[str] | None) -> list[int]:
    if not values:
        return []
    seeds: list[int] = []
    for value in values:
        for item in value.replace(",", " ").split():
            seeds.append(int(item))
    return seeds


def generate_random_seeds(base_seed: int, count: int) -> list[int]:
    if count < 1:
        raise ValueError("--count must be >= 1 when --seeds is not provided")
    rng = random.Random(int(base_seed))
    seeds: list[int] = []
    seen: set[int] = set()
    while len(seeds) < count:
        seed = rng.randrange(1, 2_147_483_647)
        if seed not in seen:
            seeds.append(seed)
            seen.add(seed)
    return seeds


def friction_scale_for_seed(base_seed: int, seed: int, minimum: float, maximum: float) -> float:
    if minimum <= 0.0 or maximum <= 0.0:
        raise ValueError("friction scale bounds must be positive")
    if minimum > maximum:
        raise ValueError("--friction-scale-min cannot be greater than --friction-scale-max")
    if math.isclose(minimum, maximum):
        return float(minimum)
    rng = random.Random(f"{int(base_seed)}:{int(seed)}:g1-locomotion-friction")
    return float(rng.uniform(minimum, maximum))


def build_seed_metric(
    *,
    index: int,
    seed: int,
    profile: str,
    friction_scale: float,
    run_dir: Path,
    elapsed_wall_s: float,
    suite_timing: dict[str, float],
    gates: GateSettings,
    summary: dict[str, Any],
    command_rows: list[dict[str, Any]],
    error: str = "",
) -> dict[str, Any]:
    total_commands = _int(summary.get("total_commands"), len(command_rows))
    stable_commands = _int(summary.get("stable_commands"), sum(1 for row in command_rows if _row_stable(row)))
    unstable_commands = max(0, total_commands - stable_commands)
    fall_count = _int(summary.get("fall_count"), sum(1 for row in command_rows if row.get("fell")))
    stuck_count = _int(summary.get("stuck_count"), sum(1 for row in command_rows if row.get("stuck")))
    non_floor_contact_count = _int(
        summary.get("non_floor_contact_count"),
        sum(1 for row in command_rows if row.get("non_floor_contact")),
    )
    contact_count = sum(1 for row in command_rows if row.get("contact"))
    scores = [_number(row.get("stability_score")) for row in command_rows]
    limits = summary.get("recommended_safe_limits", {})
    if not isinstance(limits, dict):
        limits = {}
    locomotion_calibration = summary.get("locomotion_calibration", {})
    if not isinstance(locomotion_calibration, dict):
        locomotion_calibration = {}
    max_safe_vx = _number(limits.get("max_safe_vx"))
    max_safe_wz = _number(limits.get("max_safe_wz"))
    selected_max_forward = _number(locomotion_calibration.get("selected_max_forward_mps"), max_safe_vx)
    stable_rate = _rate(stable_commands, total_commands)
    suite_completed = not error and total_commands > 0 and str(summary.get("status", "")).lower() == "passed"
    failure_reasons = _gate_failures(
        suite_completed=suite_completed,
        error=error,
        stable_rate=stable_rate,
        fall_count=fall_count,
        stuck_count=stuck_count,
        non_floor_contact_count=non_floor_contact_count,
        max_safe_vx=max_safe_vx,
        max_safe_wz=max_safe_wz,
        gates=gates,
    )
    reached_goal = not failure_reasons
    timing_per_command = (
        float(suite_timing.get("warmup_s", 0.0))
        + float(suite_timing.get("command_s", 0.0))
        + float(suite_timing.get("settle_s", 0.0))
    )
    metric: dict[str, Any] = {
        "index": index,
        "seed": seed,
        "profile": profile,
        "friction_scale": friction_scale,
        "status": summary.get("status", "failed"),
        "suite_completed": bool(suite_completed),
        "reached_goal": bool(reached_goal),
        "success": bool(reached_goal),
        "failure_reason": ";".join(failure_reasons),
        "total_commands": total_commands,
        "stable_commands": stable_commands,
        "unstable_commands": unstable_commands,
        "stable_rate": stable_rate,
        "fall_count": fall_count,
        "stuck_count": stuck_count,
        "contact_count": contact_count,
        "non_floor_contact_count": non_floor_contact_count,
        "collision_count": non_floor_contact_count,
        "max_safe_vx": max_safe_vx,
        "max_safe_wz": max_safe_wz,
        "selected_max_forward_mps": selected_max_forward,
        "avg_stability_score": _mean(scores),
        "min_stability_score": min(scores) if scores else 0.0,
        "max_stability_score": max(scores) if scores else 0.0,
        "elapsed_wall_s": elapsed_wall_s,
        "goal_time_s": elapsed_wall_s if reached_goal else None,
        "simulated_time_s": total_commands * timing_per_command,
        "command_time_s": total_commands * float(suite_timing.get("command_s", 0.0)),
        "failure_reason_counts_json": json.dumps(_failure_reason_counts(command_rows), sort_keys=True),
        "run_dir": str(run_dir),
        "summary_path": str(run_dir / "summary.json"),
    }
    metric.update(_flatten_group_metrics(command_rows))
    return metric


def run_seed(
    *,
    index: int,
    seed: int,
    args: argparse.Namespace,
    suite: Any,
    gates: GateSettings,
    batch_dir: Path,
) -> dict[str, Any]:
    friction_scale = friction_scale_for_seed(args.seed, seed, args.friction_scale_min, args.friction_scale_max)
    run_dir = batch_dir / f"seed-{seed}"
    run_dir.mkdir(parents=True, exist_ok=True)
    write_manifest(
        run_dir,
        command="g1_locomotion_seed_batch_member",
        seed=seed,
        parameters={
            "batch_seed": args.seed,
            "profile": suite.profile,
            "calibration_config": str(args.calibration_config),
            "friction_scale": friction_scale,
        },
        project_root=PROJECT_ROOT,
        config_path=args.config,
    )
    started = time.monotonic()
    summary: dict[str, Any] = {}
    command_rows: list[dict[str, Any]] = []
    error = ""
    try:
        runner = G1LocomotionCalibrationRunner(
            project_config_path=args.config,
            calibration_config_path=args.calibration_config,
            suite=suite,
            run_dir=run_dir,
            seed=seed,
            unitree_rl_gym_repo=args.unitree_rl_gym_repo,
            friction_scale=friction_scale,
        )
        summary = runner.run()
        command_rows = _read_json_list(run_dir / "command_results.json")
        finalize_manifest(run_dir, str(summary.get("status", "passed")), run_dir / "summary.json")
    except Exception as exc:
        error = str(exc)
        summary = {
            "schema_version": 1,
            "status": "failed",
            "seed": seed,
            "profile": suite.profile,
            "error": error,
            "environment": {"friction_scale": friction_scale},
            "artifacts": {"run_dir": str(run_dir)},
        }
        write_json(run_dir / "summary.json", summary)
        finalize_manifest(run_dir, "failed", run_dir / "summary.json")
    elapsed_wall_s = time.monotonic() - started
    return build_seed_metric(
        index=index,
        seed=seed,
        profile=suite.profile,
        friction_scale=friction_scale,
        run_dir=run_dir,
        elapsed_wall_s=elapsed_wall_s,
        suite_timing=suite.timing,
        gates=gates,
        summary=summary,
        command_rows=command_rows,
        error=error,
    )


def build_batch_report(
    *,
    batch_dir: Path,
    seeds: list[int],
    rows: list[dict[str, Any]],
    args: argparse.Namespace,
    gates: GateSettings,
) -> dict[str, Any]:
    successes = [row for row in rows if row.get("reached_goal")]
    completed = [row for row in rows if row.get("suite_completed")]
    goal_times = [_number(row.get("goal_time_s")) for row in successes if row.get("goal_time_s") is not None]
    failure_counts = Counter(str(row.get("failure_reason") or "success") for row in rows)
    return {
        "schema_version": 1,
        "batch_dir": str(batch_dir),
        "profile": args.profile,
        "base_seed": args.seed,
        "seeds_requested": len(seeds),
        "seeds_completed": len(completed),
        "seeds_evaluated": len(rows),
        "goal_reached_count": len(successes),
        "success_rate": _rate(len(successes), len(rows)),
        "average_time_to_goal_s": _mean(goal_times),
        "median_time_to_goal_s": _median(goal_times),
        "total_elapsed_wall_s": sum(_number(row.get("elapsed_wall_s")) for row in rows),
        "total_fall_count": sum(_int(row.get("fall_count")) for row in rows),
        "total_stuck_count": sum(_int(row.get("stuck_count")) for row in rows),
        "total_collision_count": sum(_int(row.get("collision_count")) for row in rows),
        "total_non_floor_contact_count": sum(_int(row.get("non_floor_contact_count")) for row in rows),
        "average_stable_rate": _mean([_number(row.get("stable_rate")) for row in rows]),
        "average_max_safe_vx": _mean([_number(row.get("max_safe_vx")) for row in rows]),
        "average_max_safe_wz": _mean([_number(row.get("max_safe_wz")) for row in rows]),
        "best_seed_by_score": _best_seed(rows),
        "failure_reason_counts": dict(sorted(failure_counts.items())),
        "gates": gates.to_dict(),
        "friction_scale_range": [args.friction_scale_min, args.friction_scale_max],
        "seeds": seeds,
        "by_group": _aggregate_groups(rows),
        "artifacts": {
            "seed_metrics_csv": str(batch_dir / "seed_metrics.csv"),
            "seed_metrics_json": str(batch_dir / "seed_metrics.json"),
            "summary_json": str(batch_dir / "summary.json"),
            "report_md": str(batch_dir / "report.md"),
            "dashboard_html": str(batch_dir / "dashboard.html"),
            "seeds_txt": str(batch_dir / "seeds.txt"),
        },
    }


def write_batch_outputs(batch_dir: Path, report: dict[str, Any], rows: list[dict[str, Any]]) -> None:
    _write_csv(batch_dir / "seed_metrics.csv", rows)
    write_json(batch_dir / "seed_metrics.json", rows)
    write_json(batch_dir / "summary.json", report)
    (batch_dir / "report.md").write_text(render_markdown_report(report, rows), encoding="utf-8")
    (batch_dir / "dashboard.html").write_text(render_html_report(report, rows), encoding="utf-8")


def render_markdown_report(report: dict[str, Any], rows: list[dict[str, Any]]) -> str:
    lines = [
        "# G1 Locomotion Calibration Seed Batch",
        "",
        f"- Profile: `{report['profile']}`",
        f"- Seeds evaluated: {report['seeds_evaluated']}/{report['seeds_requested']}",
        f"- Success rate: {_format_percent(report['success_rate'])}",
        f"- Average time to goal: {_format_number(report['average_time_to_goal_s'])} s",
        f"- Falls: {report['total_fall_count']}",
        f"- Stuck commands: {report['total_stuck_count']}",
        f"- Collision/contact proxy: {report['total_collision_count']} non-floor contacts",
        f"- Average stable command rate: {_format_percent(report['average_stable_rate'])}",
        f"- Average max safe vx: {_format_number(report['average_max_safe_vx'])} m/s",
        f"- Average max safe wz: {_format_number(report['average_max_safe_wz'])} rad/s",
        "",
        "The calibration goal means the seed completed the command sweep and passed the configured safety gates. It is not a maze-navigation goal.",
        "",
        "## Gate Settings",
        "",
    ]
    lines.extend(f"- `{key}`: {value}" for key, value in report["gates"].items())
    lines.extend(["", "## Failure Reasons", ""])
    lines.extend(f"- `{key}`: {value}" for key, value in report["failure_reason_counts"].items())
    lines.extend(["", "## Per-Seed Results", ""])
    lines.append("| Seed | Result | Stable rate | Falls | Stuck | Non-floor contacts | Max vx | Max wz | Time |")
    lines.append("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for row in rows[:100]:
        result = "success" if row.get("reached_goal") else (row.get("failure_reason") or "failed")
        lines.append(
            f"| {row['seed']} | {result} | {_format_percent(row['stable_rate'])} | "
            f"{row['fall_count']} | {row['stuck_count']} | {row['non_floor_contact_count']} | "
            f"{_format_number(row['max_safe_vx'])} | {_format_number(row['max_safe_wz'])} | "
            f"{_format_number(row['elapsed_wall_s'])} |"
        )
    lines.append("")
    return "\n".join(lines)


def render_html_report(report: dict[str, Any], rows: list[dict[str, Any]]) -> str:
    row_html = "".join(
        "<tr>"
        f"<td>{int(row['seed'])}</td>"
        f"<td>{escape('success' if row.get('reached_goal') else str(row.get('failure_reason') or 'failed'))}</td>"
        f"<td>{_format_percent(row.get('stable_rate'))}</td>"
        f"<td>{int(row.get('fall_count', 0))}</td>"
        f"<td>{int(row.get('stuck_count', 0))}</td>"
        f"<td>{int(row.get('non_floor_contact_count', 0))}</td>"
        f"<td>{_format_number(row.get('max_safe_vx'))}</td>"
        f"<td>{_format_number(row.get('max_safe_wz'))}</td>"
        f"<td>{_format_number(row.get('elapsed_wall_s'))}</td>"
        f"<td>{escape(str(row.get('run_dir')))}</td>"
        "</tr>"
        for row in rows
    )
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>G1 Locomotion Calibration Seed Batch</title>
  <style>
    body {{ margin: 0; font-family: Arial, sans-serif; color: #1f2933; background: #f7f8fa; }}
    main {{ max-width: 1240px; margin: 0 auto; padding: 28px; }}
    table {{ width: 100%; border-collapse: collapse; background: white; border: 1px solid #d9e0ea; }}
    th, td {{ padding: 8px 10px; border-bottom: 1px solid #edf0f4; text-align: left; vertical-align: top; }}
    th {{ background: #eef3f8; }}
    .kpis {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; margin: 16px 0; }}
    .kpi {{ background: white; border: 1px solid #d9e0ea; padding: 12px; }}
    .label {{ color: #52606d; font-size: 13px; }}
    .value {{ font-size: 24px; font-weight: 700; margin-top: 4px; }}
  </style>
</head>
<body>
<main>
  <h1>G1 Locomotion Calibration Seed Batch</h1>
  <p>The goal is a calibration-suite gate, not a navigation target. Ground truth is used only for offline calibration metrics.</p>
  <section class="kpis">
    <div class="kpi"><div class="label">Success Rate</div><div class="value">{_format_percent(report['success_rate'])}</div></div>
    <div class="kpi"><div class="label">Seeds</div><div class="value">{report['seeds_evaluated']}/{report['seeds_requested']}</div></div>
    <div class="kpi"><div class="label">Average Goal Time</div><div class="value">{_format_number(report['average_time_to_goal_s'])} s</div></div>
    <div class="kpi"><div class="label">Falls</div><div class="value">{report['total_fall_count']}</div></div>
    <div class="kpi"><div class="label">Stuck Commands</div><div class="value">{report['total_stuck_count']}</div></div>
    <div class="kpi"><div class="label">Non-floor Contacts</div><div class="value">{report['total_non_floor_contact_count']}</div></div>
  </section>
  <table>
    <thead><tr><th>Seed</th><th>Result</th><th>Stable Rate</th><th>Falls</th><th>Stuck</th><th>Non-floor Contacts</th><th>Max vx</th><th>Max wz</th><th>Wall Time</th><th>Run Dir</th></tr></thead>
    <tbody>{row_html}</tbody>
  </table>
</main>
</body>
</html>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=123, help="Base seed used to generate the random seed list.")
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--seeds", nargs="*", default=[], help="Explicit seeds. Accepts space or comma separated values.")
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs" / "default.yaml")
    parser.add_argument(
        "--calibration-config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "g1_locomotion_calibration.yaml",
    )
    parser.add_argument("--profile", choices=("balanced", "smoke"), default="balanced")
    parser.add_argument("--run-root", type=Path, default=PROJECT_ROOT / "runs" / "calibration")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--unitree-rl-gym-repo", type=Path, default=PROJECT_ROOT / "third_party" / "unitree_rl_gym")
    parser.add_argument("--friction-scale-min", type=float, default=0.75)
    parser.add_argument("--friction-scale-max", type=float, default=1.15)
    parser.add_argument("--min-stable-rate", type=float, default=0.70)
    parser.add_argument("--max-fall-count", type=int, default=0)
    parser.add_argument("--max-stuck-count", type=int, default=0)
    parser.add_argument("--max-non-floor-contact-count", type=int, default=5)
    parser.add_argument("--min-safe-vx", type=float, default=0.40)
    parser.add_argument("--min-safe-wz", type=float, default=0.40)
    parser.add_argument("--fail-on-gate-failure", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    explicit_seeds = parse_seed_values(args.seeds)
    seeds = explicit_seeds or generate_random_seeds(args.seed, args.count)
    suite = load_calibration_suite(args.calibration_config, args.profile)
    gates = GateSettings(
        min_stable_rate=args.min_stable_rate,
        max_fall_count=args.max_fall_count,
        max_stuck_count=args.max_stuck_count,
        max_non_floor_contact_count=args.max_non_floor_contact_count,
        min_safe_vx=args.min_safe_vx,
        min_safe_wz=args.min_safe_wz,
    )
    if args.output_dir is not None:
        batch_dir = args.output_dir
        batch_dir.mkdir(parents=True, exist_ok=True)
    else:
        batch_dir = allocate_run(
            args.run_root,
            "g1_locomotion_seed_batch",
            args.seed,
            {"count": len(seeds), "profile": suite.profile},
        )
    write_manifest(
        batch_dir,
        command="g1_locomotion_seed_batch",
        seed=args.seed,
        parameters={
            "count": len(seeds),
            "explicit_seeds": bool(explicit_seeds),
            "profile": suite.profile,
            "calibration_config": str(args.calibration_config),
            "friction_scale_min": args.friction_scale_min,
            "friction_scale_max": args.friction_scale_max,
            "gates": gates.to_dict(),
        },
        project_root=PROJECT_ROOT,
        config_path=args.config,
    )
    (batch_dir / "seeds.txt").write_text("\n".join(str(seed) for seed in seeds) + "\n", encoding="utf-8")
    print(f"batch_dir: {batch_dir}", flush=True)
    print(f"seeds: {len(seeds)} profile={suite.profile}", flush=True)

    rows: list[dict[str, Any]] = []
    for index, seed in enumerate(seeds, start=1):
        friction_scale = friction_scale_for_seed(args.seed, seed, args.friction_scale_min, args.friction_scale_max)
        print(
            f"[{index}/{len(seeds)}] seed {seed}: start friction_scale={friction_scale:.4f}",
            flush=True,
        )
        row = run_seed(index=index, seed=seed, args=args, suite=suite, gates=gates, batch_dir=batch_dir)
        rows.append(row)
        report = build_batch_report(batch_dir=batch_dir, seeds=seeds, rows=rows, args=args, gates=gates)
        write_batch_outputs(batch_dir, report, rows)
        result = "success" if row["reached_goal"] else row["failure_reason"]
        print(
            f"[{index}/{len(seeds)}] seed {seed}: {result} "
            f"stable_rate={_format_percent(row['stable_rate'])} falls={row['fall_count']} stuck={row['stuck_count']} "
            f"elapsed_s={row['elapsed_wall_s']:.1f}",
            flush=True,
        )

    report = build_batch_report(batch_dir=batch_dir, seeds=seeds, rows=rows, args=args, gates=gates)
    write_batch_outputs(batch_dir, report, rows)
    final_status = "passed" if report["goal_reached_count"] == len(seeds) else "completed_with_gate_failures"
    if any(str(row.get("status")).lower() == "failed" for row in rows):
        final_status = "failed"
    finalize_manifest(batch_dir, final_status, batch_dir / "summary.json")
    print(
        json.dumps(
            {
                "status": final_status,
                "batch_dir": str(batch_dir),
                "success_rate": report["success_rate"],
                "summary": str(batch_dir / "summary.json"),
                "csv": str(batch_dir / "seed_metrics.csv"),
                "report": str(batch_dir / "report.md"),
                "dashboard": str(batch_dir / "dashboard.html"),
            },
            indent=2,
            sort_keys=True,
        ),
        flush=True,
    )
    if args.fail_on_gate_failure and report["goal_reached_count"] != len(seeds):
        return 1
    return 0


def _gate_failures(
    *,
    suite_completed: bool,
    error: str,
    stable_rate: float,
    fall_count: int,
    stuck_count: int,
    non_floor_contact_count: int,
    max_safe_vx: float,
    max_safe_wz: float,
    gates: GateSettings,
) -> list[str]:
    failures: list[str] = []
    if error:
        failures.append("runtime_error")
    if not suite_completed:
        failures.append("suite_incomplete")
    if stable_rate < gates.min_stable_rate:
        failures.append("low_stable_rate")
    if fall_count > gates.max_fall_count:
        failures.append("fall")
    if stuck_count > gates.max_stuck_count:
        failures.append("stuck")
    if non_floor_contact_count > gates.max_non_floor_contact_count:
        failures.append("non_floor_contact")
    if max_safe_vx < gates.min_safe_vx:
        failures.append("low_safe_vx")
    if max_safe_wz < gates.min_safe_wz:
        failures.append("low_safe_wz")
    return failures


def _flatten_group_metrics(command_rows: list[dict[str, Any]]) -> dict[str, Any]:
    values: dict[str, dict[str, int]] = {
        prefix: {"total": 0, "stable": 0, "failed": 0, "falls": 0, "stuck": 0, "non_floor_contacts": 0}
        for prefix in GROUP_PREFIXES.values()
    }
    for row in command_rows:
        prefix = GROUP_PREFIXES.get(str(row.get("group")))
        if not prefix:
            continue
        values[prefix]["total"] += 1
        if _row_stable(row):
            values[prefix]["stable"] += 1
        else:
            values[prefix]["failed"] += 1
        if row.get("fell"):
            values[prefix]["falls"] += 1
        if row.get("stuck"):
            values[prefix]["stuck"] += 1
        if row.get("non_floor_contact"):
            values[prefix]["non_floor_contacts"] += 1
    flattened: dict[str, Any] = {}
    for prefix, metrics in values.items():
        for key, value in metrics.items():
            flattened[f"{prefix}_{key}"] = value
    return flattened


def _aggregate_groups(rows: list[dict[str, Any]]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for prefix in GROUP_PREFIXES.values():
        total = sum(_int(row.get(f"{prefix}_total")) for row in rows)
        stable = sum(_int(row.get(f"{prefix}_stable")) for row in rows)
        failed = sum(_int(row.get(f"{prefix}_failed")) for row in rows)
        output[prefix] = {
            "total": total,
            "stable": stable,
            "failed": failed,
            "stable_rate": _rate(stable, total),
            "fall_count": sum(_int(row.get(f"{prefix}_falls")) for row in rows),
            "stuck_count": sum(_int(row.get(f"{prefix}_stuck")) for row in rows),
            "non_floor_contact_count": sum(_int(row.get(f"{prefix}_non_floor_contacts")) for row in rows),
        }
    return output


def _failure_reason_counts(command_rows: list[dict[str, Any]]) -> dict[str, int]:
    counts = Counter(str(row.get("failure_reason") or "stable") for row in command_rows)
    return dict(sorted(counts.items()))


def _read_json_list(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    values = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(values, list):
        return []
    return [row for row in values if isinstance(row, dict)]


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SEED_METRIC_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _csv_value(row.get(key)) for key in SEED_METRIC_FIELDS})


def _best_seed(rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    successful = [row for row in rows if row.get("reached_goal")]
    if not successful:
        return None
    best = max(
        successful,
        key=lambda row: (
            _number(row.get("stable_rate")),
            _number(row.get("avg_stability_score")),
            _number(row.get("max_safe_vx")),
            -_number(row.get("elapsed_wall_s")),
        ),
    )
    return {
        "seed": best.get("seed"),
        "stable_rate": best.get("stable_rate"),
        "avg_stability_score": best.get("avg_stability_score"),
        "max_safe_vx": best.get("max_safe_vx"),
        "max_safe_wz": best.get("max_safe_wz"),
        "run_dir": best.get("run_dir"),
    }


def _row_stable(row: dict[str, Any]) -> bool:
    value = row.get("stable")
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes"}
    return bool(value)


def _int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _number(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _mean(values: Iterable[float]) -> float | None:
    rows = [float(value) for value in values if math.isfinite(float(value))]
    return sum(rows) / len(rows) if rows else None


def _median(values: Iterable[float]) -> float | None:
    rows = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not rows:
        return None
    middle = len(rows) // 2
    if len(rows) % 2:
        return rows[middle]
    return (rows[middle - 1] + rows[middle]) / 2.0


def _rate(numerator: int, denominator: int) -> float:
    return float(numerator) / float(denominator) if denominator else 0.0


def _format_percent(value: Any) -> str:
    return f"{_number(value) * 100.0:.1f}%"


def _format_number(value: Any) -> str:
    if value is None:
        return "n/a"
    return f"{_number(value):.3f}"


def _csv_value(value: Any) -> Any:
    if value is None:
        return ""
    return value


if __name__ == "__main__":
    raise SystemExit(main())
