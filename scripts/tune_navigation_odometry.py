#!/usr/bin/env python3
"""Sweep navigation odometry parameters over repeated maze runs.

Ground truth is read only from completed run summaries for offline scoring.
It is never passed into the navigation launch.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import time
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]


DEFAULT_CANDIDATES: list[dict[str, Any]] = [
    {"name": "baseline", "launch_args": {}},
    {
        "name": "tight_icp_gate",
        "launch_args": {
            "icp_min_inlier_ratio": 0.18,
            "icp_max_prediction_translation_error_m": 0.16,
            "icp_max_prediction_yaw_error_rad": 0.20,
        },
    },
    {
        "name": "dense_scan",
        "launch_args": {
            "scan_maximum_points": 260,
            "icp_maximum_correspondence_m": 0.30,
        },
    },
    {
        "name": "corridor_tolerant",
        "launch_args": {
            "icp_min_inlier_ratio": 0.10,
            "icp_maximum_correspondence_m": 0.45,
            "icp_max_prediction_translation_error_m": 0.28,
        },
    },
]


def _stamp() -> str:
    return datetime.now().astimezone().strftime("%Y%m%dT%H%M%S")


def _parse_seeds(values: list[str]) -> list[int]:
    seeds: list[int] = []
    for value in values:
        for item in value.replace(",", " ").split():
            seeds.append(int(item))
    return seeds


def _candidate(value: str) -> dict[str, Any]:
    parsed = json.loads(value)
    if not isinstance(parsed, dict):
        raise argparse.ArgumentTypeError("--candidate must be a JSON object")
    name = str(parsed.get("name", "")).strip()
    launch_args = parsed.get("launch_args", {})
    if not name:
        raise argparse.ArgumentTypeError("--candidate JSON requires a non-empty name")
    if not isinstance(launch_args, dict):
        raise argparse.ArgumentTypeError("--candidate launch_args must be an object")
    return {"name": name, "launch_args": launch_args}


def _launch_arg_value(value: object) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def _launch_args(values: dict[str, object]) -> str:
    return " ".join(f"{key}:={_launch_arg_value(value)}" for key, value in sorted(values.items()))


def _run_logged(command: list[str], *, log_path: Path, env: dict[str, str]) -> tuple[int, float]:
    started = time.monotonic()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log:
        log.write("$ " + shlex.join(command) + "\n\n")
        log.flush()
        completed = subprocess.run(
            command,
            cwd=PROJECT_ROOT,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            check=False,
        )
    return completed.returncode, time.monotonic() - started


def _latest_summary(run_root: Path, seed: int) -> Path | None:
    seed_root = run_root / "navigate" / f"seed-{seed}"
    paths = [
        path for path in seed_root.glob("*/summary.json")
        if path.parent.name != "latest"
    ]
    if not paths:
        return None
    return sorted(paths, key=lambda path: path.parent.name)[-1]


def _numeric(value: object) -> float | None:
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return None


def _median(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def _score(summary: dict[str, Any] | None) -> dict[str, float | None]:
    if summary is None:
        return {
            "score": None,
            "localization_rmse_m": None,
            "final_error_m": None,
            "distance_scale": None,
            "wall_contact_count": None,
        }
    localization = summary.get("localization_evaluation_ground_truth_only", {})
    motion = summary.get("motion", {})
    contact_counts = motion.get("contact_counts", {}) if isinstance(motion, dict) else {}
    rmse = _numeric(localization.get("position_rmse_m")) if isinstance(localization, dict) else None
    final = _numeric(localization.get("final_position_error_m")) if isinstance(localization, dict) else None
    scale = _numeric(localization.get("distance_scale")) if isinstance(localization, dict) else None
    wall_contacts = _numeric(contact_counts.get("wall")) if isinstance(contact_counts, dict) else None
    if rmse is None or final is None:
        score = None
    else:
        scale_penalty = abs(scale - 1.0) if scale is not None else 1.0
        contact_penalty = wall_contacts or 0.0
        score = rmse + 0.5 * final + 0.25 * scale_penalty + contact_penalty
    return {
        "score": score,
        "localization_rmse_m": rmse,
        "final_error_m": final,
        "distance_scale": scale,
        "wall_contact_count": wall_contacts,
    }


def _write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    fieldnames = [
        "candidate",
        "seed",
        "returncode",
        "elapsed_s",
        "score",
        "localization_rmse_m",
        "final_error_m",
        "distance_scale",
        "wall_contact_count",
        "final_status",
        "summary_path",
        "log_path",
        "launch_args",
    ]
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fieldnames})


def _candidate_summary(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    names = sorted({str(row["candidate"]) for row in rows})
    summaries: list[dict[str, Any]] = []
    for name in names:
        subset = [row for row in rows if row["candidate"] == name]
        scores = [float(row["score"]) for row in subset if _numeric(row.get("score")) is not None]
        rmses = [float(row["localization_rmse_m"]) for row in subset if _numeric(row.get("localization_rmse_m")) is not None]
        summaries.append(
            {
                "candidate": name,
                "run_count": len(subset),
                "valid_score_count": len(scores),
                "median_score": _median(scores),
                "median_localization_rmse_m": _median(rmses),
                "nonzero_returncode_count": sum(1 for row in subset if row["returncode"] != 0),
            }
        )
    return summaries


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, default=Path("runs/odom_tuning"))
    parser.add_argument("--seeds", nargs="*", default=["123"])
    parser.add_argument("--cell-size-m", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=240.0)
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--base-ros-domain-id", type=int, default=80)
    parser.add_argument("--candidate", action="append", type=_candidate, default=[])
    parser.add_argument("--skip-prebuild", action="store_true")
    parser.add_argument("--fail-on-run-error", action="store_true")
    args = parser.parse_args()

    if os.environ.get("ROS_DISTRO") != "humble":
        print("tune_navigation_odometry.py must run inside a ROS 2 Humble environment.", flush=True)
        return 2

    seeds = _parse_seeds(args.seeds)
    if not seeds:
        print("At least one seed is required.", flush=True)
        return 2

    candidates = args.candidate or DEFAULT_CANDIDATES
    batch_dir = args.run_root / f"{_stamp()}__scan_odom"
    batch_dir.mkdir(parents=True, exist_ok=True)
    logs_dir = batch_dir / "logs"
    run_rows: list[dict[str, Any]] = []

    if not args.skip_prebuild:
        returncode, elapsed = _run_logged(
            ["make", "prebuild-inner"],
            log_path=logs_dir / "prebuild.log",
            env=os.environ.copy(),
        )
        print(f"prebuild-inner finished rc={returncode} elapsed_s={elapsed:.1f}", flush=True)
        if returncode != 0:
            return returncode

    for candidate_index, candidate in enumerate(candidates):
        candidate_name = str(candidate["name"])
        candidate_run_root = batch_dir / "runs" / candidate_name
        launch_args = _launch_args(candidate.get("launch_args", {}))
        for seed_index, seed in enumerate(seeds):
            ros_domain_id = args.base_ros_domain_id + candidate_index * max(1, len(seeds)) + seed_index
            env = os.environ.copy()
            env["ROS_DOMAIN_ID"] = str(ros_domain_id)
            command = [
                "make",
                "navigate-inner",
                f"SEED={seed}",
                f"CELL_SIZE_M={args.cell_size_m:g}",
                f"NAVIGATE_DURATION={args.duration:g}",
                f"CONFIG={args.config}",
                f"RUN_ROOT={candidate_run_root}",
                "NAVIGATE_SKIP_BUILD=true",
                "NAVIGATE_DASHBOARD=false",
                "DASHBOARD_AUTO_OPEN=false",
                f"ROS_DOMAIN_ID={ros_domain_id}",
                f"NAVIGATE_LAUNCH_ARGS={launch_args}",
            ]
            print(f"{candidate_name} seed {seed}: start ROS_DOMAIN_ID={ros_domain_id}", flush=True)
            log_path = logs_dir / candidate_name / f"seed-{seed}.log"
            returncode, elapsed = _run_logged(command, log_path=log_path, env=env)
            print(f"{candidate_name} seed {seed}: finished rc={returncode} elapsed_s={elapsed:.1f}", flush=True)
            summary_path = _latest_summary(candidate_run_root, seed)
            summary = json.loads(summary_path.read_text(encoding="utf-8")) if summary_path else None
            row = {
                "candidate": candidate_name,
                "seed": seed,
                "returncode": returncode,
                "elapsed_s": elapsed,
                "final_status": summary.get("final_status") if summary else None,
                "summary_path": str(summary_path) if summary_path else "",
                "log_path": str(log_path),
                "launch_args": launch_args,
            }
            row.update(_score(summary))
            run_rows.append(row)

    candidate_rows = _candidate_summary(run_rows)
    valid_candidates = [
        row for row in candidate_rows
        if _numeric(row.get("median_score")) is not None
    ]
    best = min(valid_candidates, key=lambda row: float(row["median_score"])) if valid_candidates else None
    report = {
        "schema_version": 1,
        "created_at": datetime.now().astimezone().isoformat(),
        "batch_dir": str(batch_dir),
        "seeds": seeds,
        "cell_size_m": args.cell_size_m,
        "duration_s": args.duration,
        "ground_truth_usage": "offline_scoring_only",
        "best_candidate": best,
        "candidates": candidate_rows,
        "runs": run_rows,
    }
    (batch_dir / "odometry_tuning_summary.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _write_csv(run_rows, batch_dir / "odometry_tuning_runs.csv")
    print(json.dumps({"best_candidate": best, "summary": str(batch_dir / "odometry_tuning_summary.json")}, indent=2), flush=True)

    if args.fail_on_run_error and any(row["returncode"] != 0 for row in run_rows):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
