#!/usr/bin/env python3
"""Run headless navigation over a fixed seed batch and aggregate results."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import subprocess
import time
from typing import Any

from aggregate_navigation_seeds import (
    DEFAULT_HELDOUT_SEEDS,
    parse_seed_values,
    print_report,
    summarize,
    write_report_files,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _stamp() -> str:
    return datetime.now().astimezone().strftime("%Y%m%dT%H%M%S.%f")[:-3]


def _format_number(value: float) -> str:
    return f"{value:g}"


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


def _run_prebuild(batch_dir: Path, env: dict[str, str]) -> int:
    command = ["make", "prebuild-inner"]
    returncode, elapsed = _run_logged(command, log_path=batch_dir / "logs" / "prebuild.log", env=env)
    print(f"prebuild-inner finished rc={returncode} elapsed_s={elapsed:.1f}", flush=True)
    return returncode


def _run_seed(index: int, seed: int, args: argparse.Namespace, batch_dir: Path) -> dict[str, Any]:
    ros_domain_id = args.base_ros_domain_id + index
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(ros_domain_id)
    command = [
        "make",
        "navigate-inner",
        f"SEED={seed}",
        f"CELL_SIZE_M={_format_number(args.cell_size_m)}",
        f"NAVIGATE_DURATION={_format_number(args.duration)}",
        f"RUN_ROOT={args.run_root}",
        f"CONFIG={args.config}",
        "NAVIGATE_SKIP_BUILD=true",
        "NAVIGATE_DASHBOARD=false",
        f"ROS_DOMAIN_ID={ros_domain_id}",
    ]
    log_path = batch_dir / "logs" / f"seed-{seed}.log"
    print(f"seed {seed}: start ROS_DOMAIN_ID={ros_domain_id}", flush=True)
    returncode, elapsed = _run_logged(command, log_path=log_path, env=env)
    print(f"seed {seed}: finished rc={returncode} elapsed_s={elapsed:.1f}", flush=True)
    return {
        "index": index,
        "seed": seed,
        "returncode": returncode,
        "elapsed_s": elapsed,
        "ros_domain_id": ros_domain_id,
        "log_path": str(log_path),
    }


def _write_batch_manifest(batch_dir: Path, args: argparse.Namespace, seeds: list[int]) -> None:
    values = {
        "schema_version": 1,
        "created_at": datetime.now().astimezone().isoformat(),
        "label": args.label,
        "seeds": seeds,
        "run_root": str(args.run_root),
        "cell_size_m": args.cell_size_m,
        "duration_s": args.duration,
        "config": str(args.config),
        "jobs": args.jobs,
        "base_ros_domain_id": args.base_ros_domain_id,
    }
    batch_dir.mkdir(parents=True, exist_ok=True)
    (batch_dir / "batch_manifest.json").write_text(json.dumps(values, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, default=Path("runs/heldout-20"))
    parser.add_argument("--label", default="held_out")
    parser.add_argument("--seeds", nargs="*", default=[])
    parser.add_argument("--cell-size-m", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--base-ros-domain-id", type=int, default=40)
    parser.add_argument("--skip-prebuild", action="store_true")
    parser.add_argument("--fail-on-run-error", action="store_true")
    args = parser.parse_args()

    if os.environ.get("ROS_DISTRO") != "humble":
        print("run_navigation_seed_batch.py must run inside a ROS 2 Humble environment.", flush=True)
        return 2
    if args.jobs < 1:
        print("--jobs must be >= 1", flush=True)
        return 2

    seeds = parse_seed_values(args.seeds) or DEFAULT_HELDOUT_SEEDS
    batch_dir = args.run_root / "batches" / f"{_stamp()}__navigate"
    _write_batch_manifest(batch_dir, args, seeds)

    env = os.environ.copy()
    if not args.skip_prebuild:
        prebuild_status = _run_prebuild(batch_dir, env)
        if prebuild_status != 0:
            return prebuild_status

    run_results: list[dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=args.jobs) as executor:
        futures = {
            executor.submit(_run_seed, index, seed, args, batch_dir): seed
            for index, seed in enumerate(seeds)
        }
        for future in as_completed(futures):
            run_results.append(future.result())

    report = summarize(args.run_root / "navigate", label=args.label, seeds=seeds)
    by_seed = {row["seed"]: row for row in run_results}
    for row in report["results"]:
        row.update(by_seed.get(row["seed"], {}))
    report["batch"] = {
        "batch_dir": str(batch_dir),
        "jobs": args.jobs,
        "run_command_count": len(run_results),
        "nonzero_returncode_count": sum(1 for row in run_results if row["returncode"] != 0),
    }
    report["run_commands"] = sorted(run_results, key=lambda row: row["index"])
    write_report_files(report, batch_dir / "summary.json", batch_dir / "summary.csv", batch_dir / "summary.html")
    print_report(report)
    print(f"batch_report: {batch_dir / 'summary.json'}", flush=True)
    print(f"batch_html_report: {batch_dir / 'summary.html'}", flush=True)

    if args.fail_on_run_error and any(row["returncode"] != 0 for row in run_results):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
