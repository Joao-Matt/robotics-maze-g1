#!/usr/bin/env python3
"""Run a direct MuJoCo command-sweep calibration for the G1 locomotion policy."""

from __future__ import annotations

from pathlib import Path
import argparse
import json
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sim.locomotion_calibration import G1LocomotionCalibrationRunner, load_calibration_suite  # noqa: E402
from sim.run_context import allocate_run, finalize_manifest, write_manifest  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs" / "default.yaml")
    parser.add_argument(
        "--calibration-config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "g1_locomotion_calibration.yaml",
    )
    parser.add_argument("--profile", choices=("balanced", "smoke"), default=None)
    parser.add_argument("--run-root", type=Path, default=PROJECT_ROOT / "runs" / "calibration")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--unitree-rl-gym-repo", type=Path, default=PROJECT_ROOT / "third_party" / "unitree_rl_gym")
    parser.add_argument("--friction-scale", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    suite = load_calibration_suite(args.calibration_config, args.profile)
    profile = suite.profile
    if args.output_dir is not None:
        run_dir = args.output_dir
        run_dir.mkdir(parents=True, exist_ok=True)
    else:
        run_dir = allocate_run(args.run_root, "g1_locomotion", args.seed, {"profile": profile})
    write_manifest(
        run_dir,
        command="g1_locomotion",
        seed=args.seed,
        parameters={
            "profile": profile,
            "calibration_config": str(args.calibration_config),
            "friction_scale": args.friction_scale,
        },
        project_root=PROJECT_ROOT,
        config_path=args.config,
    )
    runner = G1LocomotionCalibrationRunner(
        project_config_path=args.config,
        calibration_config_path=args.calibration_config,
        suite=suite,
        run_dir=run_dir,
        seed=args.seed,
        unitree_rl_gym_repo=args.unitree_rl_gym_repo,
        friction_scale=args.friction_scale,
    )
    try:
        summary = runner.run()
    except Exception as exc:
        failure = {"status": "failed", "error": str(exc), "run_directory": str(run_dir)}
        (run_dir / "summary.json").write_text(json.dumps(failure, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        finalize_manifest(run_dir, "failed", run_dir / "summary.json")
        print(json.dumps(failure, indent=2, sort_keys=True), file=sys.stderr)
        return 1

    finalize_manifest(run_dir, str(summary.get("status", "passed")), run_dir / "summary.json")
    print(json.dumps({
        "status": summary.get("status"),
        "run_dir": str(run_dir),
        "locomotion_calibration": str(run_dir / "locomotion_calibration.json"),
        "summary": str(run_dir / "summary.json"),
        "csv": str(run_dir / "command_results.csv"),
        "report": str(run_dir / "report.md"),
        "dashboard": str(run_dir / "dashboard.html"),
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
