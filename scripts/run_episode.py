"""Run a single Milestone 1 MuJoCo bring-up episode."""

from __future__ import annotations

from pathlib import Path
import argparse
import json
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sim.config import ConfigError, load_config
from sim.mujoco_runner import MuJoCoRunner, MuJoCoRunnerError


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a Unitree G1 MuJoCo bring-up episode.")
    parser.add_argument("--seed", type=int, default=1, help="Accepted for reproducibility; unused in Milestone 1.")
    parser.add_argument("--duration", type=float, default=None, help="Simulation duration in seconds.")
    parser.add_argument("--viewer", action="store_true", help="Launch MuJoCo passive viewer if available.")
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "default.yaml",
        help="Path to YAML config.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        config = load_config(args.config)
        summary = MuJoCoRunner(config).run(duration_s=args.duration, viewer=args.viewer)
    except (ConfigError, MuJoCoRunnerError) as exc:
        print(f"Episode failed: {exc}", file=sys.stderr)
        return 1

    print(f"seed: {args.seed}")
    print("mode: bringup")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
