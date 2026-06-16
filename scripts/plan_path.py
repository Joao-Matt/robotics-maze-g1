"""Plan an oracle/debug path through a generated maze."""

from __future__ import annotations

from pathlib import Path
import argparse
import json
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from maze.generator import generate_maze_from_config
from nav.planner import PlannerError, plan_oracle_path, save_plan_artifacts
from sim.config import ConfigError, load_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plan an oracle path on a generated maze.")
    parser.add_argument("--seed", type=int, default=1, help="Integer maze seed.")
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "default.yaml",
        help="Path to YAML config.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "runs" / "visual",
        help="Directory for visible plan artifacts.",
    )
    parser.add_argument("--cell-px", type=int, default=48, help="Cell size in pixels for SVG output.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        config = load_config(args.config)
        maze = generate_maze_from_config(config, args.seed)
        plan = plan_oracle_path(
            maze,
            safety_radius_m=float(config["robot"]["safety_radius_m"]),
        )
        svg_path, json_path = save_plan_artifacts(maze, plan, output_dir=args.output_dir, cell_px=args.cell_px)
    except (ConfigError, KeyError, ValueError, PlannerError) as exc:
        print(f"Oracle planning failed: {exc}", file=sys.stderr)
        return 1

    print(f"seed: {args.seed}")
    print("mode: oracle")
    print(json.dumps(plan.to_dict(), indent=2, sort_keys=True))
    print(f"plan_svg_artifact: {svg_path}")
    print(f"plan_json_artifact: {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
