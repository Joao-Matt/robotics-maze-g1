"""Generate Milestone 4 oracle-planner artifacts without robot execution."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from maze.generator import generate_maze_from_config
from maze.visualization import save_ascii, save_svg
from nav.planner import PlanningError, plan_oracle_path
from sim.config import ConfigError, load_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Milestone 4 oracle planner (no robot control).")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--label", default="", help="Optional artifact label, such as 'wide'.")
    parser.add_argument("--corridor-width-m", type=float, default=None)
    parser.add_argument("--cell-px", type=int, default=48)
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs" / "default.yaml")
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "runs" / "visual")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    label = f"_{args.label.strip()}" if args.label.strip() else ""
    prefix = f"milestone_4{label}_seed-{args.seed}"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    path_svg = args.output_dir / f"{prefix}_path.svg"
    path_ascii = args.output_dir / f"{prefix}_path.txt"
    summary_json = args.output_dir / f"{prefix}_planner_summary.json"

    try:
        config = load_config(args.config)
        corridor_width_m = _apply_corridor_width(config, args.corridor_width_m)
        maze = generate_maze_from_config(config, args.seed)
        plan = plan_oracle_path(
            maze,
            safety_radius_m=float(config["robot"]["safety_radius_m"]),
            simplify=False,
            planner=str(config.get("oracle", {}).get("planner", "bfs")),
            turn_penalty_cost=float(config.get("oracle", {}).get("turn_penalty_cost", 0.0)),
        )
    except (ConfigError, KeyError, ValueError, PlanningError) as exc:
        print(f"Milestone 4 planner failed: {exc}", file=sys.stderr)
        return 1

    save_svg(maze, path_svg, plan.cells, cell_px=args.cell_px)
    save_ascii(maze, path_ascii, plan.cells)
    summary = {
        "milestone": 4,
        "mode": "oracle_planner_only",
        "robot_execution": False,
        "seed": args.seed,
        "planner": str(config.get("oracle", {}).get("planner", "bfs")),
        "corridor_width_m": corridor_width_m,
        "path_cell_count": len(plan.cells),
        "waypoint_count": len(plan.waypoints),
        "start_cell": list(maze.spec.start_cell),
        "goal_cell": list(maze.spec.goal_cell),
        "artifacts": {"path_svg": str(path_svg), "path_ascii": str(path_ascii)},
        "next_command": f"make milestone_5 SEED={args.seed}",
    }
    summary_json.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print("Milestone 4 oracle planner passed (planner only; robot execution is Milestone 5).")
    print(f"path_cells: {len(plan.cells)}")
    print(f"waypoints: {len(plan.waypoints)}")
    print(f"path_svg: {path_svg}")
    print(f"path_ascii: {path_ascii}")
    print(f"summary_json: {summary_json}")
    return 0


def _apply_corridor_width(config: dict, override: float | None) -> float:
    if override is None:
        return float(config["maze"]["cell_size_m"])
    if not 1.0 <= override <= 2.0:
        raise ValueError(f"corridor width must be between 1.0 and 2.0 meters, got {override:.3g}")
    config["maze"]["cell_size_m"] = float(override)
    return float(override)


if __name__ == "__main__":
    raise SystemExit(main())
