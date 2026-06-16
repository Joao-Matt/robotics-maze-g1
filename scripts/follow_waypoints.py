"""Run the Milestone 5 waypoint follower in point-robot debug mode."""

from __future__ import annotations

from pathlib import Path
import argparse
import json
import math
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from maze.generator import generate_maze_from_config
from maze.visualization import save_svg
from nav.controller import Pose2D, WaypointFollower, WaypointFollowerConfig, integrate_point_robot
from nav.planner import PlannerError, plan_oracle_path
from sim.config import ConfigError, load_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Follow oracle waypoints with a simple point robot.")
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
        help="Directory for visible follow artifacts.",
    )
    parser.add_argument("--cell-px", type=int, default=48, help="Cell size in pixels for SVG output.")
    parser.add_argument("--max-time", type=float, default=None, help="Maximum simulated point-robot time.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        config = load_config(args.config)
        maze = generate_maze_from_config(config, args.seed)
        plan = plan_oracle_path(maze, safety_radius_m=float(config["robot"]["safety_radius_m"]))
        result = simulate_point_robot_follow(config, plan.waypoints_xyz, max_time_s=args.max_time)
        args.output_dir.mkdir(parents=True, exist_ok=True)
        svg_path = args.output_dir / f"follow_seed-{args.seed}_point.svg"
        json_path = args.output_dir / f"follow_seed-{args.seed}_point.json"
        _save_follow_svg(maze, plan.path_cells, result["trajectory"], svg_path, cell_px=args.cell_px)
        artifact = {
            "mode": "point_robot_debug",
            "planner_mode": plan.mode,
            "seed": args.seed,
            "note": "Point-robot follower validates waypoint-control math; it is not G1 walking.",
            "plan": plan.to_dict(),
            "follow": result,
        }
        json_path.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    except (ConfigError, KeyError, ValueError, PlannerError) as exc:
        print(f"Waypoint following failed: {exc}", file=sys.stderr)
        return 1

    print(f"seed: {args.seed}")
    print("mode: point_robot_debug")
    print(json.dumps(result["summary"], indent=2, sort_keys=True))
    print(f"follow_svg_artifact: {svg_path}")
    print(f"follow_json_artifact: {json_path}")
    return 0


def simulate_point_robot_follow(
    config: dict,
    waypoints: list[tuple[float, float, float]],
    *,
    max_time_s: float | None = None,
) -> dict:
    robot_config = config["robot"]
    sim_config = config["sim"]
    dt = float(sim_config["control_dt"])
    follower = WaypointFollower(
        WaypointFollowerConfig(
            max_forward_speed_mps=float(robot_config["max_forward_speed_mps"]),
            max_yaw_rate_radps=float(robot_config["max_yaw_rate_radps"]),
            goal_tolerance_m=float(robot_config["goal_tolerance_m"]),
            waypoint_tolerance_m=min(0.25, float(robot_config["goal_tolerance_m"]) * 0.5),
        )
    )

    if not waypoints:
        raise ValueError("Cannot follow an empty waypoint list.")

    pose = Pose2D(float(waypoints[0][0]), float(waypoints[0][1]), _initial_yaw(waypoints))
    path_length = _path_length(waypoints)
    nominal_time = path_length / max(float(robot_config["max_forward_speed_mps"]), 1e-6)
    if max_time_s is None:
        max_time_s = max(float(sim_config["max_episode_time_s"]), nominal_time * 1.8)
    max_steps = max(1, math.ceil(max_time_s / dt))

    waypoint_index = 0
    trajectory: list[dict] = []
    command_counts: dict[str, int] = {}
    final_status = "timeout"
    output = None

    for step in range(max_steps + 1):
        output = follower.compute_command(pose, waypoints, waypoint_index)
        waypoint_index = output.waypoint_index
        command_counts[output.status] = command_counts.get(output.status, 0) + 1
        if step % 10 == 0 or output.status == "goal_reached":
            trajectory.append(
                {
                    "time_s": round(step * dt, 6),
                    "x": pose.x,
                    "y": pose.y,
                    "yaw": pose.yaw,
                    "status": output.status,
                    "waypoint_index": output.waypoint_index,
                }
            )
        if output.status == "goal_reached":
            final_status = "goal_reached"
            break
        pose = integrate_point_robot(pose, output.command, dt)

    elapsed = (sum(command_counts.values()) - 1) * dt
    summary = {
        "status": final_status,
        "elapsed_s": elapsed,
        "steps": sum(command_counts.values()) - 1,
        "waypoint_count": len(waypoints),
        "final_waypoint_index": waypoint_index,
        "final_pose": {"x": pose.x, "y": pose.y, "yaw": pose.yaw},
        "command_status_counts": command_counts,
        "max_time_s": float(max_time_s),
    }
    return {"summary": summary, "trajectory": trajectory, "last_controller_output": output.to_dict() if output else None}


def _save_follow_svg(maze, route, trajectory: list[dict], path: Path, *, cell_px: int) -> None:
    save_svg(maze, path, route, cell_px=cell_px)
    if not trajectory:
        return
    content = path.read_text(encoding="utf-8")
    points = " ".join(_world_to_svg_point(maze, item["x"], item["y"], cell_px) for item in trajectory)
    end = trajectory[-1]
    end_x, end_y = _world_to_svg_point(maze, end["x"], end["y"], cell_px).split(",")
    overlay = (
        f'<polyline points="{points}" fill="none" stroke="#f97316" stroke-width="{max(3, cell_px // 10)}" '
        'stroke-linecap="round" stroke-linejoin="round"/>\n'
        f'<circle cx="{end_x}" cy="{end_y}" r="{max(4, cell_px // 7)}" fill="#f97316" stroke="#7c2d12" stroke-width="2"/>\n'
    )
    path.write_text(content.replace("</svg>", overlay + "</svg>"), encoding="utf-8")


def _world_to_svg_point(maze, x: float, y: float, cell_px: int) -> str:
    col = x / maze.spec.cell_size_m + (maze.spec.width_cells - 1) / 2.0
    row = (maze.spec.height_cells - 1) / 2.0 - y / maze.spec.cell_size_m
    return f"{(col + 0.5) * cell_px:.3f},{(row + 0.5) * cell_px:.3f}"


def _initial_yaw(waypoints: list[tuple[float, float, float]]) -> float:
    if len(waypoints) < 2:
        return 0.0
    return math.atan2(waypoints[1][1] - waypoints[0][1], waypoints[1][0] - waypoints[0][0])


def _path_length(waypoints: list[tuple[float, float, float]]) -> float:
    return sum(
        math.hypot(waypoints[index][0] - waypoints[index - 1][0], waypoints[index][1] - waypoints[index - 1][1])
        for index in range(1, len(waypoints))
    )


if __name__ == "__main__":
    raise SystemExit(main())
