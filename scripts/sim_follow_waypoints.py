"""Visual MuJoCo waypoint-follow inspection with a moving proxy body."""

from __future__ import annotations

from pathlib import Path
import argparse
import csv
import html
import json
import math
import os
import sys
import time
from datetime import datetime, timezone

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from maze.generator import generate_maze_from_config
from maze.visualization import save_svg
from nav.controller import Pose2D, WaypointFollower, WaypointFollowerConfig, integrate_point_robot
from nav.planner import PlannerError, plan_oracle_path
from sim.config import ConfigError, load_config
from sim.mujoco_runner import MuJoCoRunnerError, import_mujoco
from sim.proxy_robot import add_proxy_to_world_xml, set_proxy_pose
from sim.world_builder import build_maze_world
from sim.mujoco_runner import _write_png


FINAL_STATUSES = {"GOAL_REACHED", "TIMEOUT", "STUCK", "FAILED"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run visible MuJoCo proxy waypoint following.")
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
        help="Directory for generated visual artifacts.",
    )
    parser.add_argument("--viewer", action="store_true", help="Open the live MuJoCo viewer.")
    parser.add_argument("--time-scale", type=float, default=32.0, help="Viewer playback speed multiplier.")
    parser.add_argument("--viewer-frame-stride", type=int, default=32, help="Controller ticks per viewer sync.")
    parser.add_argument("--hold-s", type=float, default=2.0, help="Seconds to keep viewer open after completion.")
    parser.add_argument("--cell-px", type=int, default=48, help="Cell size in pixels for SVG outputs.")
    parser.add_argument("--render-width", type=int, default=640, help="Final render width.")
    parser.add_argument("--render-height", type=int, default=480, help="Final render height.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    paths = _artifact_paths(args.output_dir, args.seed)
    summary = {
        "seed": args.seed,
        "mode": "proxy_waypoint_follow",
        "g1_locomotion_implemented": False,
        "final_status": "FAILED",
        "artifacts": {name: str(path) for name, path in paths.items()},
        "rerun_live_command": f"make sim-follow SEED={args.seed}",
        "rerun_headless_command": f"make view-sim-follow SEED={args.seed}",
    }

    try:
        config = load_config(args.config)
        maze = generate_maze_from_config(config, args.seed)
        plan = plan_oracle_path(maze, safety_radius_m=float(config["robot"]["safety_radius_m"]))
        world = build_maze_world(config, args.seed, args.output_dir)
        add_proxy_to_world_xml(
            Path(world.model_xml_path),
            paths["world_xml"],
            start_xyz=plan.waypoints_xyz[0],
            path_waypoints=plan.waypoints_xyz,
        )
        save_svg(maze, paths["topdown_svg"], None, cell_px=args.cell_px)
        save_svg(maze, paths["path_svg"], plan.path_cells, cell_px=args.cell_px)

        sim_result = run_proxy_simulation(
            config,
            paths["world_xml"],
            plan.waypoints_xyz,
            paths["trajectory_csv"],
            final_render_path=paths["final_render"],
            viewer=args.viewer,
            time_scale=args.time_scale,
            viewer_frame_stride=args.viewer_frame_stride,
            hold_s=args.hold_s,
            render_width=args.render_width,
            render_height=args.render_height,
        )
        summary.update(sim_result)
        summary["final_status"] = sim_result["final_status"]
        summary["waypoint_count"] = len(plan.waypoints_xyz)
        summary["waypoints_reached"] = sim_result["waypoints_reached"]
        summary["generated_world_xml_path"] = str(paths["world_xml"])
        summary["planned_path_svg_path"] = str(paths["path_svg"])
        summary["topdown_svg_path"] = str(paths["topdown_svg"])
        summary["trajectory_csv_path"] = str(paths["trajectory_csv"])
        summary["final_render_path"] = str(paths["final_render"])
        summary["note"] = "Moving proxy validates waypoint following in MuJoCo; G1 remains a standing reference."
    except (ConfigError, KeyError, ValueError, PlannerError, MuJoCoRunnerError, RuntimeError) as exc:
        summary["error"] = str(exc)
        paths["summary_json"].write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        write_dashboard(paths, summary)
        print(f"Sim-follow failed: {exc}", file=sys.stderr)
        print_artifacts(paths)
        return 1

    paths["summary_json"].write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_dashboard(paths, summary)
    print(json.dumps({key: summary[key] for key in ("seed", "mode", "final_status", "waypoints_reached", "duration_s")}, indent=2))
    print_artifacts(paths)
    if summary["final_status"] not in FINAL_STATUSES:
        return 1
    return 0 if summary["final_status"] == "GOAL_REACHED" else 1


def run_proxy_simulation(
    config: dict,
    world_xml_path: Path,
    waypoints: list[tuple[float, float, float]],
    trajectory_csv_path: Path,
    *,
    final_render_path: Path,
    viewer: bool,
    time_scale: float,
    viewer_frame_stride: int,
    hold_s: float,
    render_width: int,
    render_height: int,
) -> dict:
    mujoco = import_mujoco()
    try:
        model = mujoco.MjModel.from_xml_path(str(world_xml_path))
    except Exception as exc:
        raise RuntimeError(f"Failed to load proxy MuJoCo world: {world_xml_path}: {exc}") from exc

    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, config["robot"].get("initial_keyframe", "stand"))
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)

    follower = WaypointFollower(
        WaypointFollowerConfig(
            max_forward_speed_mps=float(config["robot"]["max_forward_speed_mps"]),
            max_yaw_rate_radps=float(config["robot"]["max_yaw_rate_radps"]),
            goal_tolerance_m=float(config["robot"]["goal_tolerance_m"]),
            waypoint_tolerance_m=min(0.25, float(config["robot"]["goal_tolerance_m"]) * 0.5),
        )
    )
    dt = float(config["sim"]["control_dt"])
    max_time_s = max(float(config["sim"]["max_episode_time_s"]), _path_length(waypoints) / max(float(config["robot"]["max_forward_speed_mps"]), 1e-6) * 1.8)
    max_steps = int(max_time_s / dt)
    pose = Pose2D(float(waypoints[0][0]), float(waypoints[0][1]), _initial_yaw(waypoints))
    waypoint_index = 0
    started = datetime.now(timezone.utc)
    rows: list[dict] = []
    final_status = "TIMEOUT"
    last_output = None
    viewer_closed_before_completion = False
    frame_stride = max(1, int(viewer_frame_stride))

    viewer_context = _launch_viewer(mujoco, model, data) if viewer else None
    try:
        active_viewer = viewer_context.__enter__() if viewer_context else None
        for step in range(max_steps + 1):
            last_output = follower.compute_command(pose, waypoints, waypoint_index)
            waypoint_index = last_output.waypoint_index
            set_proxy_pose(mujoco, model, data, pose)
            data.time = step * dt
            mujoco.mj_forward(model, data)
            rows.append(
                {
                    "time_s": f"{step * dt:.6f}",
                    "x": f"{pose.x:.9f}",
                    "y": f"{pose.y:.9f}",
                    "yaw": f"{pose.yaw:.9f}",
                    "vx": f"{last_output.command.vx:.9f}",
                    "vy": f"{last_output.command.vy:.9f}",
                    "wz": f"{last_output.command.wz:.9f}",
                    "status": last_output.status.upper(),
                    "waypoint_index": str(last_output.waypoint_index),
                }
            )
            if last_output.status == "goal_reached":
                final_status = "GOAL_REACHED"
                if active_viewer is not None and active_viewer.is_running():
                    active_viewer.sync()
                break
            if active_viewer is not None and step % frame_stride == 0:
                if active_viewer.is_running():
                    active_viewer.sync()
                    if time_scale > 0:
                        time.sleep(min((dt * frame_stride) / time_scale, 0.03))
                else:
                    viewer_closed_before_completion = True
                    active_viewer = None
            pose = integrate_point_robot(pose, last_output.command, dt)

        if active_viewer is not None and hold_s > 0:
            end = time.time() + hold_s
            while time.time() < end and active_viewer.is_running():
                set_proxy_pose(mujoco, model, data, pose)
                mujoco.mj_forward(model, data)
                active_viewer.sync()
                time.sleep(0.05)
    finally:
        if viewer_context is not None:
            viewer_context.__exit__(None, None, None)

    set_proxy_pose(mujoco, model, data, pose)
    mujoco.mj_forward(model, data)
    _save_render(mujoco, model, data, final_render_path, render_width, render_height)
    _write_trajectory_csv(trajectory_csv_path, rows)

    duration_s = float(rows[-1]["time_s"]) if rows else 0.0
    return {
        "final_status": final_status,
        "duration_s": duration_s,
        "wall_clock_started": started.isoformat(timespec="seconds"),
        "trajectory_samples": len(rows),
        "waypoints_reached": waypoint_index + 1,
        "final_pose": {"x": pose.x, "y": pose.y, "yaw": pose.yaw},
        "last_controller_output": last_output.to_dict() if last_output else None,
        "viewer_opened": bool(viewer),
        "viewer_closed_before_completion": viewer_closed_before_completion,
        "viewer_frame_stride": frame_stride,
    }


def write_dashboard(paths: dict[str, Path], summary: dict) -> None:
    generated = datetime.now(timezone.utc).isoformat(timespec="seconds")
    base = paths["dashboard"].parent
    status = html.escape(str(summary.get("final_status", "FAILED")))
    links = "\n".join(
        f'<a href="{html.escape(_rel(path, base))}">{html.escape(label)}</a>'
        for label, path in (
            ("Generated world XML", paths["world_xml"]),
            ("Top-down maze", paths["topdown_svg"]),
            ("Planned path", paths["path_svg"]),
            ("Final MuJoCo render", paths["final_render"]),
            ("Summary JSON", paths["summary_json"]),
            ("Trajectory CSV", paths["trajectory_csv"]),
        )
    )
    document = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Sim Follow Seed {summary.get('seed')}</title>
  <style>
    body {{ margin: 0; font-family: system-ui, sans-serif; color: #111827; background: #f3f4f6; }}
    header {{ padding: 14px 18px; color: #f9fafb; background: #111827; }}
    h1 {{ margin: 0; font-size: 18px; }}
    .meta {{ margin-top: 4px; color: #cbd5e1; font-size: 13px; }}
    main {{ display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 12px; padding: 12px; }}
    section {{ background: #ffffff; border: 1px solid #d1d5db; border-radius: 6px; min-width: 0; }}
    h2 {{ margin: 0; padding: 10px 12px; font-size: 15px; border-bottom: 1px solid #e5e7eb; }}
    iframe, img {{ width: 100%; height: 68vh; border: 0; object-fit: contain; background: #e5e7eb; }}
    .summary {{ padding: 12px; display: grid; gap: 8px; font-size: 14px; }}
    .summary code {{ font-family: ui-monospace, SFMono-Regular, Consolas, monospace; }}
    footer {{ display: flex; flex-wrap: wrap; gap: 10px; padding: 10px 12px 14px; background: #f9fafb; border-top: 1px solid #d1d5db; }}
    footer a {{ color: #1d4ed8; font-size: 13px; }}
    @media (max-width: 1100px) {{ main {{ grid-template-columns: 1fr; }} iframe, img {{ height: 60vh; }} }}
  </style>
</head>
<body>
  <header>
    <h1>MuJoCo Proxy Waypoint Follow</h1>
    <div class="meta">seed={summary.get('seed')} · mode={html.escape(str(summary.get('mode')))} · generated={html.escape(generated)}</div>
  </header>
  <main>
    <section><h2>Top-Down Maze</h2><iframe src="{html.escape(_rel(paths['topdown_svg'], base))}"></iframe></section>
    <section><h2>Planned Path</h2><iframe src="{html.escape(_rel(paths['path_svg'], base))}"></iframe></section>
    <section><h2>MuJoCo Final Render</h2><img src="{html.escape(_rel(paths['final_render'], base))}" alt="MuJoCo final render"></section>
    <section class="summary">
      <h2>Run Summary</h2>
      <div>Status: <code>{status}</code></div>
      <div>G1 locomotion implemented: <code>{str(summary.get('g1_locomotion_implemented')).lower()}</code></div>
      <div>Waypoints reached: <code>{summary.get('waypoints_reached')}</code></div>
      <div>Duration: <code>{summary.get('duration_s')}</code> simulated seconds</div>
      <div>Trajectory: <code>{html.escape(str(paths['trajectory_csv']))}</code></div>
      <div>Rerun live: <code>{html.escape(str(summary.get('rerun_live_command')))}</code></div>
    </section>
  </main>
  <footer>{links}</footer>
</body>
</html>
"""
    paths["dashboard"].write_text(document, encoding="utf-8")


def print_artifacts(paths: dict[str, Path]) -> None:
    for label, path in paths.items():
        print(f"sim_follow_{label}_artifact: {path}")


def _artifact_paths(output_dir: Path, seed: int) -> dict[str, Path]:
    return {
        "world_xml": output_dir / f"sim_follow_seed-{seed}_world.xml",
        "topdown_svg": output_dir / f"sim_follow_seed-{seed}_topdown.svg",
        "path_svg": output_dir / f"sim_follow_seed-{seed}_path.svg",
        "final_render": output_dir / f"sim_follow_seed-{seed}_final.png",
        "dashboard": output_dir / f"sim_follow_seed-{seed}_dashboard.html",
        "summary_json": output_dir / f"sim_follow_seed-{seed}_summary.json",
        "trajectory_csv": output_dir / f"sim_follow_seed-{seed}_trajectory.csv",
    }


def _launch_viewer(mujoco, model, data):
    try:
        import mujoco.viewer
    except Exception as exc:
        raise RuntimeError(f"Live MuJoCo viewer is unavailable: {exc}") from exc
    return mujoco.viewer.launch_passive(model, data)


def _save_render(mujoco, model, data, path: Path, width: int, height: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    renderer = mujoco.Renderer(model, width=width, height=height)
    try:
        renderer.update_scene(data)
        pixels = renderer.render()
    finally:
        close = getattr(renderer, "close", None)
        if close:
            close()
    _write_png(path, pixels)


def _write_trajectory_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=["time_s", "x", "y", "yaw", "vx", "vy", "wz", "status", "waypoint_index"])
        writer.writeheader()
        writer.writerows(rows)


def _rel(path: Path, base: Path) -> str:
    return os.path.relpath(path, start=base)


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
