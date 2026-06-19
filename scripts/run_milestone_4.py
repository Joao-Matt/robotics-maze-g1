"""Legacy oracle waypoint walker; robot execution is exposed as Milestone 5."""

from __future__ import annotations

from pathlib import Path
import argparse
import csv
import json
import math
import sys
import time
import xml.etree.ElementTree as ET

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from maze.generator import generate_maze_from_config
from maze.visualization import save_svg
from nav.controller import GOAL_REACHED, Pose2D, WaypointFollower, WaypointFollowerConfig, pose_from_base_state
from nav.planner import PlanningError, plan_oracle_path, simplify_cell_path, cells_to_waypoints
from sim.config import ConfigError, load_config
from sim.locomotion_policy_adapter import LocomotionPolicyError, VelocityCommand, create_policy_adapter
from sim.locomotion_sandbox import base_state, config_from_dict, determine_status, save_render
from sim.mujoco_runner import MuJoCoImportError, MuJoCoModelError, import_mujoco
from sim.world_builder import WorldBuildResult, build_maze_world, cell_to_world_xy


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Milestone 4 Lucky G1 oracle waypoint walking.")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--label", default="", help="Optional artifact label, such as 'wide'.")
    parser.add_argument(
        "--corridor-width-m",
        type=float,
        default=None,
        help="Override maze.cell_size_m for this run. Valid range: 1.0 to 2.0 meters.",
    )
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs" / "default.yaml")
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "runs" / "visual")
    parser.add_argument(
        "--lucky-g1-repo",
        type=Path,
        default=PROJECT_ROOT / "third_party" / "g1-manipulation-challenge",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    label = f"_{args.label.strip()}" if args.label.strip() else ""
    prefix = f"milestone_4{label}_seed-{args.seed}"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    artifacts = {
        "world_xml": args.output_dir / f"{prefix}_world.xml",
        "topdown_svg": args.output_dir / f"{prefix}_topdown.svg",
        "path_svg": args.output_dir / f"{prefix}_path.svg",
        "trajectory_csv": args.output_dir / f"{prefix}_trajectory.csv",
        "summary_json": args.output_dir / f"{prefix}_summary.json",
        "dashboard_html": args.output_dir / f"{prefix}_dashboard.html",
        "final_render": args.output_dir / f"{prefix}_final.png",
        "compatibility_json": args.output_dir / f"{prefix}_policy_compatibility.json",
    }
    started = time.time()
    summary: dict[str, object] = {
        "mode": "milestone_4_lucky_oracle_walking",
        "seed": args.seed,
        "policy": "lucky_walker",
        "final_status": "ERROR",
        "g1_locomotion_implemented": True,
        "proxy_body_used": False,
        "viewer_requested": bool(args.viewer),
        "viewer_opened": False,
        "contact_summary": {},
        "error": None,
        "rerun_live_command": f"make milestone_4 SEED={args.seed}",
        "rerun_headless_command": f"make view-milestone_4 SEED={args.seed}",
    }

    try:
        config = load_config(args.config)
        corridor_width_m = _apply_corridor_width_override(config, args.corridor_width_m)
        maze = generate_maze_from_config(config, args.seed)
        dense_plan = plan_oracle_path(maze, safety_radius_m=float(config["robot"]["safety_radius_m"]), simplify=False)
        controller_cells = simplify_cell_path(dense_plan.cells)
        waypoints = cells_to_waypoints(maze, controller_cells)

        world = build_maze_world(config, args.seed, args.output_dir)
        world_xml = artifacts["world_xml"]
        Path(world.model_xml_path).replace(world_xml)
        world = _world_with_xml_path(world, world_xml)
        _append_path_markers(world_xml, maze, dense_plan.cells)
        save_svg(maze, artifacts["path_svg"], dense_plan.cells, cell_px=48)
        if Path(world.topdown_svg_path).exists():
            Path(world.topdown_svg_path).replace(artifacts["topdown_svg"])
            world = _world_with_topdown_path(world, artifacts["topdown_svg"])

        mujoco = import_mujoco()
        model = mujoco.MjModel.from_xml_path(str(world_xml))
        data = mujoco.MjData(model)
        adapter = create_policy_adapter("lucky_walker", lucky_g1_repo=args.lucky_g1_repo)
        report = adapter.compatibility_report(model, world_xml)
        report.write_json(artifacts["compatibility_json"])
        if report.errors:
            raise LocomotionPolicyError("; ".join(report.errors))

        start_x, start_y = cell_to_world_xy(maze, maze.spec.start_cell)
        start_yaw = _initial_yaw(waypoints)
        reset_at_pose = getattr(adapter, "reset_at_pose", None)
        if reset_at_pose is None:
            raise LocomotionPolicyError("Lucky walker adapter does not support reset_at_pose.")
        reset_at_pose(model, data, start_x, start_y, start_yaw)
        mujoco.mj_forward(model, data)

        follower = WaypointFollower(waypoints, _controller_config(config))
        run_result = _run_oracle_walk(
            mujoco=mujoco,
            model=model,
            data=data,
            adapter=adapter,
            follower=follower,
            duration_s=args.duration,
            viewer=args.viewer,
            trajectory_csv=artifacts["trajectory_csv"],
            config=config,
        )
        summary.update(run_result)
        save_render(
            mujoco,
            model,
            data,
            artifacts["final_render"],
            int(config.get("locomotion_sandbox", {}).get("render_width", 640)),
            int(config.get("locomotion_sandbox", {}).get("render_height", 480)),
        )
        summary.update(
            {
                "world": world.to_dict(),
                "corridor_width_m": corridor_width_m,
                "waypoint_count": len(waypoints),
                "dense_path_cells": len(dense_plan.cells),
                "controller_waypoint_cells": len(controller_cells),
                "elapsed_wall_s": round(time.time() - started, 3),
                "artifacts": {key: str(path) for key, path in artifacts.items()},
            }
        )
    except (
        ConfigError,
        FileNotFoundError,
        KeyError,
        ValueError,
        PlanningError,
        LocomotionPolicyError,
        MuJoCoImportError,
        MuJoCoModelError,
    ) as exc:
        summary["error"] = str(exc)
        summary["elapsed_wall_s"] = round(time.time() - started, 3)
        _write_json(artifacts["summary_json"], summary)
        _write_dashboard(artifacts["dashboard_html"], summary)
        print(f"Milestone 4 failed: {exc}", file=sys.stderr)
        _print_artifacts(summary, artifacts)
        return 1

    _write_json(artifacts["summary_json"], summary)
    _write_dashboard(artifacts["dashboard_html"], summary)
    _print_artifacts(summary, artifacts)
    return 0


def _run_oracle_walk(
    *,
    mujoco,
    model,
    data,
    adapter,
    follower: WaypointFollower,
    duration_s: float,
    viewer: bool,
    trajectory_csv: Path,
    config: dict,
) -> dict[str, object]:
    sandbox_config = config_from_dict(config)
    control_dt = 1.0 / sandbox_config.control_rate_hz
    sim_substeps = max(1, round(control_dt / float(model.opt.timestep)))
    end_time = data.time + max(0.0, duration_s)
    final_status = "TIMEOUT"
    viewer_opened = False
    last_progress_time = data.time
    best_route_progress = float("-inf")
    contact_stats = _new_contact_stats()
    oracle_config = config.get("oracle", {})
    stuck_timeout = float(oracle_config.get("stuck_timeout_s", 8.0))
    stuck_min_progress = float(oracle_config.get("stuck_min_progress_m", 0.08))

    trajectory_csv.parent.mkdir(parents=True, exist_ok=True)
    with trajectory_csv.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=(
                "time_s",
                "x",
                "y",
                "z",
                "yaw",
                "vx",
                "vy",
                "yaw_rate",
                "status",
                "waypoint_index",
                "distance_to_goal_m",
                "heading_error_rad",
                "contact_count",
                "wall_contact_count",
                "wall_contact_pairs",
            ),
        )
        writer.writeheader()
        if viewer:
            try:
                import mujoco.viewer
            except Exception as exc:
                raise LocomotionPolicyError(f"MuJoCo viewer is unavailable in this environment: {exc}") from exc
            with mujoco.viewer.launch_passive(model, data) as passive_viewer:
                viewer_opened = True
                while passive_viewer.is_running() and data.time < end_time:
                    final_status, best_route_progress, last_progress_time = _step_once(
                        mujoco,
                        model,
                        data,
                        adapter,
                        follower,
                        writer,
                        control_dt,
                        sim_substeps,
                        sandbox_config,
                        best_route_progress,
                        last_progress_time,
                        stuck_timeout,
                        stuck_min_progress,
                        contact_stats,
                    )
                    passive_viewer.sync()
                    if final_status in (GOAL_REACHED, "FALL_DETECTED", "STUCK"):
                        break
                    time.sleep(max(0.0, control_dt))
        else:
            while data.time < end_time:
                final_status, best_route_progress, last_progress_time = _step_once(
                    mujoco,
                    model,
                    data,
                    adapter,
                    follower,
                    writer,
                    control_dt,
                    sim_substeps,
                    sandbox_config,
                    best_route_progress,
                    last_progress_time,
                    stuck_timeout,
                    stuck_min_progress,
                    contact_stats,
                )
                if final_status in (GOAL_REACHED, "FALL_DETECTED", "STUCK"):
                    break

    state = base_state(data)
    return {
        "final_status": final_status,
        "viewer_opened": viewer_opened,
        "final_pose": {"x": state["base_x"], "y": state["base_y"], "z": state["base_z"], "yaw": state["yaw"]},
        "sim_time_s": round(float(data.time), 6),
        "waypoints_reached": int(follower.index + 1),
        "contact_summary": _final_contact_summary(contact_stats),
    }


def _step_once(
    mujoco,
    model,
    data,
    adapter,
    follower: WaypointFollower,
    writer: csv.DictWriter,
    control_dt: float,
    sim_substeps: int,
    sandbox_config,
    best_route_progress: float,
    last_progress_time: float,
    stuck_timeout: float,
    stuck_min_progress: float,
    contact_stats: dict[str, object],
) -> tuple[str, float, float]:
    state = base_state(data)
    controller_output = follower.update(pose_from_base_state(state))
    command = controller_output.command
    route_progress = float(controller_output.waypoint_index) * 1000.0 - controller_output.distance_to_target_m
    contacts = _contact_summary(mujoco, model, data)
    _update_contact_stats(contact_stats, data.time, contacts)
    sim_status = determine_status(command, state, sandbox_config)
    final_status = controller_output.status
    if sim_status == "fallen":
        command = VelocityCommand()
        data.ctrl[:] = 0.0
        final_status = "FALL_DETECTED"
    elif abs(command.vx) <= 1e-4:
        last_progress_time = data.time
    elif route_progress > best_route_progress + stuck_min_progress:
        best_route_progress = route_progress
        last_progress_time = data.time
    elif abs(command.vx) > 1e-4 and data.time - last_progress_time > stuck_timeout:
        command = VelocityCommand()
        final_status = "STUCK"

    if final_status not in (GOAL_REACHED, "FALL_DETECTED", "STUCK"):
        adapter.step(model, data, command, control_dt)

    for _ in range(sim_substeps):
        mujoco.mj_step(model, data)

    writer.writerow(
        {
            "time_s": f"{data.time:.6f}",
            "x": f"{state['base_x']:.9f}",
            "y": f"{state['base_y']:.9f}",
            "z": f"{state['base_z']:.9f}",
            "yaw": f"{state['yaw']:.9f}",
            "vx": f"{command.vx:.9f}",
            "vy": f"{command.vy:.9f}",
            "yaw_rate": f"{command.yaw_rate:.9f}",
            "status": final_status,
            "waypoint_index": follower.index,
            "distance_to_goal_m": f"{controller_output.distance_to_goal_m:.9f}",
            "heading_error_rad": f"{controller_output.heading_error_rad:.9f}",
            "contact_count": contacts["contact_count"],
            "wall_contact_count": contacts["wall_contact_count"],
            "wall_contact_pairs": ";".join(contacts["wall_contact_pairs"]),
        }
    )
    return final_status, best_route_progress, last_progress_time


def _new_contact_stats() -> dict[str, object]:
    return {
        "steps_with_contacts": 0,
        "steps_with_wall_contacts": 0,
        "max_contact_count": 0,
        "max_wall_contact_count": 0,
        "first_wall_contact_time_s": None,
        "first_wall_contact_pairs": [],
        "last_wall_contact_time_s": None,
        "last_wall_contact_pairs": [],
    }


def _contact_summary(mujoco, model, data) -> dict[str, object]:
    contact_count = int(getattr(data, "ncon", 0))
    wall_pairs: list[str] = []
    for index in range(contact_count):
        contact = data.contact[index]
        geom1 = _geom_name(mujoco, model, int(contact.geom1))
        geom2 = _geom_name(mujoco, model, int(contact.geom2))
        if _is_robot_wall_contact(geom1, geom2):
            wall_pairs.append(f"{geom1}<->{geom2}")
    return {
        "contact_count": contact_count,
        "wall_contact_count": len(wall_pairs),
        "wall_contact_pairs": wall_pairs[:6],
    }


def _update_contact_stats(contact_stats: dict[str, object], sim_time: float, contacts: dict[str, object]) -> None:
    contact_count = int(contacts["contact_count"])
    wall_contact_count = int(contacts["wall_contact_count"])
    if contact_count > 0:
        contact_stats["steps_with_contacts"] = int(contact_stats["steps_with_contacts"]) + 1
    if wall_contact_count > 0:
        contact_stats["steps_with_wall_contacts"] = int(contact_stats["steps_with_wall_contacts"]) + 1
        contact_stats["last_wall_contact_time_s"] = round(float(sim_time), 6)
        contact_stats["last_wall_contact_pairs"] = list(contacts["wall_contact_pairs"])
        if contact_stats["first_wall_contact_time_s"] is None:
            contact_stats["first_wall_contact_time_s"] = round(float(sim_time), 6)
            contact_stats["first_wall_contact_pairs"] = list(contacts["wall_contact_pairs"])
    contact_stats["max_contact_count"] = max(int(contact_stats["max_contact_count"]), contact_count)
    contact_stats["max_wall_contact_count"] = max(int(contact_stats["max_wall_contact_count"]), wall_contact_count)


def _final_contact_summary(contact_stats: dict[str, object]) -> dict[str, object]:
    return dict(contact_stats)


def _geom_name(mujoco, model, geom_id: int) -> str:
    if geom_id < 0:
        return ""
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
    if name:
        return name
    body_id = int(model.geom_bodyid[geom_id])
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    return f"geom_{geom_id}:{body_name or f'body_{body_id}'}"


def _is_robot_wall_contact(geom1: str, geom2: str) -> bool:
    return (geom1.startswith("maze_wall_") and _is_robot_geom(geom2)) or (
        geom2.startswith("maze_wall_") and _is_robot_geom(geom1)
    )


def _is_robot_geom(name: str) -> bool:
    if not name:
        return False
    return not (
        name.startswith("maze_")
        or name.startswith("oracle_path_marker_")
        or name in {"floor", "maze_floor", "world", "groundplane"}
    )


def _controller_config(config: dict) -> WaypointFollowerConfig:
    values = config.get("oracle", {})
    return WaypointFollowerConfig(
        waypoint_tolerance_m=float(values.get("waypoint_tolerance_m", 0.35)),
        goal_tolerance_m=float(values.get("goal_tolerance_m", config.get("robot", {}).get("goal_tolerance_m", 0.5))),
        heading_threshold_rad=float(values.get("heading_threshold_rad", 0.45)),
        forward_speed_mps=float(values.get("forward_speed_mps", config.get("robot", {}).get("max_forward_speed_mps", 0.25))),
        arc_turn_speed_mps=float(values.get("arc_turn_speed_mps", 0.4)),
        heading_gain=float(values.get("heading_gain", 1.4)),
        max_yaw_rate_radps=float(values.get("max_yaw_rate_radps", config.get("robot", {}).get("max_yaw_rate_radps", 0.8))),
    )


def _apply_corridor_width_override(config: dict, corridor_width_m: float | None) -> float:
    if corridor_width_m is None:
        return float(config["maze"]["cell_size_m"])
    if corridor_width_m < 1.0 or corridor_width_m > 2.0:
        raise ValueError(
            f"corridor width must be between 1.0 and 2.0 meters, got {corridor_width_m:.3g}. "
            "Use CORRIDOR_WIDTH_M=1.0..2.0."
        )
    config["maze"]["cell_size_m"] = float(corridor_width_m)
    return float(corridor_width_m)


def _initial_yaw(waypoints: list[tuple[float, float, float]]) -> float:
    if len(waypoints) < 2:
        return 0.0
    return math.atan2(waypoints[1][1] - waypoints[0][1], waypoints[1][0] - waypoints[0][0])


def _append_path_markers(world_xml: Path, maze, cells: list[tuple[int, int]]) -> None:
    tree = ET.parse(world_xml)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")
    stride = max(1, len(cells) // 18)
    for index, cell in enumerate(cells):
        if index % stride != 0 and index != len(cells) - 1:
            continue
        x, y = cell_to_world_xy(maze, cell)
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": f"oracle_path_marker_{index}",
                "type": "sphere",
                "pos": f"{x:.6g} {y:.6g} 0.08",
                "size": "0.08",
                "rgba": "0.10 0.35 1.0 0.85",
                "contype": "0",
                "conaffinity": "0",
            },
        )
    tree.write(world_xml, encoding="utf-8", xml_declaration=True)


def _world_with_xml_path(world: WorldBuildResult, path: Path) -> WorldBuildResult:
    return WorldBuildResult(**{**world.to_dict(), "model_xml_path": str(path)})


def _world_with_topdown_path(world: WorldBuildResult, path: Path) -> WorldBuildResult:
    return WorldBuildResult(**{**world.to_dict(), "topdown_svg_path": str(path)})


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_dashboard(path: Path, summary: dict[str, object]) -> None:
    artifacts = summary.get("artifacts", {})
    rows = "\n".join(
        f"<tr><th>{key}</th><td>{value}</td></tr>"
        for key, value in summary.items()
        if not isinstance(value, (dict, list))
    )
    artifact_links = ""
    if isinstance(artifacts, dict):
        artifact_links = "\n".join(
            f'<li><a href="{Path(str(value)).name}">{key}</a></li>' for key, value in artifacts.items()
        )
    final_render = ""
    if isinstance(artifacts, dict) and artifacts.get("final_render"):
        final_render = f'<img src="{Path(str(artifacts["final_render"])).name}" alt="Milestone 4 final render">'
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Milestone 4 Lucky Oracle Walking</title>
  <style>
    body {{ margin: 0; font-family: Arial, sans-serif; background: #f6f7f9; color: #17202a; }}
    main {{ max-width: 1120px; margin: 0 auto; padding: 28px; }}
    h1 {{ font-size: 28px; margin: 0 0 16px; }}
    table {{ border-collapse: collapse; width: 100%; background: white; border: 1px solid #d8dee8; }}
    th, td {{ text-align: left; padding: 9px 11px; border-bottom: 1px solid #e5e9f0; vertical-align: top; }}
    th {{ width: 260px; background: #eef2f6; }}
    img {{ display: block; max-width: 100%; border: 1px solid #d8dee8; background: white; }}
  </style>
</head>
<body>
<main>
  <h1>Milestone 4 Lucky Oracle Walking</h1>
  <table>{rows}</table>
  <h2>Final Render</h2>
  {final_render}
  <h2>Artifacts</h2>
  <ul>{artifact_links}</ul>
</main>
</body>
</html>
"""
    path.write_text(html, encoding="utf-8")


def _print_artifacts(summary: dict[str, object], artifacts: dict[str, Path]) -> None:
    print(json.dumps(summary, indent=2, sort_keys=True))
    for key, path in artifacts.items():
        print(f"{key}_artifact: {path}")


if __name__ == "__main__":
    raise SystemExit(main())
