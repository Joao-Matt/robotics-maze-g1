"""Run the turn-aware Lucky G1 oracle follower in a generated maze."""

from __future__ import annotations

from pathlib import Path
import argparse
import csv
import json
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from maze.generator import generate_maze_from_config
from nav.controller import GOAL_REACHED as LEGACY_GOAL_REACHED, pose_from_base_state
from nav.oracle_follow import (
    FAILED,
    GOAL_REACHED,
    TurnAwareFollowerConfig,
    TurnAwareOracleFollower,
    build_turn_aware_path,
)
from nav.planner import PlanningError, plan_oracle_path
from sim.config import ConfigError, load_config
from sim.locomotion_policy_adapter import LocomotionPolicyError, VelocityCommand, create_policy_adapter
from sim.locomotion_sandbox import base_state, config_from_dict, determine_status, save_render
from sim.mujoco_runner import MuJoCoImportError, MuJoCoModelError, import_mujoco
from sim.world_builder import cell_to_world_xy
from scripts.run_milestone_4 import (
    _append_path_markers,
    _apply_corridor_width_override,
    _contact_summary,
    _final_contact_summary,
    _new_contact_stats,
    _print_artifacts,
    _update_contact_stats,
    _world_with_topdown_path,
    _world_with_xml_path,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Turn-aware Lucky G1 oracle follower.")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--label", default="", help="Optional artifact label.")
    parser.add_argument("--corridor-width-m", type=float, default=None)
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
    prefix = f"g1_oracle_follow{label}_seed-{args.seed}"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    artifacts = {
        "world_xml": args.output_dir / f"{prefix}_world.xml",
        "topdown_overlay_svg": args.output_dir / f"{prefix}_topdown_overlay.svg",
        "trajectory_csv": args.output_dir / f"{prefix}_trajectory.csv",
        "commands_csv": args.output_dir / f"{prefix}_commands.csv",
        "events_jsonl": args.output_dir / f"{prefix}_events.jsonl",
        "summary_json": args.output_dir / f"{prefix}_summary.json",
        "dashboard_html": args.output_dir / f"{prefix}_dashboard.html",
        "final_render": args.output_dir / f"{prefix}_final.png",
        "compatibility_json": args.output_dir / f"{prefix}_policy_compatibility.json",
    }
    started = time.time()
    summary: dict[str, object] = {
        "milestone": 5,
        "mode": "milestone_5_oracle_path_execution",
        "seed": args.seed,
        "policy": "lucky_walker",
        "final_status": "ERROR",
        "error": None,
        "failure_reason": None,
        "viewer_requested": bool(args.viewer),
        "viewer_opened": False,
        "proxy_body_used": False,
        "contact_summary": {},
        "rerun_live_command": f"make milestone_5 SEED={args.seed}",
        "rerun_headless_command": f"make report-milestone_5 SEED={args.seed}",
    }

    try:
        config = load_config(args.config)
        corridor_width_m = _apply_corridor_width_override(config, args.corridor_width_m)
        follower_config = _follower_config(config)
        maze = generate_maze_from_config(config, args.seed)
        oracle_values = config.get("oracle", {})
        dense_plan = plan_oracle_path(
            maze,
            safety_radius_m=float(config["robot"]["safety_radius_m"]),
            simplify=False,
            planner="heading_astar",
            turn_penalty_cost=float(oracle_values.get("turn_penalty_cost", 2.0)),
        )
        turn_path = build_turn_aware_path(maze, dense_plan.cells, follower_config)

        world = _build_world(
            config,
            args.seed,
            args.output_dir,
            artifacts["world_xml"],
            artifacts["topdown_overlay_svg"],
            maze,
            dense_plan.cells,
        )
        mujoco = import_mujoco()
        model = mujoco.MjModel.from_xml_path(str(artifacts["world_xml"]))
        data = mujoco.MjData(model)
        adapter = create_policy_adapter("lucky_walker", lucky_g1_repo=args.lucky_g1_repo)
        report = adapter.compatibility_report(model, artifacts["world_xml"])
        report.write_json(artifacts["compatibility_json"])
        if report.errors:
            raise LocomotionPolicyError("; ".join(report.errors))

        start_x, start_y = cell_to_world_xy(maze, maze.spec.start_cell)
        reset_at_pose = getattr(adapter, "reset_at_pose", None)
        if reset_at_pose is None:
            raise LocomotionPolicyError("Lucky walker adapter does not support reset_at_pose.")
        reset_at_pose(model, data, start_x, start_y, turn_path.segments[0].target_heading_rad)
        mujoco.mj_forward(model, data)

        follower = TurnAwareOracleFollower(turn_path, follower_config)
        run_result = _run_follow(
            mujoco=mujoco,
            model=model,
            data=data,
            adapter=adapter,
            follower=follower,
            duration_s=args.duration,
            viewer=args.viewer,
            trajectory_csv=artifacts["trajectory_csv"],
            commands_csv=artifacts["commands_csv"],
            events_jsonl=artifacts["events_jsonl"],
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
        _write_overlay_svg(
            maze=maze,
            path=turn_path,
            actual_xy=run_result["actual_xy"],
            output_path=artifacts["topdown_overlay_svg"],
        )
        summary.update(
            {
                "world": world.to_dict(),
                "corridor_width_m": corridor_width_m,
                "dense_path_cells": len(dense_plan.cells),
                "navigation_segments": len(turn_path.segments),
                "pre_turn_points": len(turn_path.pre_turn_points),
                "arc_turn_segments": len(turn_path.arc_segments),
                "turns": _turn_counts(turn_path),
                "controller_config": follower_config.__dict__,
                "elapsed_wall_s": round(time.time() - started, 3),
                "artifacts": {key: str(path) for key, path in artifacts.items()},
            }
        )
        summary.pop("actual_xy", None)
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
        _ensure_artifact_placeholders(artifacts)
        _write_json(artifacts["summary_json"], summary)
        _write_dashboard(artifacts["dashboard_html"], summary)
        print(f"G1 oracle follow failed: {exc}", file=sys.stderr)
        _print_artifacts(summary, artifacts)
        return 1

    _write_json(artifacts["summary_json"], summary)
    _write_dashboard(artifacts["dashboard_html"], summary)
    _print_artifacts(summary, artifacts)
    return 0


def _build_world(config: dict, seed: int, output_dir: Path, world_xml: Path, topdown_overlay_svg: Path, maze, cells):
    from sim.world_builder import build_maze_world

    world = build_maze_world(config, seed, output_dir)
    Path(world.model_xml_path).replace(world_xml)
    world = _world_with_xml_path(world, world_xml)
    _append_path_markers(world_xml, maze, cells)
    if Path(world.topdown_svg_path).exists():
        Path(world.topdown_svg_path).unlink()
    return _world_with_topdown_path(world, topdown_overlay_svg)


def _run_follow(
    *,
    mujoco,
    model,
    data,
    adapter,
    follower: TurnAwareOracleFollower,
    duration_s: float,
    viewer: bool,
    trajectory_csv: Path,
    commands_csv: Path,
    events_jsonl: Path,
    config: dict,
) -> dict[str, object]:
    sandbox_config = config_from_dict(config)
    control_dt = 1.0 / sandbox_config.control_rate_hz
    sim_substeps = max(1, round(control_dt / float(model.opt.timestep)))
    end_time = data.time + max(0.0, duration_s)
    final_status = "TIMEOUT"
    viewer_opened = False
    actual_xy: list[tuple[float, float]] = []
    contact_stats = _new_contact_stats()
    event_counts: dict[str, int] = {}

    trajectory_csv.parent.mkdir(parents=True, exist_ok=True)
    with (
        trajectory_csv.open("w", newline="", encoding="utf-8") as trajectory_file,
        commands_csv.open("w", newline="", encoding="utf-8") as commands_file,
        events_jsonl.open("w", encoding="utf-8") as events_file,
    ):
        trajectory_writer = csv.DictWriter(
            trajectory_file,
            fieldnames=(
                "time_s",
                "x",
                "y",
                "z",
                "yaw",
                "controller_state",
                "segment_index",
                "target_x",
                "target_y",
                "distance_to_target_m",
                "heading_error_rad",
                "progress_m",
                "recovery_attempts",
                "contact_count",
                "wall_contact_count",
                "wall_contact_pairs",
            ),
        )
        command_writer = csv.DictWriter(
            commands_file,
            fieldnames=("time_s", "vx", "vy", "yaw_rate", "controller_state", "segment_index"),
        )
        trajectory_writer.writeheader()
        command_writer.writeheader()

        def step_loop() -> str:
            nonlocal final_status
            while data.time < end_time:
                final_status = _step_once(
                    mujoco,
                    model,
                    data,
                    adapter,
                    follower,
                    trajectory_writer,
                    command_writer,
                    events_file,
                    event_counts,
                    actual_xy,
                    contact_stats,
                    control_dt,
                    sim_substeps,
                    sandbox_config,
                )
                if final_status in (GOAL_REACHED, FAILED, "FALL_DETECTED"):
                    break
            return final_status

        if viewer:
            try:
                import mujoco.viewer
            except Exception as exc:
                raise LocomotionPolicyError(f"MuJoCo viewer is unavailable in this environment: {exc}") from exc
            with mujoco.viewer.launch_passive(model, data) as passive_viewer:
                viewer_opened = True
                while passive_viewer.is_running() and data.time < end_time:
                    final_status = _step_once(
                        mujoco,
                        model,
                        data,
                        adapter,
                        follower,
                        trajectory_writer,
                        command_writer,
                        events_file,
                        event_counts,
                        actual_xy,
                        contact_stats,
                        control_dt,
                        sim_substeps,
                        sandbox_config,
                    )
                    passive_viewer.sync()
                    if final_status in (GOAL_REACHED, FAILED, "FALL_DETECTED"):
                        break
                    time.sleep(max(0.0, control_dt))
        else:
            step_loop()

    state = base_state(data)
    if final_status not in (GOAL_REACHED, FAILED, "FALL_DETECTED") and data.time >= end_time:
        final_status = "TIMEOUT"
    return {
        "final_status": final_status,
        "failure_reason": follower.failure_reason,
        "viewer_opened": viewer_opened,
        "final_controller_state": follower.state,
        "final_segment_index": follower.segment_index,
        "recovery_attempts": follower.recovery_attempts,
        "event_counts": event_counts,
        "contact_summary": _final_contact_summary(contact_stats),
        "final_pose": {"x": state["base_x"], "y": state["base_y"], "z": state["base_z"], "yaw": state["yaw"]},
        "sim_time_s": round(float(data.time), 6),
        "actual_xy": actual_xy,
    }


def _step_once(
    mujoco,
    model,
    data,
    adapter,
    follower: TurnAwareOracleFollower,
    trajectory_writer: csv.DictWriter,
    command_writer: csv.DictWriter,
    events_file,
    event_counts: dict[str, int],
    actual_xy: list[tuple[float, float]],
    contact_stats: dict[str, object],
    control_dt: float,
    sim_substeps: int,
    sandbox_config,
) -> str:
    state = base_state(data)
    output = follower.update(pose_from_base_state(state), float(data.time))
    command = output.command
    contacts = _contact_summary(mujoco, model, data)
    _update_contact_stats(contact_stats, data.time, contacts)
    sim_status = determine_status(command, state, sandbox_config)
    final_status = output.state
    if sim_status == "fallen":
        command = VelocityCommand()
        data.ctrl[:] = 0.0
        final_status = "FALL_DETECTED"
        _write_event(
            events_file,
            event_counts,
            {
                "time_s": round(float(data.time), 6),
                "event": "fall_detected",
                "state": final_status,
                "segment_index": output.segment_index,
                "detail": "base height/tilt exceeded locomotion sandbox limits",
                "turn_direction": None,
            },
        )
    else:
        for event in output.events:
            _write_event(events_file, event_counts, event.to_dict())

    if final_status not in (GOAL_REACHED, LEGACY_GOAL_REACHED, FAILED, "FALL_DETECTED"):
        adapter.step(model, data, command, control_dt)

    command_writer.writerow(
        {
            "time_s": f"{data.time:.6f}",
            "vx": f"{command.vx:.9f}",
            "vy": f"{command.vy:.9f}",
            "yaw_rate": f"{command.yaw_rate:.9f}",
            "controller_state": output.state,
            "segment_index": output.segment_index,
        }
    )

    for _ in range(sim_substeps):
        mujoco.mj_step(model, data)

    next_state = base_state(data)
    actual_xy.append((float(next_state["base_x"]), float(next_state["base_y"])))
    trajectory_writer.writerow(
        {
            "time_s": f"{data.time:.6f}",
            "x": f"{next_state['base_x']:.9f}",
            "y": f"{next_state['base_y']:.9f}",
            "z": f"{next_state['base_z']:.9f}",
            "yaw": f"{next_state['yaw']:.9f}",
            "controller_state": output.state,
            "segment_index": output.segment_index,
            "target_x": f"{output.target_xy[0]:.9f}",
            "target_y": f"{output.target_xy[1]:.9f}",
            "distance_to_target_m": f"{output.distance_to_target_m:.9f}",
            "heading_error_rad": f"{output.heading_error_rad:.9f}",
            "progress_m": f"{output.progress_m:.9f}",
            "recovery_attempts": output.recovery_attempts,
            "contact_count": contacts["contact_count"],
            "wall_contact_count": contacts["wall_contact_count"],
            "wall_contact_pairs": ";".join(contacts["wall_contact_pairs"]),
        }
    )
    return GOAL_REACHED if final_status == LEGACY_GOAL_REACHED else final_status


def _write_event(events_file, event_counts: dict[str, int], event: dict[str, object]) -> None:
    event_name = str(event.get("event", "unknown"))
    event_counts[event_name] = event_counts.get(event_name, 0) + 1
    events_file.write(json.dumps(event, sort_keys=True) + "\n")
    events_file.flush()


def _follower_config(config: dict) -> TurnAwareFollowerConfig:
    values = config.get("oracle", {})
    return TurnAwareFollowerConfig(
        approach_tolerance_m=float(values.get("approach_tolerance_m", 0.35)),
        waypoint_tolerance_m=float(values.get("waypoint_tolerance_m", 0.75)),
        goal_tolerance_m=float(values.get("goal_tolerance_m", config.get("robot", {}).get("goal_tolerance_m", 0.5))),
        heading_threshold_rad=float(values.get("heading_threshold_rad", 0.45)),
        forward_speed_mps=float(values.get("forward_speed_mps", 0.8)),
        heading_gain=float(values.get("heading_gain", 1.4)),
        max_yaw_rate_radps=float(values.get("max_yaw_rate_radps", 0.8)),
        turn_start_distance_m=float(values.get("turn_start_distance_m", values.get("pre_turn_distance_m", 0.8))),
        pre_turn_distance_m=float(values.get("pre_turn_distance_m", values.get("turn_start_distance_m", 0.8))),
        arc_turn_forward_speed_mps=float(values.get("arc_turn_forward_speed_mps", 0.4)),
        arc_turn_yaw_rate_radps=float(values.get("arc_turn_yaw_rate_radps", 0.8)),
        post_turn_heading_tolerance_rad=float(values.get("post_turn_heading_tolerance_rad", 0.3)),
        stuck_timeout_s=float(values.get("stuck_timeout_s", 8.0)),
        stuck_min_progress_m=float(values.get("stuck_min_progress_m", 0.08)),
        max_recovery_attempts=int(values.get("max_recovery_attempts", 2)),
    )


def _turn_counts(path) -> dict[str, int]:
    counts = {"left": 0, "right": 0}
    for segment in path.arc_segments:
        if segment.turn_direction in counts:
            counts[segment.turn_direction] += 1
    return counts


def _write_overlay_svg(*, maze, path, actual_xy: list[tuple[float, float]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cell_px = 48
    width = maze.spec.width_cells * cell_px
    height = maze.spec.height_cells * cell_px

    def world_to_px(point: tuple[float, float]) -> tuple[float, float]:
        col = point[0] / maze.spec.cell_size_m + (maze.spec.width_cells - 1) / 2.0
        row = (maze.spec.height_cells - 1) / 2.0 - point[1] / maze.spec.cell_size_m
        return col * cell_px + cell_px / 2.0, row * cell_px + cell_px / 2.0

    def polyline(points: list[tuple[float, float]], color: str, width_px: float) -> str:
        if len(points) < 2:
            return ""
        rendered = " ".join(f"{x:.2f},{y:.2f}" for x, y in (world_to_px(point) for point in points))
        return f'<polyline points="{rendered}" fill="none" stroke="{color}" stroke-width="{width_px}" stroke-linecap="round" stroke-linejoin="round"/>'

    elements = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        "<title>Turn-aware G1 oracle follow overlay</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
    ]
    for row in range(maze.spec.height_cells):
        for col in range(maze.spec.width_cells):
            x = col * cell_px
            y = row * cell_px
            fill = "#111827" if maze.grid[row, col] else "#ffffff"
            elements.append(f'<rect x="{x}" y="{y}" width="{cell_px}" height="{cell_px}" fill="{fill}" stroke="#cbd5e1"/>')

    planned_points = [(x, y) for x, y, _ in path.dense_waypoints]
    elements.append(polyline(planned_points, "#2563eb", 5))
    for arc in path.arc_segments:
        color = "#dc2626" if arc.turn_direction == "right" else "#7c3aed"
        elements.append(polyline([arc.start_xy, arc.end_xy], color, 8))
        x, y = world_to_px(arc.corner_xy or arc.end_xy)
        label = "R" if arc.turn_direction == "right" else "L"
        elements.append(f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="middle" dominant-baseline="central" font-size="18" font-weight="700" fill="{color}">{label}</text>')
    for point in path.pre_turn_points:
        x, y = world_to_px(point)
        elements.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="7" fill="#f59e0b" stroke="#92400e" stroke-width="2"/>')
    for point in path.post_turn_points:
        x, y = world_to_px(point)
        elements.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="5" fill="#10b981" stroke="#047857" stroke-width="2"/>')
    elements.append(polyline(actual_xy, "#0f172a", 3))
    sx, sy = world_to_px(cell_to_world_xy(maze, maze.spec.start_cell))
    gx, gy = world_to_px(cell_to_world_xy(maze, maze.spec.goal_cell))
    elements.append(f'<text x="{sx:.2f}" y="{sy:.2f}" text-anchor="middle" dominant-baseline="central" font-size="22" font-weight="700" fill="#166534">S</text>')
    elements.append(f'<text x="{gx:.2f}" y="{gy:.2f}" text-anchor="middle" dominant-baseline="central" font-size="22" font-weight="700" fill="#991b1b">G</text>')
    elements.append("</svg>")
    output_path.write_text("\n".join(elements) + "\n", encoding="utf-8")


def _write_dashboard(path: Path, summary: dict[str, object]) -> None:
    artifacts = summary.get("artifacts", {})
    rows = "\n".join(
        f"<tr><th>{key}</th><td><code>{value}</code></td></tr>"
        for key, value in summary.items()
        if not isinstance(value, (dict, list))
    )
    links = ""
    if isinstance(artifacts, dict):
        links = "\n".join(f'<li><a href="{Path(str(value)).name}">{key}</a></li>' for key, value in artifacts.items())
    overlay = ""
    render = ""
    if isinstance(artifacts, dict) and artifacts.get("topdown_overlay_svg"):
        overlay = f'<img src="{Path(str(artifacts["topdown_overlay_svg"])).name}" alt="topdown path overlay">'
    if isinstance(artifacts, dict) and artifacts.get("final_render"):
        render = f'<img src="{Path(str(artifacts["final_render"])).name}" alt="final MuJoCo render">'
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Milestone 5 — Turn-aware G1 Oracle Path Execution</title>
  <style>
    body {{ margin: 0; font-family: Arial, sans-serif; background: #f6f7f9; color: #17202a; }}
    main {{ max-width: 1280px; margin: 0 auto; padding: 28px; }}
    h1 {{ font-size: 28px; margin: 0 0 16px; }}
    .grid {{ display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1fr); gap: 18px; align-items: start; }}
    table {{ border-collapse: collapse; width: 100%; background: white; border: 1px solid #d8dee8; margin-bottom: 18px; }}
    th, td {{ text-align: left; padding: 9px 11px; border-bottom: 1px solid #e5e9f0; vertical-align: top; }}
    th {{ width: 260px; background: #eef2f6; }}
    img {{ display: block; max-width: 100%; border: 1px solid #d8dee8; background: white; }}
    @media (max-width: 900px) {{ .grid {{ grid-template-columns: 1fr; }} }}
  </style>
</head>
<body>
<main>
  <h1>Milestone 5 — Turn-aware G1 Oracle Path Execution</h1>
  <table>{rows}</table>
  <div class="grid">
    <section><h2>Path Overlay</h2>{overlay}</section>
    <section><h2>Final Render</h2>{render}</section>
  </div>
  <h2>Artifacts</h2>
  <ul>{links}</ul>
</main>
</body>
</html>
"""
    path.write_text(html, encoding="utf-8")


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _ensure_artifact_placeholders(artifacts: dict[str, Path]) -> None:
    artifacts["trajectory_csv"].write_text("", encoding="utf-8")
    artifacts["commands_csv"].write_text("", encoding="utf-8")
    artifacts["events_jsonl"].write_text("", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
