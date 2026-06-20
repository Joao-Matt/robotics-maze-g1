#!/usr/bin/env python3
"""Generate Phase 1 D435i mount, RGB, depth, and dashboard artifacts."""

from __future__ import annotations

from html import escape
from pathlib import Path
from typing import Any
import argparse
import json
import math
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sim.config import ConfigError, load_config
from sim.d435i_sensor import D435iSpec
from sim.mujoco_runner import _write_png, import_mujoco
from maze.generator import generate_maze_from_config
from maze.validator import validate_maze
from sim.world_builder import build_maze_world, cell_to_world_xy


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate the simulated D435i-style G1 mount.")
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs" / "default.yaml")
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "runs" / "visual")
    return parser.parse_args()


def artifact_paths(output_dir: Path, seed: int) -> dict[str, Path]:
    return {
        "mount_image": output_dir / f"d435i_mount_seed-{seed}_final.png",
        "rgb_image": output_dir / f"d435i_camera_view_seed-{seed}_rgb.png",
        "depth_image": output_dir / f"d435i_camera_view_seed-{seed}_depth.png",
        "dashboard": output_dir / "d435i_mount_dashboard.html",
        "summary": output_dir / "d435i_mount_summary.json",
    }


def normalize_depth(depth: np.ndarray, minimum: float, maximum: float) -> tuple[np.ndarray, dict[str, float | int]]:
    valid = np.isfinite(depth) & (depth > 0.0)
    if not np.any(valid):
        raise ValueError("depth camera produced no finite positive pixels")
    clipped = np.clip(depth, minimum, maximum)
    intensity = 255.0 * (maximum - clipped) / (maximum - minimum)
    gray = np.where(valid, intensity, 0.0).astype(np.uint8)
    rgb = np.repeat(gray[:, :, None], 3, axis=2)
    values = depth[valid]
    return rgb, {
        "valid_pixel_count": int(values.size),
        "minimum_m": float(np.min(values)),
        "maximum_m": float(np.max(values)),
        "mean_m": float(np.mean(values)),
        "visual_min_m": minimum,
        "visual_max_m": maximum,
    }


def render_artifacts(mujoco, model, data, spec: D435iSpec, paths: dict[str, Path]) -> dict[str, object]:
    renderer = mujoco.Renderer(model, width=spec.width, height=spec.height)
    try:
        renderer.update_scene(data, camera=spec.rgb_camera_name)
        rgb = renderer.render().copy()
        _write_png(paths["rgb_image"], rgb)

        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=spec.depth_camera_name)
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()
        depth_rgb, depth_stats = normalize_depth(
            depth, spec.depth_visual_min_m, spec.depth_visual_max_m
        )
        _write_png(paths["depth_image"], depth_rgb)

        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, spec.link_name)
        inspection = mujoco.MjvCamera()
        inspection.type = mujoco.mjtCamera.mjCAMERA_FREE
        inspection.lookat[:] = data.xpos[body_id]
        inspection.distance = 1.35
        inspection.azimuth = 145.0
        inspection.elevation = -8.0
        inspection_options = mujoco.MjvOption()
        inspection_options.geomgroup[0] = 0  # Hide maze geometry for an unobstructed mount close-up.
        inspection_options.geomgroup[3] = 0  # Hide the robot's translucent collision geometry.
        renderer.update_scene(data, camera=inspection, scene_option=inspection_options)
        mount = renderer.render().copy()
        _write_png(paths["mount_image"], mount)
    finally:
        renderer.close()

    return {
        "depth": depth_stats,
        "rgb_non_uniform": bool(np.ptp(rgb.astype(np.int16)) > 0),
        "depth_non_uniform": bool(np.ptp(depth_rgb.astype(np.int16)) > 0),
    }


def camera_pose(mujoco, model, data, camera_name: str) -> dict[str, list[float]]:
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    return {
        "position_world_m": [float(value) for value in data.cam_xpos[camera_id]],
        "rotation_world_matrix": [float(value) for value in data.cam_xmat[camera_id]],
    }


def initial_path_yaw(config: dict[str, Any], seed: int) -> float:
    """Face the first oracle-path segment so the sensor shows the open maze corridor."""
    maze = generate_maze_from_config(config, seed)
    path = validate_maze(
        maze,
        safety_radius_m=float(config["robot"]["safety_radius_m"]),
        min_corridor_width_m=float(config["maze"]["min_corridor_width_m"]),
        max_corridor_width_m=(
            float(config["maze"]["max_corridor_width_m"])
            if "max_corridor_width_m" in config["maze"]
            else None
        ),
    ).path
    if len(path) < 2:
        return 0.0
    start_x, start_y = cell_to_world_xy(maze, path[0])
    next_x, next_y = cell_to_world_xy(maze, path[1])
    return math.atan2(next_y - start_y, next_x - start_x)


def write_dashboard(path: Path, summary: dict[str, object], paths: dict[str, Path]) -> None:
    metadata = escape(json.dumps(summary, indent=2, sort_keys=True))
    panels = "".join(
        f'<section><h2>{escape(title)}</h2><img src="{escape(image.name)}" alt="{escape(title)}"></section>'
        for title, image in (
            ("Visible D435i Mount", paths["mount_image"]),
            ("D435i RGB View", paths["rgb_image"]),
            ("D435i Depth View", paths["depth_image"]),
        )
    )
    path.write_text(
        f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>D435i Mount Visual Check</title>
<style>body{{font-family:sans-serif;margin:1.5rem;background:#111827;color:#e5e7eb}}main{{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:1rem}}section,pre{{background:#1f2937;padding:1rem;border-radius:.5rem}}img{{width:100%;height:auto}}pre{{white-space:pre-wrap;overflow-wrap:anywhere}}</style>
</head><body><h1>D435i Mount Visual Check</h1><main>{panels}</main><h2>Metadata</h2><pre>{metadata}</pre></body></html>
""",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    paths = artifact_paths(args.output_dir, args.seed)
    try:
        config = load_config(args.config)
        spec = D435iSpec.from_config(config)
        if spec is None:
            raise ValueError("d435i is disabled or missing from the configuration")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        world = build_maze_world(config, args.seed, args.output_dir)
        mujoco = import_mujoco()
        model = mujoco.MjModel.from_xml_path(world.model_xml_path)
        data = mujoco.MjData(model)
        data.qpos[:3] = np.asarray(world.start_world_xyz)
        yaw = initial_path_yaw(config, args.seed)
        data.qpos[3:7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        mujoco.mj_forward(model, data)

        render_details = render_artifacts(mujoco, model, data, spec, paths)
        summary: dict[str, object] = {
            "status": "completed",
            "phase": 1,
            "seed": args.seed,
            "model_xml_path": world.base_model_xml_path,
            "world_xml_path": world.model_xml_path,
            "initial_yaw_rad": yaw,
            "mount": {
                "parent_body": spec.parent_body,
                "link_name": spec.link_name,
                "position_parent_m": list(spec.mount_pos_m),
                "pitch_deg": spec.pitch_deg,
            },
            "frames": {
                "rgb_optical": spec.rgb_optical_frame,
                "depth_optical": spec.depth_optical_frame,
                "imu": spec.imu_frame,
            },
            "rgb_camera": {
                **spec.camera_metadata(spec.rgb_camera_name),
                **camera_pose(mujoco, model, data, spec.rgb_camera_name),
            },
            "depth_camera": {
                **spec.camera_metadata(spec.depth_camera_name),
                **camera_pose(mujoco, model, data, spec.depth_camera_name),
                "visual_encoding": "near-white far-black normalized 8-bit grayscale PNG",
            },
            "render_validation": render_details,
            "artifacts": {key: str(value) for key, value in paths.items()},
        }
        paths["summary"].write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        write_dashboard(paths["dashboard"], summary, paths)
    except (ConfigError, FileNotFoundError, ImportError, KeyError, RuntimeError, ValueError) as exc:
        print(f"D435i visual check failed: {exc}", file=sys.stderr)
        return 1

    print(json.dumps(summary, indent=2, sort_keys=True))
    for name, path in paths.items():
        print(f"{name}_artifact: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
