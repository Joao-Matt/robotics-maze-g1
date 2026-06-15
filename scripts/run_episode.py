"""Run a single Milestone 1 MuJoCo bring-up episode."""

from __future__ import annotations

from pathlib import Path
import argparse
import copy
import json
import os
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sim.config import ConfigError, load_config
from sim.mujoco_runner import MuJoCoRunner, MuJoCoRunnerError
from sim.world_builder import build_maze_world


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a Unitree G1 MuJoCo bring-up episode.")
    parser.add_argument("--seed", type=int, default=1, help="Integer seed for generated maze worlds.")
    parser.add_argument("--duration", type=float, default=None, help="Simulation duration in seconds.")
    parser.add_argument("--viewer", action="store_true", help="Launch MuJoCo passive viewer if available.")
    parser.add_argument(
        "--world",
        choices=("maze", "empty"),
        default="maze",
        help="World source: generated maze world or original empty bring-up scene.",
    )
    parser.add_argument(
        "--world-output-dir",
        type=Path,
        default=PROJECT_ROOT / "runs" / "visual",
        help="Directory for generated maze world artifacts.",
    )
    parser.add_argument("--save-summary-json", type=Path, default=None, help="Optional JSON summary artifact path.")
    parser.add_argument("--save-render", type=Path, default=None, help="Optional final frame PPM artifact path.")
    parser.add_argument("--render-width", type=int, default=640, help="Final render width in pixels.")
    parser.add_argument("--render-height", type=int, default=480, help="Final render height in pixels.")
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
        run_config = copy.deepcopy(config)
        world_summary = None
        if args.world == "maze":
            world_summary = build_maze_world(run_config, args.seed, args.world_output_dir)
            run_config["robot"]["model_xml_path"] = world_summary.model_xml_path

        summary = MuJoCoRunner(run_config).run(
            duration_s=args.duration,
            viewer=args.viewer,
            render_path=args.save_render,
            render_width=args.render_width,
            render_height=args.render_height,
        )
        if world_summary:
            summary["world"] = world_summary.to_dict()
    except (ConfigError, MuJoCoRunnerError, FileNotFoundError, KeyError, ValueError) as exc:
        print(f"Episode failed: {exc}", file=sys.stderr)
        return 1

    print(f"seed: {args.seed}")
    print(f"mode: {args.world}")
    print(json.dumps(summary, indent=2, sort_keys=True))
    if world_summary:
        print(f"world_xml_artifact: {world_summary.model_xml_path}")
        print(f"world_summary_artifact: {world_summary.summary_json_path}")
        print(f"world_topdown_artifact: {world_summary.topdown_svg_path}")
    if args.save_summary_json:
        args.save_summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.save_summary_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"run_summary_artifact: {args.save_summary_json}")
    if args.save_render:
        if summary.get("render_path"):
            print(f"run_render_artifact: {summary['render_path']}")
        elif summary.get("render_error"):
            print(f"run_render_warning: {summary['render_error']}")

    if args.viewer:
        # Some Linux/OpenGL stacks can crash during Python teardown after the
        # passive viewer closes. At this point the viewer run is complete and
        # all intended output has been flushed.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
