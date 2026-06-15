"""Generate a MuJoCo maze world XML for visual inspection and simulation."""

from __future__ import annotations

from pathlib import Path
import argparse
import json
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sim.config import ConfigError, load_config
from sim.world_builder import build_maze_world


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a seeded MuJoCo maze world.")
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
        help="Directory for generated XML and visual artifacts.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        config = load_config(args.config)
        result = build_maze_world(config, args.seed, args.output_dir)
    except (ConfigError, FileNotFoundError, KeyError, ValueError) as exc:
        print(f"World generation failed: {exc}", file=sys.stderr)
        return 1

    print(f"seed: {args.seed}")
    print("mode: world-build")
    print(json.dumps(result.to_dict(), indent=2, sort_keys=True))
    print(f"world_xml_artifact: {result.model_xml_path}")
    print(f"world_summary_artifact: {result.summary_json_path}")
    print(f"world_topdown_artifact: {result.topdown_svg_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
