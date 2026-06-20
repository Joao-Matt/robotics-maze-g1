"""Generate, validate, and print a seeded maze."""

from __future__ import annotations

from pathlib import Path
import argparse
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from maze.generator import generate_maze_from_config
from maze.validator import raise_for_invalid, validate_maze
from maze.visualization import maze_to_ascii, save_ascii, save_pgm, save_svg
from sim.config import ConfigError, load_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate and validate a seeded maze.")
    parser.add_argument("--seed", type=int, default=1, help="Integer maze seed.")
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "default.yaml",
        help="Path to YAML config.",
    )
    parser.add_argument("--show-path", action="store_true", help="Overlay the BFS validation path.")
    parser.add_argument("--save-ascii", type=Path, default=None, help="Optional path for ASCII output.")
    parser.add_argument("--save-pgm", type=Path, default=None, help="Optional path for PGM image output.")
    parser.add_argument("--save-svg", type=Path, default=None, help="Optional path for readable SVG output.")
    parser.add_argument("--cell-px", type=int, default=36, help="Cell size in pixels for SVG output.")
    parser.add_argument("--cell-size-m", type=float, default=None, help="Override square maze cell size in meters.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        config = load_config(args.config)
        if args.cell_size_m is not None:
            if args.cell_size_m < 1.0 or args.cell_size_m > 4.0:
                raise ValueError(f"cell size must be between 1.0 and 4.0 meters, got {args.cell_size_m:.3g}")
            config["maze"]["cell_size_m"] = float(args.cell_size_m)
            config["maze"]["cell_width_m"] = float(args.cell_size_m)
            config["maze"]["cell_length_m"] = float(args.cell_size_m)
        maze = generate_maze_from_config(config, args.seed)
        result = validate_maze(
            maze,
            safety_radius_m=float(config["robot"]["safety_radius_m"]),
            min_corridor_width_m=float(config["maze"]["min_corridor_width_m"]),
            max_corridor_width_m=(
                float(config["maze"]["max_corridor_width_m"])
                if "max_corridor_width_m" in config["maze"]
                else None
            ),
        )
        raise_for_invalid(result)
    except (ConfigError, KeyError, ValueError) as exc:
        print(f"Maze generation failed: {exc}", file=sys.stderr)
        return 1

    route = result.path if args.show_path else None
    print(f"seed: {args.seed}")
    print(f"size: {maze.spec.width_cells}x{maze.spec.height_cells}")
    print(f"path_cells: {len(result.path or [])}")
    print(maze_to_ascii(maze, route))

    if args.save_ascii:
        save_ascii(maze, args.save_ascii, route)
        print(f"saved_ascii: {args.save_ascii}")
    if args.save_pgm:
        save_pgm(maze, args.save_pgm, route)
        print(f"saved_pgm: {args.save_pgm}")
    if args.save_svg:
        save_svg(maze, args.save_svg, route, cell_px=args.cell_px)
        print(f"saved_svg: {args.save_svg}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
