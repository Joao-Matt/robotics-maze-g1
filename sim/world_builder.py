"""Build generated MuJoCo maze worlds for Milestone 3."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any
import json
import xml.etree.ElementTree as ET

import numpy as np

from maze.generator import generate_maze_from_config
from maze.grid import WALL, Cell, Maze
from maze.validator import raise_for_invalid, validate_maze
from maze.visualization import save_svg
from sim.mujoco_runner import PROJECT_ROOT, resolve_project_path


@dataclass(frozen=True)
class WorldBuildResult:
    """Paths and metadata for a generated MuJoCo maze world."""

    seed: int
    model_xml_path: str
    summary_json_path: str
    topdown_svg_path: str
    base_model_xml_path: str
    wall_count: int
    free_count: int
    start_cell: Cell
    goal_cell: Cell
    start_world_xyz: tuple[float, float, float]
    goal_world_xyz: tuple[float, float, float]
    coordinate_convention: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


COORDINATE_CONVENTION = (
    "world origin is the maze center; x=(col-(width_cells-1)/2)*cell_size_m; "
    "y=((height_cells-1)/2-row)*cell_size_m; z=0 is floor height"
)


def cell_to_world_xy(maze: Maze, cell: Cell) -> tuple[float, float]:
    """Convert a maze cell to the center of that cell in MuJoCo world x/y."""
    row, col = cell
    spec = maze.spec
    x = (col - (spec.width_cells - 1) / 2.0) * spec.cell_size_m
    y = ((spec.height_cells - 1) / 2.0 - row) * spec.cell_size_m
    return float(x), float(y)


def build_maze_world(
    config: dict[str, Any],
    seed: int,
    output_dir: Path,
    project_root: Path = PROJECT_ROOT,
) -> WorldBuildResult:
    """Generate a validated maze world XML and visible debug artifacts."""
    output_dir.mkdir(parents=True, exist_ok=True)

    maze = generate_maze_from_config(config, seed)
    validation = validate_maze(
        maze,
        safety_radius_m=float(config["robot"]["safety_radius_m"]),
        min_corridor_width_m=float(config["maze"]["min_corridor_width_m"]),
    )
    raise_for_invalid(validation)

    base_model_path = _base_model_path(config, project_root)
    if not base_model_path.exists():
        raise FileNotFoundError(
            f"Base G1 model XML does not exist: {base_model_path}. "
            "Run `make fetch-lucky-g1-policy` for the default Lucky model, or "
            "`git submodule update --init --recursive` if you are using the legacy Menagerie model."
        )

    world_xml_path = output_dir / f"world_seed-{seed}.xml"
    summary_json_path = output_dir / f"world_seed-{seed}_summary.json"
    topdown_svg_path = output_dir / f"world_seed-{seed}_topdown.svg"

    tree = _load_base_model_tree(base_model_path)
    _append_world_geometry(tree, maze, config)
    _place_stand_keyframe_at_start(tree, maze, config)
    tree.write(world_xml_path, encoding="utf-8", xml_declaration=True)

    save_svg(maze, topdown_svg_path, validation.path, cell_px=48)

    start_x, start_y = cell_to_world_xy(maze, maze.spec.start_cell)
    goal_x, goal_y = cell_to_world_xy(maze, maze.spec.goal_cell)
    wall_count = int(np.count_nonzero(maze.grid == WALL))
    free_count = int(maze.grid.size - wall_count)

    result = WorldBuildResult(
        seed=int(seed),
        model_xml_path=str(world_xml_path),
        summary_json_path=str(summary_json_path),
        topdown_svg_path=str(topdown_svg_path),
        base_model_xml_path=str(base_model_path),
        wall_count=wall_count,
        free_count=free_count,
        start_cell=maze.spec.start_cell,
        goal_cell=maze.spec.goal_cell,
        start_world_xyz=(start_x, start_y, _robot_base_height(config)),
        goal_world_xyz=(goal_x, goal_y, 0.0),
        coordinate_convention=COORDINATE_CONVENTION,
    )
    summary_json_path.write_text(json.dumps(result.to_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def _base_model_path(config: dict[str, Any], project_root: Path) -> Path:
    raw_path = config.get("robot", {}).get("base_model_xml_path")
    if raw_path is None:
        raw_path = "assets/mujoco_menagerie/unitree_g1/g1.xml"
    return resolve_project_path(raw_path, project_root)


def _load_base_model_tree(base_model_path: Path) -> ET.ElementTree:
    tree = ET.parse(base_model_path)
    root = tree.getroot()
    root.set("model", "robotics_maze_g1_world")

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("angle", "radian")
    compiler.set("meshdir", str(base_model_path.parent / "assets"))

    return tree


def _append_world_geometry(tree: ET.ElementTree, maze: Maze, config: dict[str, Any]) -> None:
    root = tree.getroot()
    _set_world_view_defaults(root, maze)
    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")

    cell_size = float(config["maze"]["cell_size_m"])
    wall_height = float(config["maze"]["wall_height_m"])
    half_cell = cell_size / 2.0
    half_height = wall_height / 2.0
    floor_size_x = maze.spec.width_cells * cell_size / 2.0
    floor_size_y = maze.spec.height_cells * cell_size / 2.0

    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "maze_floor",
            "type": "plane",
            "pos": "0 0 0",
            "size": f"{floor_size_x:.6g} {floor_size_y:.6g} 0.05",
            "rgba": "0.82 0.86 0.88 1",
        },
    )

    for row in range(maze.spec.height_cells):
        for col in range(maze.spec.width_cells):
            if maze.grid[row, col] != WALL:
                continue
            x, y = cell_to_world_xy(maze, (row, col))
            ET.SubElement(
                worldbody,
                "geom",
                {
                    "name": f"maze_wall_{row}_{col}",
                    "type": "box",
                    "pos": f"{x:.6g} {y:.6g} {half_height:.6g}",
                    "size": f"{half_cell:.6g} {half_cell:.6g} {half_height:.6g}",
                    "rgba": "0.16 0.18 0.22 1",
                    "friction": "0.8 0.1 0.1",
                },
            )

    _append_marker(worldbody, maze, maze.spec.start_cell, "maze_start_marker", "0.10 0.70 0.25 0.8")
    _append_marker(worldbody, maze, maze.spec.goal_cell, "maze_goal_marker", "0.90 0.20 0.18 0.8")


def _set_world_view_defaults(root: ET.Element, maze: Maze) -> None:
    extent = max(maze.spec.width_cells, maze.spec.height_cells) * maze.spec.cell_size_m * 0.75

    statistic = root.find("statistic")
    if statistic is None:
        statistic = ET.Element("statistic")
        root.insert(1, statistic)
    statistic.set("center", "0 0 1")
    statistic.set("extent", f"{extent:.6g}")

    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    global_visual = visual.find("global")
    if global_visual is None:
        global_visual = ET.SubElement(visual, "global")
    global_visual.set("azimuth", "135")
    global_visual.set("elevation", "-55")


def _append_marker(worldbody: ET.Element, maze: Maze, cell: Cell, name: str, rgba: str) -> None:
    x, y = cell_to_world_xy(maze, cell)
    radius = maze.spec.cell_size_m * 0.28
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": name,
            "type": "cylinder",
            "pos": f"{x:.6g} {y:.6g} 0.015",
            "size": f"{radius:.6g} 0.015",
            "rgba": rgba,
            "contype": "0",
            "conaffinity": "0",
        },
    )


def _place_stand_keyframe_at_start(tree: ET.ElementTree, maze: Maze, config: dict[str, Any]) -> None:
    root = tree.getroot()
    keyframe_name = config.get("robot", {}).get("initial_keyframe", "stand")
    if not keyframe_name:
        return
    keyframe = root.find("keyframe")
    if keyframe is None:
        keyframe = ET.SubElement(root, "keyframe")

    key = None
    for candidate in keyframe.findall("key"):
        if candidate.get("name") == keyframe_name:
            key = candidate
            break
    if key is None:
        key = ET.SubElement(keyframe, "key", {"name": str(keyframe_name)})

    qpos_values = _float_list(key.get("qpos", ""))
    if len(qpos_values) < 7:
        qpos_values = [0.0, 0.0, _robot_base_height(config), 1.0, 0.0, 0.0, 0.0]

    start_x, start_y = cell_to_world_xy(maze, maze.spec.start_cell)
    qpos_values[0] = start_x
    qpos_values[1] = start_y
    qpos_values[2] = _robot_base_height(config)
    key.set("qpos", _format_float_list(qpos_values))


def _float_list(raw: str) -> list[float]:
    if not raw.strip():
        return []
    return [float(value) for value in raw.split()]


def _format_float_list(values: list[float]) -> str:
    return " ".join(f"{value:.8g}" for value in values)


def _robot_base_height(config: dict[str, Any]) -> float:
    return float(config.get("robot", {}).get("initial_base_height_m", 0.79))
