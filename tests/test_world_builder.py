from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from maze.generator import generate_maze_from_config
from maze.grid import WALL
from sim.config import load_config
from sim.world_builder import build_maze_world, cell_to_world_xy


PROJECT_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def default_config():
    return load_config(PROJECT_ROOT / "configs" / "default.yaml")


def test_cell_to_world_uses_center_origin(default_config):
    maze = generate_maze_from_config(default_config, seed=123)

    assert cell_to_world_xy(maze, (7, 7)) == (0.0, 0.0)
    assert cell_to_world_xy(maze, (0, 0)) == (-7.0, 7.0)
    assert cell_to_world_xy(maze, (14, 14)) == (7.0, -7.0)


def test_default_start_and_goal_world_coordinates(default_config):
    maze = generate_maze_from_config(default_config, seed=123)

    assert cell_to_world_xy(maze, maze.spec.start_cell) == (-6.0, 6.0)
    assert cell_to_world_xy(maze, maze.spec.goal_cell) == (6.0, -6.0)


def test_build_world_writes_expected_artifacts_and_wall_count(default_config, tmp_path):
    result = build_maze_world(default_config, seed=123, output_dir=tmp_path)
    maze = generate_maze_from_config(default_config, seed=123)
    tree = ET.parse(result.model_xml_path)
    wall_geoms = tree.findall(".//geom")
    wall_count = sum(1 for geom in wall_geoms if (geom.get("name") or "").startswith("maze_wall_"))

    assert Path(result.model_xml_path).exists()
    assert Path(result.summary_json_path).exists()
    assert Path(result.topdown_svg_path).exists()
    assert wall_count == int(np.count_nonzero(maze.grid == WALL))
    assert result.start_world_xyz == (-6.0, 6.0, 0.79)
    assert result.goal_world_xyz == (6.0, -6.0, 0.0)


def test_start_and_goal_markers_are_visual_only(default_config, tmp_path):
    result = build_maze_world(default_config, seed=123, output_dir=tmp_path)
    root = ET.parse(result.model_xml_path).getroot()

    for name in ("maze_start_marker", "maze_goal_marker"):
        marker = root.find(f".//geom[@name='{name}']")
        assert marker is not None
        assert marker.get("contype") == "0"
        assert marker.get("conaffinity") == "0"


def test_stand_keyframe_places_robot_at_start(default_config, tmp_path):
    result = build_maze_world(default_config, seed=123, output_dir=tmp_path)
    root = ET.parse(result.model_xml_path).getroot()
    key = root.find(".//key[@name='stand']")

    assert key is not None
    qpos = [float(value) for value in key.get("qpos", "").split()]
    assert qpos[:3] == [-6.0, 6.0, 0.79]


def test_generated_world_loads_in_mujoco_when_available(default_config, tmp_path):
    mujoco = pytest.importorskip("mujoco")
    result = build_maze_world(default_config, seed=123, output_dir=tmp_path)

    model = mujoco.MjModel.from_xml_path(result.model_xml_path)

    assert model.nq == 36
    assert model.nv == 35
    assert model.ngeom > 100
