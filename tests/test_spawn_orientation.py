from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _numpy_unavailable() -> bool:
    try:
        return importlib.util.find_spec("numpy") is None
    except ValueError:
        return True


@unittest.skipIf(_numpy_unavailable(), "numpy unavailable")
class SpawnOrientationTest(unittest.TestCase):
    def test_explicit_numeric_yaw_is_preserved(self) -> None:
        from sim.config import load_config
        from sim.spawn_orientation import resolve_initial_spawn_yaw

        config = load_config(ROOT / "configs" / "default.yaml")
        config["nav2_navigation"]["initial_spawn_yaw_rad"] = -1.25

        self.assertAlmostEqual(resolve_initial_spawn_yaw(config, 123), -1.25)

    def test_auto_yaw_faces_first_validated_corridor_cell(self) -> None:
        from maze.generator import generate_maze_from_config
        from maze.validator import validate_maze
        from sim.config import load_config
        from sim.spawn_orientation import resolve_initial_spawn_yaw
        from sim.world_builder import cell_to_world_xy

        seed = 123
        config = load_config(ROOT / "configs" / "default.yaml")
        config["maze"]["cell_size_m"] = 4.0
        config["maze"]["cell_width_m"] = 4.0
        config["maze"]["cell_length_m"] = 4.0
        config["nav2_navigation"]["initial_spawn_yaw_rad"] = "auto"

        maze = generate_maze_from_config(config, seed)
        path = validate_maze(
            maze,
            safety_radius_m=float(config["robot"]["safety_radius_m"]),
            min_corridor_width_m=float(config["maze"]["min_corridor_width_m"]),
            max_corridor_width_m=float(config["maze"]["max_corridor_width_m"]),
        ).path
        start = cell_to_world_xy(maze, path[0])
        following = cell_to_world_xy(maze, path[1])
        expected = math.atan2(following[1] - start[1], following[0] - start[0])

        self.assertAlmostEqual(resolve_initial_spawn_yaw(config, seed), expected)


if __name__ == "__main__":
    unittest.main()
