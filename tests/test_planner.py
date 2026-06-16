import numpy as np
import pytest

from maze.generator import generate_maze_from_config
from maze.grid import FREE, WALL, Maze, MazeSpec
from nav.planner import PlannerError, astar_path, cells_to_waypoints, inflate_obstacles, plan_oracle_path
from sim.config import load_config


def make_test_maze(grid: list[list[int]], start=(0, 0), goal=(0, 0), cell_size_m=1.0) -> Maze:
    array = np.array(grid, dtype=np.uint8)
    return Maze(
        spec=MazeSpec(
            width_cells=array.shape[1],
            height_cells=array.shape[0],
            cell_size_m=cell_size_m,
            seed=999,
            start_cell=start,
            goal_cell=goal,
        ),
        grid=array,
    )


def test_astar_finds_manhattan_path_on_empty_grid():
    grid = np.zeros((3, 3), dtype=np.uint8)

    path = astar_path(grid, (0, 0), (2, 2))

    assert path[0] == (0, 0)
    assert path[-1] == (2, 2)
    assert len(path) == 5


def test_astar_fails_cleanly_on_blocked_grid():
    grid = np.array(
        [
            [FREE, WALL, FREE],
            [FREE, WALL, FREE],
            [FREE, WALL, FREE],
        ],
        dtype=np.uint8,
    )

    with pytest.raises(PlannerError, match="No oracle path exists"):
        astar_path(grid, (0, 0), (0, 2))


def test_default_safety_radius_does_not_erase_one_meter_corridors():
    maze = make_test_maze(
        [
            [WALL, WALL, WALL],
            [WALL, FREE, WALL],
            [WALL, WALL, WALL],
        ],
        start=(1, 1),
        goal=(1, 1),
    )

    inflated = inflate_obstacles(maze, safety_radius_m=0.45)

    assert inflated[1, 1] == FREE


def test_inflation_blocks_cells_when_safety_radius_exceeds_clearance():
    maze = make_test_maze(
        [
            [WALL, WALL, WALL],
            [WALL, FREE, WALL],
            [WALL, WALL, WALL],
        ],
        start=(1, 1),
        goal=(1, 1),
    )

    with pytest.raises(PlannerError, match="start cell"):
        inflate_obstacles(maze, safety_radius_m=0.51)


def test_oracle_planner_finds_path_for_generated_maze():
    config = load_config("configs/default.yaml")
    maze = generate_maze_from_config(config, seed=123)

    plan = plan_oracle_path(maze, safety_radius_m=float(config["robot"]["safety_radius_m"]))

    assert plan.mode == "oracle"
    assert plan.path_cells[0] == maze.spec.start_cell
    assert plan.path_cells[-1] == maze.spec.goal_cell
    assert len(plan.waypoints_xyz) == len(plan.path_cells)


def test_waypoints_use_world_builder_coordinate_convention():
    maze = make_test_maze(
        [
            [FREE, FREE, FREE],
            [FREE, FREE, FREE],
            [FREE, FREE, FREE],
        ],
        start=(0, 0),
        goal=(2, 2),
    )

    waypoints = cells_to_waypoints(maze, [(0, 0), (1, 1), (2, 2)], z_m=0.0)

    assert waypoints == [(-1.0, 1.0, 0.0), (0.0, 0.0, 0.0), (1.0, -1.0, 0.0)]


def test_waypoint_conversion_rejects_wall_cells():
    maze = make_test_maze(
        [
            [FREE, WALL],
            [FREE, FREE],
        ],
        start=(0, 0),
        goal=(1, 1),
    )

    with pytest.raises(PlannerError, match="not free"):
        cells_to_waypoints(maze, [(0, 0), (0, 1)])
