import numpy as np
import pytest

from maze.generator import generate_maze_from_config
from maze.grid import FREE, WALL, Maze, MazeSpec
from nav.planner import PlanningError, cells_to_waypoints, inflate_obstacles, plan_oracle_path
from sim.config import load_config
from sim.world_builder import cell_to_world_xy


def test_oracle_planner_finds_path_for_generated_maze():
    config = load_config("configs/default.yaml")
    maze = generate_maze_from_config(config, seed=123)

    plan = plan_oracle_path(maze, safety_radius_m=config["robot"]["safety_radius_m"])

    assert plan.cells[0] == maze.spec.start_cell
    assert plan.cells[-1] == maze.spec.goal_cell
    assert len(plan.waypoints) == len(plan.cells)


def test_oracle_planner_fails_cleanly_for_blocked_grid():
    spec = MazeSpec(width_cells=5, height_cells=5, cell_size_m=1.0, seed=1, start_cell=(1, 1), goal_cell=(3, 3))
    grid = np.ones((5, 5), dtype=int)
    grid[spec.start_cell] = FREE
    grid[spec.goal_cell] = FREE
    maze = Maze(spec=spec, grid=grid)

    with pytest.raises(PlanningError, match="No oracle path"):
        plan_oracle_path(maze)


def test_waypoint_conversion_matches_world_builder():
    config = load_config("configs/default.yaml")
    maze = generate_maze_from_config(config, seed=123)
    cells = [maze.spec.start_cell, maze.spec.goal_cell]

    waypoints = cells_to_waypoints(maze, cells)

    assert waypoints[0][:2] == cell_to_world_xy(maze, maze.spec.start_cell)
    assert waypoints[1][:2] == cell_to_world_xy(maze, maze.spec.goal_cell)
    assert waypoints[0][2] == 0.0


def test_default_safety_radius_does_not_close_one_cell_corridor():
    spec = MazeSpec(width_cells=5, height_cells=5, cell_size_m=1.0, seed=1, start_cell=(1, 1), goal_cell=(1, 3))
    grid = np.array(
        [
            [WALL, WALL, WALL, WALL, WALL],
            [WALL, FREE, FREE, FREE, WALL],
            [WALL, WALL, WALL, WALL, WALL],
            [WALL, WALL, WALL, WALL, WALL],
            [WALL, WALL, WALL, WALL, WALL],
        ]
    )
    maze = Maze(spec=spec, grid=grid)

    inflated = inflate_obstacles(maze, safety_radius_m=0.45)

    assert inflated[1, 2] == FREE
