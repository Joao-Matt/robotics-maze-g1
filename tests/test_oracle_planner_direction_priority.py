import importlib
import sys

from maze.grid import FREE, WALL, Maze, MazeSpec


def _numpy_and_planner():
    existing = sys.modules.get("numpy")
    if existing is not None and getattr(existing, "__spec__", None) is None:
        del sys.modules["numpy"]
    numpy = importlib.import_module("numpy")
    planner = importlib.import_module("nav.planner")
    planner.np = numpy
    return numpy, planner.plan_oracle_path


def test_oracle_planner_prefers_positive_x_branch_on_equal_detour():
    np, plan_oracle_path = _numpy_and_planner()
    grid = np.full((7, 7), WALL, dtype=np.uint8)
    for cell in (
        (3, 3),
        (3, 2),
        (2, 2),
        (1, 2),
        (1, 3),
        (3, 4),
        (2, 4),
        (1, 4),
    ):
        grid[cell] = FREE
    maze = Maze(
        spec=MazeSpec(
            width_cells=7,
            height_cells=7,
            cell_size_m=1.0,
            seed=1,
            start_cell=(3, 3),
            goal_cell=(1, 3),
        ),
        grid=grid,
    )

    plan = plan_oracle_path(maze, simplify=False)

    assert plan.cells[1] == (3, 4)
