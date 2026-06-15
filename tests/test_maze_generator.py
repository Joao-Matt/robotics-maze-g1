import numpy as np
import pytest

from maze.generator import generate_maze
from maze.grid import FREE, WALL
from maze.validator import validate_maze
from maze.visualization import maze_to_ascii


def test_generate_maze_is_deterministic_for_same_seed():
    first = generate_maze(seed=123)
    second = generate_maze(seed=123)

    assert np.array_equal(first.grid, second.grid)
    assert first.spec == second.spec


def test_generate_maze_varies_across_different_seeds():
    first = generate_maze(seed=1)
    second = generate_maze(seed=2)

    assert not np.array_equal(first.grid, second.grid)


def test_generated_maze_start_and_goal_are_free():
    maze = generate_maze(seed=123)

    assert maze.grid[maze.spec.start_cell] == FREE
    assert maze.grid[maze.spec.goal_cell] == FREE


def test_generated_maze_uses_only_occupancy_values():
    maze = generate_maze(seed=123)

    assert set(np.unique(maze.grid)).issubset({FREE, WALL})


def test_generated_mazes_are_valid_for_twenty_seeds():
    for seed in range(20):
        maze = generate_maze(seed=seed)
        result = validate_maze(
            maze,
            safety_radius_m=0.45,
            min_corridor_width_m=1.0,
        )

        assert result.is_valid, result.errors
        assert result.path is not None
        assert result.path[0] == maze.spec.start_cell
        assert result.path[-1] == maze.spec.goal_cell


def test_validation_fails_for_blocked_goal():
    maze = generate_maze(seed=123)
    maze.grid[maze.spec.goal_cell] = WALL

    result = validate_maze(maze)

    assert not result.is_valid
    assert any("goal cell is not free" in error for error in result.errors)


def test_even_dimensions_are_rejected():
    with pytest.raises(ValueError, match="must be odd"):
        generate_maze(seed=123, width_cells=10, height_cells=10)


def test_ascii_visualization_marks_start_goal_and_path():
    maze = generate_maze(seed=123)
    result = validate_maze(maze)

    rendered = maze_to_ascii(maze, result.path)

    assert "S" in rendered
    assert "G" in rendered
    assert "." in rendered
