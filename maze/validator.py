"""Validation and path checks for occupancy-grid mazes."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from maze.grid import FREE, Maze, Cell, is_inside, neighbors_4, physical_cell_width_m, physical_cell_length_m


@dataclass(frozen=True)
class ValidationResult:
    """Result of validating a generated maze."""

    is_valid: bool
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    path: list[Cell] | None = None


def validate_maze(
    maze: Maze,
    *,
    min_path_cells: int = 4,
    safety_radius_m: float | None = None,
    min_corridor_width_m: float | None = None,
    max_corridor_width_m: float | None = None,
) -> ValidationResult:
    """Validate maze dimensions, endpoints, physical width, and BFS solvability."""
    errors: list[str] = []
    warnings: list[str] = []

    expected_shape = (maze.spec.height_cells, maze.spec.width_cells)
    if maze.grid.shape != expected_shape:
        errors.append(f"Grid shape {maze.grid.shape} does not match spec {expected_shape}.")

    if maze.grid.ndim != 2:
        errors.append("Grid must be a 2D array.")
    elif not np.isin(maze.grid, [0, 1]).all():
        errors.append("Grid must contain only 0/free and 1/wall values.")

    corridor_width = physical_cell_width_m(maze.spec)
    cell_length = physical_cell_length_m(maze.spec)
    if maze.spec.cell_size_m <= 0 or corridor_width <= 0 or cell_length <= 0:
        errors.append("cell dimensions must be positive.")

    if min_corridor_width_m is not None and corridor_width < min_corridor_width_m:
        errors.append(
            f"cell_width_m={corridor_width} is narrower than "
            f"min_corridor_width_m={min_corridor_width_m}."
        )

    if max_corridor_width_m is not None and corridor_width > max_corridor_width_m:
        errors.append(
            f"cell_width_m={corridor_width} is wider than "
            f"max_corridor_width_m={max_corridor_width_m}."
        )

    if safety_radius_m is not None and corridor_width < 2.0 * safety_radius_m:
        errors.append(
            f"cell_width_m={corridor_width} is narrower than "
            f"2 * safety_radius_m={2.0 * safety_radius_m}."
        )

    _validate_endpoint("start", maze.spec.start_cell, maze, errors)
    _validate_endpoint("goal", maze.spec.goal_cell, maze, errors)

    path = None
    if not errors:
        path = find_path(maze)
        if path is None:
            errors.append("No BFS path exists from start to goal.")
        elif len(path) < min_path_cells:
            errors.append(f"Path is too short to be meaningful: {len(path)} cells.")

    return ValidationResult(is_valid=not errors, errors=errors, warnings=warnings, path=path)


def find_path(maze: Maze) -> list[Cell] | None:
    """Return a BFS path from start to goal, or None when unreachable."""
    start = maze.spec.start_cell
    goal = maze.spec.goal_cell
    height, width = maze.grid.shape

    queue: deque[Cell] = deque([start])
    parents: dict[Cell, Cell | None] = {start: None}

    while queue:
        current = queue.popleft()
        if current == goal:
            return _reconstruct_path(parents, goal)

        for neighbor in neighbors_4(current):
            if neighbor in parents:
                continue
            if not is_inside(neighbor, height, width):
                continue
            if maze.grid[neighbor] != FREE:
                continue
            parents[neighbor] = current
            queue.append(neighbor)

    return None


def raise_for_invalid(result: ValidationResult) -> None:
    """Raise ValueError when a ValidationResult contains errors."""
    if result.is_valid:
        return
    raise ValueError("; ".join(result.errors))


def _validate_endpoint(name: str, cell: Cell, maze: Maze, errors: list[str]) -> None:
    height, width = maze.grid.shape
    if not is_inside(cell, height, width):
        errors.append(f"{name} cell is outside bounds: {cell}.")
        return
    if maze.grid[cell] != FREE:
        errors.append(f"{name} cell is not free: {cell}.")


def _reconstruct_path(parents: dict[Cell, Cell | None], goal: Cell) -> list[Cell]:
    path: list[Cell] = []
    current: Cell | None = goal
    while current is not None:
        path.append(current)
        current = parents[current]
    path.reverse()
    return path
