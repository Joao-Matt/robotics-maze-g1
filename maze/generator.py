"""Deterministic seeded maze generation."""

from __future__ import annotations

import random

import numpy as np

from maze.grid import FREE, WALL, Maze, MazeSpec, Cell, spec_from_config, validate_spec


def generate_maze(
    seed: int,
    width_cells: int = 15,
    height_cells: int = 15,
    cell_size_m: float = 1.0,
) -> Maze:
    """Generate a deterministic perfect maze using randomized DFS."""
    spec = MazeSpec(
        width_cells=int(width_cells),
        height_cells=int(height_cells),
        cell_size_m=float(cell_size_m),
        seed=int(seed),
        start_cell=(1, 1),
        goal_cell=(int(height_cells) - 2, int(width_cells) - 2),
    )
    return generate_maze_from_spec(spec)


def generate_maze_from_config(config: dict, seed: int) -> Maze:
    """Generate a maze from project config values and an explicit seed."""
    return generate_maze_from_spec(spec_from_config(config, seed))


def generate_maze_from_spec(spec: MazeSpec) -> Maze:
    """Generate a deterministic maze for a validated MazeSpec."""
    validate_spec(spec)

    rng = random.Random(spec.seed)
    grid = np.full((spec.height_cells, spec.width_cells), WALL, dtype=np.uint8)

    start = spec.start_cell
    grid[start] = FREE
    visited: set[Cell] = {start}
    stack: list[Cell] = [start]
    carve_offsets = [(-2, 0), (2, 0), (0, -2), (0, 2)]

    while stack:
        row, col = stack[-1]
        candidates: list[Cell] = []
        for d_row, d_col in carve_offsets:
            next_cell = row + d_row, col + d_col
            if _is_carvable(next_cell, spec) and next_cell not in visited:
                candidates.append(next_cell)

        if not candidates:
            stack.pop()
            continue

        next_row, next_col = rng.choice(candidates)
        wall_between = (row + next_row) // 2, (col + next_col) // 2
        grid[wall_between] = FREE
        grid[next_row, next_col] = FREE
        visited.add((next_row, next_col))
        stack.append((next_row, next_col))

    return Maze(spec=spec, grid=grid)


def _is_carvable(cell: Cell, spec: MazeSpec) -> bool:
    row, col = cell
    return 1 <= row < spec.height_cells - 1 and 1 <= col < spec.width_cells - 1
