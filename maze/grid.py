"""Grid data structures and helpers for generated mazes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, TypeAlias

import numpy as np

FREE: int = 0
WALL: int = 1
Cell: TypeAlias = tuple[int, int]


@dataclass(frozen=True)
class MazeSpec:
    """Configuration for a seeded occupancy-grid maze."""

    width_cells: int
    height_cells: int
    cell_size_m: float
    seed: int
    start_cell: Cell
    goal_cell: Cell
    cell_width_m: float | None = None
    cell_length_m: float | None = None


@dataclass(frozen=True)
class Maze:
    """A maze occupancy grid where 0 is free and 1 is wall."""

    spec: MazeSpec
    grid: np.ndarray


def spec_from_config(config: dict, seed: int) -> MazeSpec:
    """Create a MazeSpec from the project YAML config."""
    maze_config = config["maze"]
    height = int(maze_config["height_cells"])
    width = int(maze_config["width_cells"])

    return MazeSpec(
        width_cells=width,
        height_cells=height,
        cell_size_m=float(maze_config["cell_size_m"]),
        seed=int(seed),
        start_cell=(1, 1),
        goal_cell=(height - 2, width - 2),
        cell_width_m=float(maze_config.get("cell_width_m", maze_config["cell_size_m"])),
        cell_length_m=float(maze_config.get("cell_length_m", maze_config["cell_size_m"])),
    )


def validate_spec(spec: MazeSpec) -> None:
    """Raise ValueError if a spec cannot produce a proper odd-grid maze."""
    if spec.width_cells < 5 or spec.height_cells < 5:
        raise ValueError("Maze width and height must each be at least 5 cells.")
    if spec.width_cells % 2 == 0 or spec.height_cells % 2 == 0:
        raise ValueError("Maze width and height must be odd so borders and corridors align.")
    if spec.cell_size_m <= 0 or physical_cell_width_m(spec) <= 0 or physical_cell_length_m(spec) <= 0:
        raise ValueError("Maze cell dimensions must be positive.")
    if not np.isclose(physical_cell_width_m(spec), physical_cell_length_m(spec)):
        raise ValueError("Maze physical cells must be square: cell_width_m must equal cell_length_m.")
    for name, cell in (("start_cell", spec.start_cell), ("goal_cell", spec.goal_cell)):
        if not is_inside(cell, spec.height_cells, spec.width_cells):
            raise ValueError(f"{name} is outside maze bounds: {cell}")


def is_inside(cell: Cell, height: int, width: int) -> bool:
    row, col = cell
    return 0 <= row < height and 0 <= col < width


def physical_cell_width_m(spec: MazeSpec) -> float:
    return float(spec.cell_width_m if spec.cell_width_m is not None else spec.cell_size_m)


def physical_cell_length_m(spec: MazeSpec) -> float:
    return float(spec.cell_length_m if spec.cell_length_m is not None else spec.cell_size_m)


def physical_corridor_width_m(spec: MazeSpec) -> float:
    """Return the configured width across the corridor."""
    return physical_cell_width_m(spec)


def physical_path_step_m(spec: MazeSpec) -> float:
    """Return the configured distance advanced by one grid step along a path."""
    return physical_cell_length_m(spec)


def neighbors_4(cell: Cell) -> Iterable[Cell]:
    row, col = cell
    yield row - 1, col
    yield row + 1, col
    yield row, col - 1
    yield row, col + 1
