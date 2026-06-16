"""Oracle/debug path planning on generated maze grids."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from heapq import heappop, heappush
from pathlib import Path
from typing import Any
import json
import math

import numpy as np

from maze.grid import FREE, WALL, Cell, Maze, neighbors_4, is_inside
from maze.visualization import save_svg
from sim.world_builder import cell_to_world_xy


class PlannerError(ValueError):
    """Raised when oracle planning cannot produce a valid path."""


@dataclass(frozen=True)
class PlanResult:
    """Oracle plan over the known maze grid."""

    mode: str
    seed: int
    path_cells: list[Cell]
    waypoints_xyz: list[tuple[float, float, float]]
    inflated_blocked_count: int
    safety_radius_m: float
    cell_size_m: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def plan_oracle_path(
    maze: Maze,
    *,
    safety_radius_m: float,
    waypoint_z_m: float = 0.0,
) -> PlanResult:
    """Plan a 4-connected A* path using the known generated maze grid."""
    inflated_grid = inflate_obstacles(maze, safety_radius_m=safety_radius_m)
    path = astar_path(inflated_grid, maze.spec.start_cell, maze.spec.goal_cell)
    waypoints = cells_to_waypoints(maze, path, z_m=waypoint_z_m)
    return PlanResult(
        mode="oracle",
        seed=maze.spec.seed,
        path_cells=path,
        waypoints_xyz=waypoints,
        inflated_blocked_count=int(np.count_nonzero(inflated_grid == WALL)),
        safety_radius_m=float(safety_radius_m),
        cell_size_m=float(maze.spec.cell_size_m),
    )


def inflate_obstacles(maze: Maze, *, safety_radius_m: float) -> np.ndarray:
    """Inflate obstacle cells by continuous clearance from wall-cell boxes."""
    if safety_radius_m < 0:
        raise PlannerError(f"safety_radius_m must be non-negative, got {safety_radius_m}")

    inflated = np.array(maze.grid, copy=True)
    wall_cells = list(zip(*np.where(maze.grid == WALL)))
    cell_size = maze.spec.cell_size_m

    for row in range(maze.spec.height_cells):
        for col in range(maze.spec.width_cells):
            if maze.grid[row, col] == WALL:
                continue
            if _cell_center_too_close_to_wall(
                (row, col),
                wall_cells,
                cell_size_m=cell_size,
                safety_radius_m=safety_radius_m,
            ):
                inflated[row, col] = WALL

    start = maze.spec.start_cell
    goal = maze.spec.goal_cell
    if inflated[start] != FREE:
        raise PlannerError(f"Inflated obstacles block the start cell: {start}")
    if inflated[goal] != FREE:
        raise PlannerError(f"Inflated obstacles block the goal cell: {goal}")
    return inflated


def astar_path(grid: np.ndarray, start: Cell, goal: Cell) -> list[Cell]:
    """Return a 4-connected A* path using Manhattan distance."""
    if grid.ndim != 2:
        raise PlannerError("A* grid must be 2D.")
    height, width = grid.shape
    for name, cell in (("start", start), ("goal", goal)):
        if not is_inside(cell, height, width):
            raise PlannerError(f"{name} cell is outside grid bounds: {cell}")
        if grid[cell] != FREE:
            raise PlannerError(f"{name} cell is blocked: {cell}")

    open_heap: list[tuple[int, int, Cell]] = []
    heappush(open_heap, (_manhattan(start, goal), 0, start))
    came_from: dict[Cell, Cell | None] = {start: None}
    g_score: dict[Cell, int] = {start: 0}

    while open_heap:
        _, current_cost, current = heappop(open_heap)
        if current == goal:
            return _reconstruct_path(came_from, goal)
        if current_cost > g_score[current]:
            continue

        for neighbor in neighbors_4(current):
            if not is_inside(neighbor, height, width):
                continue
            if grid[neighbor] != FREE:
                continue
            tentative = g_score[current] + 1
            if tentative >= g_score.get(neighbor, math.inf):
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative
            priority = tentative + _manhattan(neighbor, goal)
            heappush(open_heap, (priority, tentative, neighbor))

    raise PlannerError(f"No oracle path exists from {start} to {goal}.")


def cells_to_waypoints(maze: Maze, path_cells: list[Cell], *, z_m: float = 0.0) -> list[tuple[float, float, float]]:
    """Convert path cells to MuJoCo world-coordinate waypoint centers."""
    waypoints: list[tuple[float, float, float]] = []
    for cell in path_cells:
        if maze.grid[cell] != FREE:
            raise PlannerError(f"Path cell is not free in original maze: {cell}")
        x, y = cell_to_world_xy(maze, cell)
        waypoints.append((x, y, float(z_m)))
    return waypoints


def save_plan_artifacts(
    maze: Maze,
    plan: PlanResult,
    *,
    output_dir: Path,
    cell_px: int = 48,
) -> tuple[Path, Path]:
    """Save visible SVG and JSON artifacts for an oracle plan."""
    output_dir.mkdir(parents=True, exist_ok=True)
    svg_path = output_dir / f"plan_seed-{maze.spec.seed}_oracle.svg"
    json_path = output_dir / f"plan_seed-{maze.spec.seed}_oracle.json"
    save_svg(maze, svg_path, plan.path_cells, cell_px=cell_px)
    json_path.write_text(json.dumps(plan.to_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return svg_path, json_path


def _cell_center_too_close_to_wall(
    cell: Cell,
    wall_cells: list[Cell],
    *,
    cell_size_m: float,
    safety_radius_m: float,
) -> bool:
    row, col = cell
    for wall_row, wall_col in wall_cells:
        dx_cells = max(abs(col - int(wall_col)) - 0.5, 0.0)
        dy_cells = max(abs(row - int(wall_row)) - 0.5, 0.0)
        clearance = math.hypot(dx_cells * cell_size_m, dy_cells * cell_size_m)
        if clearance < safety_radius_m:
            return True
    return False


def _manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _reconstruct_path(came_from: dict[Cell, Cell | None], goal: Cell) -> list[Cell]:
    path: list[Cell] = []
    current: Cell | None = goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path
