"""Oracle maze planner for generated occupancy-grid mazes."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
from math import ceil

import numpy as np

from maze.grid import FREE, WALL, Cell, Maze, is_inside, neighbors_4, physical_corridor_width_m
from sim.world_builder import cell_to_world_xy


class PlanningError(RuntimeError):
    """Raised when the oracle planner cannot produce a valid path."""


@dataclass(frozen=True)
class OraclePlan:
    """Cell and world-coordinate path for oracle/debug navigation."""

    cells: list[Cell]
    waypoints: list[tuple[float, float, float]]


def plan_oracle_path(
    maze: Maze,
    *,
    safety_radius_m: float = 0.0,
    simplify: bool = True,
    planner: str = "bfs",
    turn_penalty_cost: float = 0.0,
) -> OraclePlan:
    """Plan a 4-connected oracle path through the maze grid."""
    inflated = inflate_obstacles(maze, safety_radius_m=safety_radius_m)
    if planner == "bfs":
        cells = _bfs_path(inflated, maze.spec.start_cell, maze.spec.goal_cell)
    elif planner == "heading_astar":
        cells = _heading_astar_path(
            inflated,
            maze.spec.start_cell,
            maze.spec.goal_cell,
            turn_penalty_cost=float(turn_penalty_cost),
        )
    else:
        raise PlanningError(f"Unsupported oracle planner: {planner}")
    if cells is None:
        raise PlanningError("No oracle path exists after obstacle inflation.")
    if simplify:
        cells = simplify_cell_path(cells)
    return OraclePlan(cells=cells, waypoints=cells_to_waypoints(maze, cells))


def inflate_obstacles(maze: Maze, *, safety_radius_m: float) -> np.ndarray:
    """Inflate walls only when the radius exceeds half a maze cell."""
    grid = np.array(maze.grid, copy=True)
    corridor_width = physical_corridor_width_m(maze.spec)
    clearance_over_half_cell = max(0.0, float(safety_radius_m) - corridor_width / 2.0)
    radius_cells = int(ceil(clearance_over_half_cell / corridor_width))
    if radius_cells <= 0:
        return grid

    inflated = np.array(grid, copy=True)
    wall_rows, wall_cols = np.where(grid == WALL)
    height, width = grid.shape
    for row, col in zip(wall_rows, wall_cols):
        for rr in range(row - radius_cells, row + radius_cells + 1):
            for cc in range(col - radius_cells, col + radius_cells + 1):
                if is_inside((rr, cc), height, width):
                    inflated[rr, cc] = WALL
    inflated[maze.spec.start_cell] = FREE
    inflated[maze.spec.goal_cell] = FREE
    return inflated


def cells_to_waypoints(maze: Maze, cells: list[Cell]) -> list[tuple[float, float, float]]:
    """Convert path cells to MuJoCo world waypoints centered in cells."""
    return [(x, y, 0.0) for x, y in (cell_to_world_xy(maze, cell) for cell in cells)]


def simplify_cell_path(cells: list[Cell]) -> list[Cell]:
    """Keep endpoints and turn cells to reduce stop-and-go waypoint chatter."""
    if len(cells) <= 2:
        return list(cells)

    simplified = [cells[0]]
    prev_dir = _cell_delta(cells[0], cells[1])
    for index in range(1, len(cells) - 1):
        next_dir = _cell_delta(cells[index], cells[index + 1])
        if next_dir != prev_dir:
            simplified.append(cells[index])
        prev_dir = next_dir
    simplified.append(cells[-1])
    return simplified


def _bfs_path(grid: np.ndarray, start: Cell, goal: Cell) -> list[Cell] | None:
    height, width = grid.shape
    if not is_inside(start, height, width) or not is_inside(goal, height, width):
        return None
    if grid[start] != FREE or grid[goal] != FREE:
        return None

    queue: deque[Cell] = deque([start])
    parents: dict[Cell, Cell | None] = {start: None}
    while queue:
        current = queue.popleft()
        if current == goal:
            return _reconstruct(parents, goal)
        for neighbor in _ordered_neighbors(current, start):
            if neighbor in parents:
                continue
            if not is_inside(neighbor, height, width):
                continue
            if grid[neighbor] != FREE:
                continue
            parents[neighbor] = current
            queue.append(neighbor)
    return None


def _heading_astar_path(
    grid: np.ndarray,
    start: Cell,
    goal: Cell,
    *,
    turn_penalty_cost: float,
) -> list[Cell] | None:
    height, width = grid.shape
    if not is_inside(start, height, width) or not is_inside(goal, height, width):
        return None
    if grid[start] != FREE or grid[goal] != FREE:
        return None

    start_heading = (0, 0)
    start_state = (start, start_heading)
    parents: dict[tuple[Cell, Cell], tuple[Cell, Cell] | None] = {start_state: None}
    costs: dict[tuple[Cell, Cell], float] = {start_state: 0.0}
    queue: list[tuple[float, float, int, tuple[Cell, Cell]]] = []
    counter = 0
    heapq.heappush(queue, (_manhattan(start, goal), 0.0, counter, start_state))

    best_goal_state: tuple[Cell, Cell] | None = None
    while queue:
        _, current_cost, _, current_state = heapq.heappop(queue)
        current_cell, current_heading = current_state
        if current_cost > costs[current_state]:
            continue
        if current_cell == goal:
            best_goal_state = current_state
            break

        for neighbor in _ordered_neighbors(current_cell, start):
            if not is_inside(neighbor, height, width):
                continue
            if grid[neighbor] != FREE:
                continue
            next_heading = _cell_delta(current_cell, neighbor)
            turn_cost = 0.0
            if current_heading != (0, 0) and next_heading != current_heading:
                turn_cost = max(0.0, turn_penalty_cost)
            next_state = (neighbor, next_heading)
            next_cost = current_cost + 1.0 + turn_cost
            if next_cost >= costs.get(next_state, float("inf")):
                continue
            costs[next_state] = next_cost
            parents[next_state] = current_state
            counter += 1
            priority = next_cost + _manhattan(neighbor, goal)
            heapq.heappush(queue, (priority, next_cost, counter, next_state))

    if best_goal_state is None:
        return None
    return _reconstruct_heading_path(parents, best_goal_state)


def _reconstruct_heading_path(
    parents: dict[tuple[Cell, Cell], tuple[Cell, Cell] | None],
    goal_state: tuple[Cell, Cell],
) -> list[Cell]:
    path: list[Cell] = []
    current: tuple[Cell, Cell] | None = goal_state
    while current is not None:
        path.append(current[0])
        current = parents[current]
    path.reverse()
    return path


def _reconstruct(parents: dict[Cell, Cell | None], goal: Cell) -> list[Cell]:
    path: list[Cell] = []
    current: Cell | None = goal
    while current is not None:
        path.append(current)
        current = parents[current]
    path.reverse()
    return path


def _cell_delta(a: Cell, b: Cell) -> Cell:
    return b[0] - a[0], b[1] - a[1]


def _manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _ordered_neighbors(cell: Cell, start: Cell) -> list[Cell]:
    neighbors = list(neighbors_4(cell))
    neighbors.sort(key=lambda neighbor: _neighbor_priority(cell, neighbor, start))
    return neighbors


def _neighbor_priority(current: Cell, neighbor: Cell, start: Cell) -> tuple[int, int, int, int, int]:
    delta_row, delta_col = _cell_delta(current, neighbor)
    away_gain = _manhattan(neighbor, start) - _manhattan(current, start)
    positive_x = delta_col
    return -away_gain, -positive_x, abs(delta_row), neighbor[0], neighbor[1]
