"""SLAM-map corridor recentering helpers.

The selector intentionally uses only an occupancy grid and the robot pose. It
does not read the generated maze, oracle path, or seed.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from collections.abc import Sequence


@dataclass(frozen=True)
class CorridorCenterGoal:
    x: float
    y: float
    clearance_m: float
    distance_m: float


def select_corridor_center_goal(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    current_xy: tuple[float, float],
    *,
    search_radius_m: float = 4.0,
    candidate_step_m: float = 0.20,
    clearance_radius_m: float = 1.60,
    clearance_sample_step_m: float = 0.15,
    min_clearance_m: float = 0.75,
    min_step_m: float = 0.20,
) -> CorridorCenterGoal | None:
    if width <= 0 or height <= 0 or resolution <= 0.0 or not data:
        return None

    current_col = math.floor((current_xy[0] - origin_x) / resolution)
    current_row = math.floor((current_xy[1] - origin_y) / resolution)
    radius_cells = max(1, int(math.ceil(max(0.0, search_radius_m) / resolution)))
    step_cells = max(1, int(round(max(resolution, candidate_step_m) / resolution)))

    candidates: list[CorridorCenterGoal] = []
    for row in range(max(0, current_row - radius_cells), min(height, current_row + radius_cells + 1), step_cells):
        y = origin_y + (row + 0.5) * resolution
        for col in range(max(0, current_col - radius_cells), min(width, current_col + radius_cells + 1), step_cells):
            x = origin_x + (col + 0.5) * resolution
            distance = math.hypot(x - current_xy[0], y - current_xy[1])
            if distance > search_radius_m or distance < min_step_m:
                continue
            if _grid_value(data, width, height, row, col) != 0:
                continue
            clearance = _occupied_clearance(
                data,
                width,
                height,
                resolution,
                origin_x,
                origin_y,
                x,
                y,
                clearance_radius_m,
                clearance_sample_step_m,
            )
            if clearance >= min_clearance_m:
                candidates.append(CorridorCenterGoal(x=x, y=y, clearance_m=clearance, distance_m=distance))

    if not candidates:
        return None

    max_clearance = max(candidate.clearance_m for candidate in candidates)
    clearance_floor = max(min_clearance_m, max_clearance * 0.85)
    eligible = [candidate for candidate in candidates if candidate.clearance_m >= clearance_floor]
    eligible.sort(key=lambda candidate: (candidate.distance_m, -candidate.clearance_m))
    return eligible[0]


def _occupied_clearance(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    x: float,
    y: float,
    limit: float,
    step: float,
) -> float:
    step = max(resolution, step, 0.05)
    rings = max(1, int(math.ceil(max(0.0, limit) / step)))
    for ring in range(1, rings + 1):
        radius = min(limit, ring * step)
        samples = max(8, int(math.ceil(2.0 * math.pi * radius / step)))
        for index in range(samples):
            angle = 2.0 * math.pi * index / samples
            value = _value_at_world(
                data,
                width,
                height,
                resolution,
                origin_x,
                origin_y,
                x + math.cos(angle) * radius,
                y + math.sin(angle) * radius,
            )
            if value is not None and value >= 65:
                return radius
    return max(0.0, limit)


def _value_at_world(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    x: float,
    y: float,
) -> int | None:
    col = math.floor((x - origin_x) / resolution)
    row = math.floor((y - origin_y) / resolution)
    if col < 0 or row < 0 or col >= width or row >= height:
        return None
    return _grid_value(data, width, height, row, col)


def _grid_value(data: Sequence[int], width: int, height: int, row: int, col: int) -> int | None:
    if col < 0 or row < 0 or col >= width or row >= height:
        return None
    return int(data[int(row) * int(width) + int(col)])
