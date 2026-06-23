"""Resolve robot spawn orientation for generated maze runs."""

from __future__ import annotations

import math
from typing import Any

from maze.generator import generate_maze_from_config
from maze.validator import validate_maze
from sim.world_builder import cell_to_world_xy


AUTO_SPAWN_YAW_VALUES = {"", "auto", "corridor", "first_corridor", "towards_corridor"}


def resolve_initial_spawn_yaw(config: dict[str, Any], seed: int, *, default_auto: bool = True) -> float:
    """Return the configured spawn yaw, or face the first open corridor."""
    raw = config.get("nav2_navigation", {}).get("initial_spawn_yaw_rad")
    if raw is None:
        return yaw_for_first_corridor(config, seed) if default_auto else 0.0
    if isinstance(raw, str):
        value = raw.strip().lower()
        if value in AUTO_SPAWN_YAW_VALUES:
            return yaw_for_first_corridor(config, seed)
        try:
            yaw = float(value)
        except ValueError as exc:
            raise ValueError(f"Unsupported initial_spawn_yaw_rad value: {raw!r}") from exc
    else:
        yaw = float(raw)
    if math.isfinite(yaw):
        return yaw
    return yaw_for_first_corridor(config, seed) if default_auto else 0.0


def yaw_for_first_corridor(config: dict[str, Any], seed: int) -> float:
    """Face from the start cell toward the next validated corridor cell."""
    maze = generate_maze_from_config(config, seed)
    path = validate_maze(
        maze,
        safety_radius_m=float(config["robot"]["safety_radius_m"]),
        min_corridor_width_m=float(config["maze"]["min_corridor_width_m"]),
        max_corridor_width_m=(
            float(config["maze"]["max_corridor_width_m"])
            if "max_corridor_width_m" in config["maze"]
            else None
        ),
    ).path
    if len(path) < 2:
        return 0.0
    start = cell_to_world_xy(maze, path[0])
    following = cell_to_world_xy(maze, path[1])
    return math.atan2(following[1] - start[1], following[0] - start[0])
