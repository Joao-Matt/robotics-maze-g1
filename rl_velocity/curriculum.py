"""Curriculum maze fixtures for direct MuJoCo velocity-controller RL."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any

import numpy as np

from maze.generator import generate_maze_from_spec
from maze.grid import FREE, WALL, Cell, Maze, MazeSpec


RANDOM_STAGE_KINDS = {"full_small_maze", "larger_random_maze", "narrow_noisy_maze"}


@dataclass(frozen=True)
class StageSpec:
    """One curriculum stage definition."""

    name: str
    kind: str
    width_cells: int
    height_cells: int
    cell_size_m: float
    weight: float = 1.0
    scan_noise_std_m: float = 0.0
    action_noise_std: float = 0.0
    friction_range: tuple[float, float] = (0.8, 0.8)
    start_yaw_offset_rad: float = 0.0


def load_stage_specs(rl_config: dict[str, Any]) -> list[StageSpec]:
    """Parse stage specs from the RL config."""
    raw_stages = rl_config.get("curriculum", {}).get("stages", [])
    stages: list[StageSpec] = []
    for raw in raw_stages:
        if not isinstance(raw, dict):
            raise ValueError("Each curriculum stage must be a mapping.")
        friction = raw.get("friction_range", [0.8, 0.8])
        if len(friction) != 2:
            raise ValueError(f"friction_range for stage {raw.get('name')} must contain two values.")
        stages.append(
            StageSpec(
                name=str(raw["name"]),
                kind=str(raw.get("kind", raw["name"])),
                width_cells=_odd(raw.get("width_cells", 15)),
                height_cells=_odd(raw.get("height_cells", 15)),
                cell_size_m=float(raw.get("cell_size_m", 2.0)),
                weight=float(raw.get("weight", 1.0)),
                scan_noise_std_m=float(raw.get("scan_noise_std_m", 0.0)),
                action_noise_std=float(raw.get("action_noise_std", 0.0)),
                friction_range=(float(friction[0]), float(friction[1])),
                start_yaw_offset_rad=float(raw.get("start_yaw_offset_rad", 0.0)),
            )
        )
    return stages


def select_stage(stages: list[StageSpec], rng: np.random.Generator, requested_name: str | None = None) -> StageSpec:
    """Select a stage by name or by configured sampling weights."""
    if requested_name:
        for stage in stages:
            if stage.name == requested_name:
                return stage
        names = ", ".join(stage.name for stage in stages)
        raise ValueError(f"Unknown curriculum stage {requested_name!r}. Available stages: {names}")
    weights = np.asarray([max(0.0, stage.weight) for stage in stages], dtype=np.float64)
    if float(weights.sum()) <= 0.0:
        weights = np.ones(len(stages), dtype=np.float64)
    weights /= float(weights.sum())
    return stages[int(rng.choice(len(stages), p=weights))]


def config_for_stage(base_config: dict[str, Any], stage: StageSpec) -> dict[str, Any]:
    """Return a simulator config with the maze dimensions overridden for a stage."""
    config = deepcopy(base_config)
    config["maze"]["width_cells"] = stage.width_cells
    config["maze"]["height_cells"] = stage.height_cells
    config["maze"]["cell_size_m"] = stage.cell_size_m
    config["maze"]["cell_width_m"] = stage.cell_size_m
    config["maze"]["cell_length_m"] = stage.cell_size_m
    return config


def maze_for_stage(stage: StageSpec, seed: int) -> Maze:
    """Generate the occupancy-grid maze for a curriculum stage."""
    if stage.kind in RANDOM_STAGE_KINDS:
        return generate_maze_from_spec(_spec(stage, seed, (1, 1), (stage.height_cells - 2, stage.width_cells - 2)))
    if stage.kind == "straight_corridor":
        return _straight_corridor(stage, seed)
    if stage.kind in {"one_90_turn", "one_90_turn_right"}:
        return _one_turn(stage, seed, turn_direction="right")
    if stage.kind == "one_90_turn_left":
        return _one_turn(stage, seed, turn_direction="left")
    if stage.kind in {"s_turns", "s_turns_right_first"}:
        return _s_turns(stage, seed, first_turn="right")
    if stage.kind == "s_turns_left_first":
        return _s_turns(stage, seed, first_turn="left")
    if stage.kind == "t_junctions":
        return _t_junctions(stage, seed)
    raise ValueError(f"Unsupported curriculum stage kind: {stage.kind}")


def _straight_corridor(stage: StageSpec, seed: int) -> Maze:
    row = stage.height_cells // 2
    start = (row, 1)
    goal = (row, stage.width_cells - 2)
    grid = _wall_grid(stage)
    _carve_line(grid, start, goal)
    return Maze(spec=_spec(stage, seed, start, goal), grid=grid)


def _one_turn(stage: StageSpec, seed: int, *, turn_direction: str) -> Maze:
    if turn_direction == "left":
        start = (stage.height_cells - 2, 1)
        corner = (stage.height_cells - 2, stage.width_cells - 2)
        goal = (1, stage.width_cells - 2)
    else:
        start = (1, 1)
        corner = (1, stage.width_cells - 2)
        goal = (stage.height_cells - 2, stage.width_cells - 2)
    grid = _wall_grid(stage)
    _carve_polyline(grid, [start, corner, goal])
    return Maze(spec=_spec(stage, seed, start, goal), grid=grid)


def _s_turns(stage: StageSpec, seed: int, *, first_turn: str) -> Maze:
    height = max(9, stage.height_cells)
    width = max(9, stage.width_cells)
    stage = _resized_stage(stage, width_cells=width, height_cells=height)
    if first_turn == "left":
        start = (stage.height_cells - 2, 1)
        points = [
            start,
            (stage.height_cells - 2, stage.width_cells - 2),
            (stage.height_cells - 4, stage.width_cells - 2),
            (stage.height_cells - 4, 1),
            (1, 1),
            (1, stage.width_cells - 2),
        ]
    else:
        start = (1, 1)
        points = [
            start,
            (1, stage.width_cells - 2),
            (3, stage.width_cells - 2),
            (3, 1),
            (stage.height_cells - 2, 1),
            (stage.height_cells - 2, stage.width_cells - 2),
        ]
    grid = _wall_grid(stage)
    _carve_polyline(grid, points)
    return Maze(spec=_spec(stage, seed, start, points[-1]), grid=grid)


def _t_junctions(stage: StageSpec, seed: int) -> Maze:
    height = max(9, stage.height_cells)
    width = max(9, stage.width_cells)
    stage = _resized_stage(stage, width_cells=width, height_cells=height)
    mid_row = stage.height_cells // 2
    mid_col = stage.width_cells // 2
    start = (stage.height_cells - 2, 1)
    goal = (1, stage.width_cells - 2)
    path = [start, (mid_row, 1), (mid_row, mid_col), (mid_row, stage.width_cells - 2), goal]
    branches = [
        [(mid_row, mid_col), (1, mid_col)],
        [(mid_row, mid_col), (stage.height_cells - 2, mid_col)],
    ]
    grid = _wall_grid(stage)
    _carve_polyline(grid, path)
    for branch in branches:
        _carve_polyline(grid, branch)
    return Maze(spec=_spec(stage, seed, start, goal), grid=grid)


def _spec(stage: StageSpec, seed: int, start: Cell, goal: Cell) -> MazeSpec:
    return MazeSpec(
        width_cells=stage.width_cells,
        height_cells=stage.height_cells,
        cell_size_m=stage.cell_size_m,
        seed=int(seed),
        start_cell=start,
        goal_cell=goal,
        cell_width_m=stage.cell_size_m,
        cell_length_m=stage.cell_size_m,
    )


def _wall_grid(stage: StageSpec) -> np.ndarray:
    return np.full((stage.height_cells, stage.width_cells), WALL, dtype=np.uint8)


def _carve_polyline(grid: np.ndarray, points: list[Cell]) -> None:
    for start, end in zip(points, points[1:]):
        _carve_line(grid, start, end)


def _carve_line(grid: np.ndarray, start: Cell, end: Cell) -> None:
    row, col = start
    target_row, target_col = end
    row_step = 0 if target_row == row else (1 if target_row > row else -1)
    col_step = 0 if target_col == col else (1 if target_col > col else -1)
    if row_step and col_step:
        raise ValueError(f"Only axis-aligned corridor segments are supported: {start}->{end}")
    while (row, col) != (target_row, target_col):
        grid[row, col] = FREE
        row += row_step
        col += col_step
    grid[target_row, target_col] = FREE


def _resized_stage(stage: StageSpec, *, width_cells: int, height_cells: int) -> StageSpec:
    return StageSpec(
        name=stage.name,
        kind=stage.kind,
        width_cells=_odd(width_cells),
        height_cells=_odd(height_cells),
        cell_size_m=stage.cell_size_m,
        weight=stage.weight,
        scan_noise_std_m=stage.scan_noise_std_m,
        action_noise_std=stage.action_noise_std,
        friction_range=stage.friction_range,
        start_yaw_offset_rad=stage.start_yaw_offset_rad,
    )


def _odd(value: Any) -> int:
    result = max(5, int(value))
    return result if result % 2 == 1 else result + 1
