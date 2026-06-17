import numpy as np

from maze.grid import FREE, WALL, Maze, MazeSpec
from nav.controller import Pose2D
from nav.oracle_follow import (
    ARC_TURN,
    FAILED,
    RECOVERY,
    NavigationSegment,
    TurnAwareFollowerConfig,
    TurnAwareOracleFollower,
    TurnAwarePath,
    build_turn_aware_path,
    signed_turn_yaw_rate,
)
from nav.planner import plan_oracle_path


def test_right_turn_metadata_produces_negative_yaw_rate():
    path = build_turn_aware_path(_open_maze(), [(1, 1), (1, 2), (2, 2)], TurnAwareFollowerConfig())

    arc = path.arc_segments[0]

    assert arc.turn_direction == "right"
    assert arc.yaw_rate_radps < 0.0
    assert signed_turn_yaw_rate("right", 0.8) == -0.8


def test_left_turn_metadata_produces_positive_yaw_rate():
    path = build_turn_aware_path(_open_maze(), [(2, 1), (2, 2), (1, 2)], TurnAwareFollowerConfig())

    arc = path.arc_segments[0]

    assert arc.turn_direction == "left"
    assert arc.yaw_rate_radps > 0.0
    assert signed_turn_yaw_rate("left", 0.8) == 0.8


def test_pre_turn_and_post_turn_points_are_generated_around_corner():
    config = TurnAwareFollowerConfig(turn_start_distance_m=0.5, pre_turn_distance_m=0.25)
    path = build_turn_aware_path(_open_maze(), [(1, 1), (1, 2), (2, 2)], config)

    assert len(path.pre_turn_points) == 1
    assert len(path.post_turn_points) == 1
    assert path.pre_turn_points[0] != path.arc_segments[0].corner_xy
    assert path.post_turn_points[0] != path.arc_segments[0].corner_xy


def test_arc_turn_command_has_forward_velocity():
    arc = NavigationSegment(
        index=0,
        state=ARC_TURN,
        start_xy=(0.0, 0.0),
        end_xy=(0.5, -0.5),
        target_heading_rad=0.0,
        turn_direction="right",
        yaw_rate_radps=-0.8,
    )
    path = TurnAwarePath(
        dense_cells=[(1, 1), (1, 2)],
        dense_waypoints=[(0.0, 0.0, 0.0), (0.5, -0.5, 0.0)],
        segments=[arc],
        pre_turn_points=[],
        post_turn_points=[],
        arc_segments=[arc],
    )
    follower = TurnAwareOracleFollower(path, TurnAwareFollowerConfig(arc_turn_forward_speed_mps=0.4))

    output = follower.update(Pose2D(x=0.0, y=0.0, yaw=0.0), sim_time_s=0.0)

    assert output.state == ARC_TURN
    assert output.command.vx > 0.0
    assert output.command.yaw_rate < 0.0


def test_stuck_detector_enters_recovery_when_progress_is_low():
    follower = TurnAwareOracleFollower(_straight_path(), TurnAwareFollowerConfig(stuck_timeout_s=0.1, stuck_min_progress_m=0.01))
    pose = Pose2D(x=0.0, y=0.0, yaw=0.0)

    follower.update(pose, sim_time_s=0.0)
    output = follower.update(pose, sim_time_s=0.2)

    assert output.state == RECOVERY
    assert any(event.event == "recovery_start" for event in output.events)


def test_recovery_attempts_are_bounded():
    follower = TurnAwareOracleFollower(
        _straight_path(),
        TurnAwareFollowerConfig(stuck_timeout_s=0.1, stuck_min_progress_m=0.01, max_recovery_attempts=0),
    )
    pose = Pose2D(x=0.0, y=0.0, yaw=0.0)

    follower.update(pose, sim_time_s=0.0)
    output = follower.update(pose, sim_time_s=0.2)

    assert output.state == FAILED
    assert output.failure_reason == "max recovery attempts exceeded"


def test_heading_astar_turn_penalty_prefers_low_zigzag_route_on_synthetic_map():
    grid = np.full((7, 9), WALL, dtype=np.uint8)
    for cell in [(3, 1), (2, 1), (2, 2), (3, 2), (3, 3), (2, 3), (2, 4), (3, 4), (3, 5), (2, 5), (2, 6), (2, 7), (3, 7)]:
        grid[cell] = FREE
    for cell in [(4, 1), (5, 1), (5, 2), (5, 3), (5, 4), (5, 5), (5, 6), (5, 7), (4, 7)]:
        grid[cell] = FREE
    maze = Maze(
        spec=MazeSpec(width_cells=9, height_cells=7, cell_size_m=1.0, seed=1, start_cell=(3, 1), goal_cell=(3, 7)),
        grid=grid,
    )

    plan = plan_oracle_path(maze, planner="heading_astar", turn_penalty_cost=3.0, simplify=False)

    assert _turn_count(plan.cells) <= 3


def _open_maze() -> Maze:
    grid = np.zeros((5, 5), dtype=np.uint8)
    return Maze(
        spec=MazeSpec(width_cells=5, height_cells=5, cell_size_m=1.0, seed=1, start_cell=(1, 1), goal_cell=(3, 3)),
        grid=grid,
    )


def _straight_path() -> TurnAwarePath:
    segment = NavigationSegment(
        index=0,
        state="FOLLOW_STRAIGHT",
        start_xy=(0.0, 0.0),
        end_xy=(3.0, 0.0),
        target_heading_rad=0.0,
    )
    return TurnAwarePath(
        dense_cells=[(1, 1), (1, 2)],
        dense_waypoints=[(0.0, 0.0, 0.0), (3.0, 0.0, 0.0)],
        segments=[segment],
        pre_turn_points=[],
        post_turn_points=[],
        arc_segments=[],
    )


def _turn_count(cells: list[tuple[int, int]]) -> int:
    if len(cells) < 3:
        return 0
    count = 0
    previous = (cells[1][0] - cells[0][0], cells[1][1] - cells[0][1])
    for index in range(1, len(cells) - 1):
        current = (cells[index + 1][0] - cells[index][0], cells[index + 1][1] - cells[index][1])
        if current != previous:
            count += 1
        previous = current
    return count
