import math

from nav.controller import GOAL_REACHED, RUNNING, Pose2D, WaypointFollower, WaypointFollowerConfig, wrap_angle


def test_controller_arc_turns_when_heading_error_is_large():
    follower = WaypointFollower(
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        WaypointFollowerConfig(heading_threshold_rad=0.2, max_yaw_rate_radps=0.5, arc_turn_speed_mps=0.4),
    )

    output = follower.update(Pose2D(x=0.0, y=0.0, yaw=math.pi / 2))

    assert output.command.vx == 0.4
    assert output.command.yaw_rate == -0.5
    assert output.status == RUNNING
    assert output.waypoint_index == 1


def test_controller_walks_forward_when_heading_is_aligned():
    follower = WaypointFollower(
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        WaypointFollowerConfig(forward_speed_mps=0.2),
    )

    output = follower.update(Pose2D(x=0.0, y=0.0, yaw=0.0))

    assert output.command.vx == 0.2
    assert abs(output.command.yaw_rate) < 1e-9
    assert output.waypoint_index == 1


def test_controller_reports_goal_reached_inside_goal_tolerance():
    follower = WaypointFollower(
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        WaypointFollowerConfig(goal_tolerance_m=0.25),
    )

    output = follower.update(Pose2D(x=0.9, y=0.0, yaw=0.0))

    assert output.status == GOAL_REACHED
    assert output.command.vx == 0.0
    assert output.command.yaw_rate == 0.0


def test_wrap_angle_uses_signed_pi_range():
    assert math.isclose(wrap_angle(3 * math.pi), -math.pi)
    assert math.isclose(wrap_angle(-3 * math.pi), -math.pi)
