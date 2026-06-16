import math

import numpy as np
import pytest

from nav.controller import (
    Pose2D,
    VelocityCommand,
    WaypointFollower,
    WaypointFollowerConfig,
    heading_error,
    integrate_point_robot,
)
from sim.robot_interface import GroundTruthPoseProvider, RobotInterface


def test_heading_error_wraps_to_shortest_direction():
    assert heading_error(math.radians(179), math.radians(-179)) == pytest.approx(math.radians(2))


def test_rotate_before_walk_outputs_yaw_only_command():
    follower = WaypointFollower(WaypointFollowerConfig(max_yaw_rate_radps=0.5))

    output = follower.compute_command(Pose2D(0.0, 0.0, math.pi / 2.0), [(2.0, 0.0, 0.0)])

    assert output.status == "rotate"
    assert output.command.vx == 0.0
    assert output.command.vy == 0.0
    assert output.command.wz == pytest.approx(-0.5)


def test_walk_command_is_clipped_to_configured_limits():
    follower = WaypointFollower(
        WaypointFollowerConfig(
            max_forward_speed_mps=0.25,
            max_yaw_rate_radps=0.4,
            rotate_to_heading_rad=0.8,
        )
    )

    output = follower.compute_command(Pose2D(0.0, 0.0, 0.0), [(10.0, 0.0, 0.0)])

    assert output.status == "walk"
    assert output.command.vx == pytest.approx(0.25)
    assert output.command.wz == 0.0


def test_waypoint_switching_advances_past_reached_waypoint():
    follower = WaypointFollower(WaypointFollowerConfig(waypoint_tolerance_m=0.25, goal_tolerance_m=0.1))

    output = follower.compute_command(Pose2D(0.0, 0.0, 0.0), [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])

    assert output.waypoint_index == 1
    assert output.target_waypoint == (1.0, 0.0, 0.0)


def test_goal_reached_reports_zero_command():
    follower = WaypointFollower(WaypointFollowerConfig(goal_tolerance_m=0.5))

    output = follower.compute_command(Pose2D(0.1, 0.0, 1.0), [(0.0, 0.0, 0.0)])

    assert output.status == "goal_reached"
    assert output.command == VelocityCommand(0.0, 0.0, 0.0)


def test_point_robot_integrates_body_frame_velocity():
    next_pose = integrate_point_robot(Pose2D(0.0, 0.0, math.pi / 2.0), VelocityCommand(1.0, 0.0, 0.5), 1.0)

    assert next_pose.x == pytest.approx(0.0)
    assert next_pose.y == pytest.approx(1.0)
    assert next_pose.yaw == pytest.approx(math.pi / 2.0 + 0.5)


def test_robot_interface_accepts_velocity_command_without_actuating_g1():
    model = _DummyModel()
    data = _DummyData()
    interface = RobotInterface(_DummyMujoco(), model, data)

    result = interface.apply_velocity_command(VelocityCommand(0.1, 0.0, 0.2))

    assert result["adapter"] == "unitree_g1_velocity_placeholder"
    assert result["applied"] is False
    assert interface.get_state()["last_command"] == {"type": "velocity", "vx": 0.1, "vy": 0.0, "wz": 0.2}


def test_ground_truth_pose_provider_is_oracle_only():
    with pytest.raises(ValueError, match="MODE=oracle"):
        GroundTruthPoseProvider(_DummyModel(), _DummyData(), mode="autonomous")

    provider = GroundTruthPoseProvider(_DummyModel(), _DummyData(), mode="oracle")

    assert provider.get_pose2d()["source"] == "ground_truth_oracle"
    assert provider.get_pose2d()["yaw"] == pytest.approx(math.pi / 2.0)


class _DummyModel:
    nq = 7
    nv = 6
    nu = 0
    njnt = 0


class _DummyData:
    qpos = np.array([1.0, 2.0, 0.79, math.cos(math.pi / 4.0), 0.0, 0.0, math.sin(math.pi / 4.0)])
    qvel = np.zeros(6)
    time = 0.0


class _DummyMujoco:
    class mjtObj:
        mjOBJ_JOINT = object()

    @staticmethod
    def mj_id2name(model, obj, joint_id):
        return None
