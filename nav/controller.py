"""Conservative waypoint follower for oracle/debug locomotion."""

from __future__ import annotations

from dataclasses import dataclass
import math

from sim.locomotion_policy_adapter import VelocityCommand


RUNNING = "RUNNING"
WAYPOINT_REACHED = "WAYPOINT_REACHED"
GOAL_REACHED = "GOAL_REACHED"


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose in MuJoCo world coordinates."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class WaypointFollowerConfig:
    """Tuning for arc-turn waypoint following."""

    waypoint_tolerance_m: float = 0.35
    goal_tolerance_m: float = 0.5
    heading_threshold_rad: float = 0.45
    forward_speed_mps: float = 0.25
    arc_turn_speed_mps: float = 0.4
    heading_gain: float = 1.4
    max_yaw_rate_radps: float = 0.8


@dataclass(frozen=True)
class WaypointControllerOutput:
    """Controller output command and status."""

    command: VelocityCommand
    status: str
    waypoint_index: int
    target_waypoint: tuple[float, float, float]
    distance_to_target_m: float
    distance_to_goal_m: float
    heading_error_rad: float


class WaypointFollower:
    """Stateful arc-turn controller for world-frame waypoints."""

    def __init__(self, waypoints: list[tuple[float, float, float]], config: WaypointFollowerConfig) -> None:
        if not waypoints:
            raise ValueError("WaypointFollower requires at least one waypoint.")
        self.waypoints = list(waypoints)
        self.config = config
        self.index = 0

    def update(self, pose: Pose2D) -> WaypointControllerOutput:
        """Return the next local velocity command for the current pose."""
        self._advance_reached_waypoints(pose)
        target = self.waypoints[self.index]
        goal = self.waypoints[-1]
        distance = _distance_xy(pose, target)
        distance_to_goal = _distance_xy(pose, goal)

        if distance_to_goal <= self.config.goal_tolerance_m:
            self.index = len(self.waypoints) - 1
            target = self.waypoints[self.index]
            distance = _distance_xy(pose, target)
            return self._output(VelocityCommand(), GOAL_REACHED, target, distance, distance_to_goal, 0.0)

        target_heading = math.atan2(target[1] - pose.y, target[0] - pose.x)
        heading_error = wrap_angle(target_heading - pose.yaw)
        yaw_rate = _clip(
            self.config.heading_gain * heading_error,
            -self.config.max_yaw_rate_radps,
            self.config.max_yaw_rate_radps,
        )
        if abs(heading_error) > self.config.heading_threshold_rad:
            command = VelocityCommand(vx=self.config.arc_turn_speed_mps, vy=0.0, yaw_rate=yaw_rate)
        else:
            command = VelocityCommand(vx=self.config.forward_speed_mps, vy=0.0, yaw_rate=yaw_rate)
        status = WAYPOINT_REACHED if distance <= self.config.waypoint_tolerance_m else RUNNING
        return self._output(command, status, target, distance, distance_to_goal, heading_error)

    def _advance_reached_waypoints(self, pose: Pose2D) -> None:
        while self.index < len(self.waypoints) - 1:
            tolerance = self.config.goal_tolerance_m if self.index == len(self.waypoints) - 1 else self.config.waypoint_tolerance_m
            if _distance_xy(pose, self.waypoints[self.index]) > tolerance:
                break
            self.index += 1

    def _output(
        self,
        command: VelocityCommand,
        status: str,
        target: tuple[float, float, float],
        distance: float,
        distance_to_goal: float,
        heading_error: float,
    ) -> WaypointControllerOutput:
        return WaypointControllerOutput(
            command=command,
            status=status,
            waypoint_index=self.index,
            target_waypoint=target,
            distance_to_target_m=distance,
            distance_to_goal_m=distance_to_goal,
            heading_error_rad=heading_error,
        )


def pose_from_base_state(state: dict[str, float]) -> Pose2D:
    """Convert the shared locomotion base state dict to a planar pose."""
    return Pose2D(x=state["base_x"], y=state["base_y"], yaw=state["yaw"])


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _distance_xy(pose: Pose2D, waypoint: tuple[float, float, float]) -> float:
    return math.hypot(waypoint[0] - pose.x, waypoint[1] - pose.y)


def _clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))
