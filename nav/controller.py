"""Conservative waypoint follower for oracle/debug waypoint tracks."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose in world coordinates."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class VelocityCommand:
    """High-level body-frame velocity command."""

    vx: float
    vy: float
    wz: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class WaypointFollowerConfig:
    """Tunable constants for conservative waypoint following."""

    max_forward_speed_mps: float = 0.25
    max_lateral_speed_mps: float = 0.0
    max_yaw_rate_radps: float = 0.5
    goal_tolerance_m: float = 0.5
    waypoint_tolerance_m: float = 0.25
    rotate_to_heading_rad: float = 0.45
    heading_deadband_rad: float = 0.04
    linear_gain: float = 0.6
    yaw_gain: float = 1.2
    supports_lateral: bool = False


@dataclass(frozen=True)
class ControlOutput:
    """Follower status plus the command to apply this tick."""

    command: VelocityCommand
    status: str
    waypoint_index: int
    target_waypoint: tuple[float, float, float] | None
    distance_to_target_m: float | None
    distance_to_goal_m: float | None
    heading_error_rad: float | None

    def to_dict(self) -> dict:
        result = asdict(self)
        result["command"] = self.command.to_dict()
        return result


class WaypointFollower:
    """Rotate-before-walk waypoint follower.

    This controller returns high-level velocity commands only. It does not claim
    to be a Unitree G1 walking controller.
    """

    def __init__(self, config: WaypointFollowerConfig | None = None) -> None:
        self.config = config or WaypointFollowerConfig()

    def compute_command(
        self,
        pose: Pose2D,
        waypoints: list[tuple[float, float, float]],
        waypoint_index: int = 0,
    ) -> ControlOutput:
        """Compute a conservative velocity command for the active waypoint."""
        if not waypoints:
            return ControlOutput(
                command=VelocityCommand(0.0, 0.0, 0.0),
                status="no_waypoints",
                waypoint_index=0,
                target_waypoint=None,
                distance_to_target_m=None,
                distance_to_goal_m=None,
                heading_error_rad=None,
            )

        index = max(0, min(int(waypoint_index), len(waypoints) - 1))
        goal_distance = _distance_xy(pose, waypoints[-1])
        if goal_distance <= self.config.goal_tolerance_m:
            return self._output(
                status="goal_reached",
                command=VelocityCommand(0.0, 0.0, 0.0),
                pose=pose,
                waypoints=waypoints,
                waypoint_index=len(waypoints) - 1,
            )

        while index < len(waypoints) - 1 and _distance_xy(pose, waypoints[index]) <= self.config.waypoint_tolerance_m:
            index += 1

        target = waypoints[index]
        distance = _distance_xy(pose, target)
        heading = math.atan2(target[1] - pose.y, target[0] - pose.x)
        error = heading_error(pose.yaw, heading)

        if abs(error) > self.config.rotate_to_heading_rad:
            command = VelocityCommand(
                vx=0.0,
                vy=0.0,
                wz=clip(self.config.yaw_gain * error, -self.config.max_yaw_rate_radps, self.config.max_yaw_rate_radps),
            )
            return self._output("rotate", command, pose, waypoints, index)

        forward = clip(
            self.config.linear_gain * distance,
            0.0,
            self.config.max_forward_speed_mps,
        )
        yaw_rate = 0.0 if abs(error) <= self.config.heading_deadband_rad else self.config.yaw_gain * error
        yaw_rate = clip(yaw_rate, -self.config.max_yaw_rate_radps, self.config.max_yaw_rate_radps)

        lateral = 0.0
        if self.config.supports_lateral:
            # Body-frame lateral correction is intentionally conservative and clipped.
            lateral_world_heading = heading_error(pose.yaw + math.pi / 2.0, heading)
            lateral = clip(
                self.config.linear_gain * distance * math.cos(lateral_world_heading),
                -self.config.max_lateral_speed_mps,
                self.config.max_lateral_speed_mps,
            )

        return self._output("walk", VelocityCommand(forward, lateral, yaw_rate), pose, waypoints, index)

    def _output(
        self,
        status: str,
        command: VelocityCommand,
        pose: Pose2D,
        waypoints: list[tuple[float, float, float]],
        waypoint_index: int,
    ) -> ControlOutput:
        target = waypoints[waypoint_index] if waypoints else None
        heading = None
        error = None
        distance = None
        if target is not None:
            distance = _distance_xy(pose, target)
            heading = math.atan2(target[1] - pose.y, target[0] - pose.x)
            error = heading_error(pose.yaw, heading)
        return ControlOutput(
            command=command,
            status=status,
            waypoint_index=waypoint_index,
            target_waypoint=target,
            distance_to_target_m=distance,
            distance_to_goal_m=_distance_xy(pose, waypoints[-1]) if waypoints else None,
            heading_error_rad=error,
        )


def heading_error(current_yaw: float, target_yaw: float) -> float:
    """Return shortest signed yaw error in radians."""
    return wrap_to_pi(target_yaw - current_yaw)


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _distance_xy(pose: Pose2D, waypoint: tuple[float, float, float]) -> float:
    return math.hypot(float(waypoint[0]) - pose.x, float(waypoint[1]) - pose.y)


def integrate_point_robot(pose: Pose2D, command: VelocityCommand, dt_s: float) -> Pose2D:
    """Integrate a simple point robot using body-frame velocity commands."""
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    world_vx = command.vx * cos_yaw - command.vy * sin_yaw
    world_vy = command.vx * sin_yaw + command.vy * cos_yaw
    return Pose2D(
        x=pose.x + world_vx * dt_s,
        y=pose.y + world_vy * dt_s,
        yaw=wrap_to_pi(pose.yaw + command.wz * dt_s),
    )
