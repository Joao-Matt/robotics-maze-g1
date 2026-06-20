"""Turn-aware oracle route following for the Unitree RL Gym native G1 policy."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Literal

from maze.grid import Cell, Maze
from nav.controller import Pose2D, wrap_angle
from sim.locomotion_policy_adapter import VelocityCommand
from sim.world_builder import cell_to_world_xy


FOLLOW_STRAIGHT = "FOLLOW_STRAIGHT"
PRE_TURN_SLOWDOWN = "PRE_TURN_SLOWDOWN"
ARC_TURN = "ARC_TURN"
POST_TURN_REALIGN = "POST_TURN_REALIGN"
RECOVERY = "RECOVERY"
GOAL_REACHED = "GOAL_REACHED"
FAILED = "FAILED"

TurnDirection = Literal["left", "right"]


@dataclass(frozen=True)
class TurnAwareFollowerConfig:
    approach_tolerance_m: float = 0.35
    waypoint_tolerance_m: float = 0.75
    goal_tolerance_m: float = 0.5
    heading_threshold_rad: float = 0.45
    forward_speed_mps: float = 0.8
    heading_gain: float = 1.4
    max_yaw_rate_radps: float = 0.8
    turn_start_distance_m: float = 0.8
    pre_turn_distance_m: float = 0.8
    arc_turn_forward_speed_mps: float = 0.4
    arc_turn_yaw_rate_radps: float = 0.8
    post_turn_heading_tolerance_rad: float = 0.3
    stuck_timeout_s: float = 8.0
    stuck_min_progress_m: float = 0.08
    max_recovery_attempts: int = 2
    recovery_stop_s: float = 0.3
    recovery_reverse_s: float = 0.8
    recovery_arc_s: float = 1.2
    recovery_reverse_speed_mps: float = -0.18
    recovery_arc_speed_mps: float = 0.22


@dataclass(frozen=True)
class NavigationSegment:
    index: int
    state: str
    start_xy: tuple[float, float]
    end_xy: tuple[float, float]
    target_heading_rad: float
    turn_direction: TurnDirection | None = None
    yaw_rate_radps: float = 0.0
    corner_xy: tuple[float, float] | None = None

    @property
    def length_m(self) -> float:
        return _distance_xy(self.start_xy, self.end_xy)

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class TurnAwarePath:
    dense_cells: list[Cell]
    dense_waypoints: list[tuple[float, float, float]]
    segments: list[NavigationSegment]
    pre_turn_points: list[tuple[float, float]]
    post_turn_points: list[tuple[float, float]]
    arc_segments: list[NavigationSegment]

    def to_dict(self) -> dict[str, object]:
        return {
            "dense_cells": self.dense_cells,
            "dense_waypoints": self.dense_waypoints,
            "segments": [segment.to_dict() for segment in self.segments],
            "pre_turn_points": self.pre_turn_points,
            "post_turn_points": self.post_turn_points,
            "arc_segments": [segment.to_dict() for segment in self.arc_segments],
        }


@dataclass(frozen=True)
class ControllerEvent:
    time_s: float
    event: str
    state: str
    segment_index: int
    detail: str
    turn_direction: TurnDirection | None = None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class TurnAwareControllerOutput:
    command: VelocityCommand
    state: str
    segment_index: int
    target_xy: tuple[float, float]
    distance_to_target_m: float
    heading_error_rad: float
    progress_m: float
    recovery_attempts: int
    events: list[ControllerEvent]
    failure_reason: str | None = None


class TurnAwareOracleFollower:
    """State machine that follows preprocessed straight and turn segments."""

    def __init__(self, path: TurnAwarePath, config: TurnAwareFollowerConfig) -> None:
        if not path.segments:
            raise ValueError("TurnAwareOracleFollower requires at least one segment.")
        self.path = path
        self.config = config
        self.segment_index = 0
        self.state = path.segments[0].state
        self.recovery_attempts = 0
        self.failure_reason: str | None = None
        self._best_progress = float("-inf")
        self._last_progress_time: float | None = None
        self._recovery_start_time: float | None = None
        self._state_announced = False
        self._arc_announced_segments: set[int] = set()
        self._last_turn_direction: TurnDirection | None = None

    @property
    def current_segment(self) -> NavigationSegment:
        return self.path.segments[min(self.segment_index, len(self.path.segments) - 1)]

    def update(self, pose: Pose2D, sim_time_s: float) -> TurnAwareControllerOutput:
        events: list[ControllerEvent] = []
        if self.state != RECOVERY:
            self._advance_completed_segments(pose, sim_time_s, events)

        if self.failure_reason:
            return self._output(VelocityCommand(), FAILED, pose, events, self.failure_reason)
        if self.segment_index >= len(self.path.segments):
            return self._output(VelocityCommand(), GOAL_REACHED, pose, events)

        segment = self.current_segment
        if self.state == RECOVERY:
            command = self._recovery_command(sim_time_s, events)
            return self._output(command, self.state, pose, events, self.failure_reason)

        self._set_state(segment.state, sim_time_s, events)
        if segment.state == ARC_TURN and segment.index not in self._arc_announced_segments:
            self._arc_announced_segments.add(segment.index)
            self._last_turn_direction = segment.turn_direction
            events.append(
                ControllerEvent(
                    time_s=sim_time_s,
                    event="turn_start",
                    state=ARC_TURN,
                    segment_index=segment.index,
                    detail=f"{segment.turn_direction} turn",
                    turn_direction=segment.turn_direction,
                )
            )

        command = self._segment_command(pose, segment)
        command = self._maybe_enter_recovery(command, pose, sim_time_s, events)

        return self._output(command, self.state, pose, events, self.failure_reason)

    def _advance_completed_segments(self, pose: Pose2D, sim_time_s: float, events: list[ControllerEvent]) -> None:
        while self.segment_index < len(self.path.segments):
            segment = self.current_segment
            if not self._segment_complete(pose, segment):
                break
            events.append(
                ControllerEvent(
                    time_s=sim_time_s,
                    event="segment_complete",
                    state=segment.state,
                    segment_index=segment.index,
                    detail=f"completed {segment.state}",
                    turn_direction=segment.turn_direction,
                )
            )
            self.segment_index += 1
            self.recovery_attempts = 0
            self._best_progress = float("-inf")
            self._last_progress_time = sim_time_s
            if self.segment_index >= len(self.path.segments):
                self._set_state(GOAL_REACHED, sim_time_s, events)
                break

    def _segment_complete(self, pose: Pose2D, segment: NavigationSegment) -> bool:
        distance = _distance_xy((pose.x, pose.y), segment.end_xy)
        is_final_segment = segment.index == len(self.path.segments) - 1
        if is_final_segment:
            return distance <= self.config.goal_tolerance_m
        if segment.state in (FOLLOW_STRAIGHT, PRE_TURN_SLOWDOWN):
            return distance <= self.config.approach_tolerance_m
        return distance <= self.config.waypoint_tolerance_m

    def _segment_command(self, pose: Pose2D, segment: NavigationSegment) -> VelocityCommand:
        if segment.state == ARC_TURN:
            return VelocityCommand(vx=self.config.arc_turn_forward_speed_mps, vy=0.0, yaw_rate=segment.yaw_rate_radps)

        heading = math.atan2(segment.end_xy[1] - pose.y, segment.end_xy[0] - pose.x)
        if segment.length_m <= 1e-6:
            heading = segment.target_heading_rad
        heading_error = wrap_angle(heading - pose.yaw)
        yaw_rate = _clip(
            self.config.heading_gain * heading_error,
            -self.config.max_yaw_rate_radps,
            self.config.max_yaw_rate_radps,
        )
        if segment.state in (PRE_TURN_SLOWDOWN, POST_TURN_REALIGN):
            speed = min(self.config.forward_speed_mps, self.config.arc_turn_forward_speed_mps)
        else:
            speed = self.config.forward_speed_mps
        return VelocityCommand(vx=speed, vy=0.0, yaw_rate=yaw_rate)

    def _maybe_enter_recovery(
        self,
        command: VelocityCommand,
        pose: Pose2D,
        sim_time_s: float,
        events: list[ControllerEvent],
    ) -> VelocityCommand:
        if abs(command.vx) <= 1e-4:
            self._last_progress_time = sim_time_s
            return command

        route_progress = float(self.segment_index) * 1000.0 + _segment_progress_m(pose, self.current_segment)
        if self._last_progress_time is None or route_progress > self._best_progress + self.config.stuck_min_progress_m:
            self._best_progress = route_progress
            self._last_progress_time = sim_time_s
            return command

        if sim_time_s - self._last_progress_time <= self.config.stuck_timeout_s:
            return command

        if self.recovery_attempts >= self.config.max_recovery_attempts:
            self.failure_reason = "max recovery attempts exceeded"
            self._set_state(FAILED, sim_time_s, events)
            return VelocityCommand()

        self.recovery_attempts += 1
        self._recovery_start_time = sim_time_s
        self._set_state(RECOVERY, sim_time_s, events)
        events.append(
            ControllerEvent(
                time_s=sim_time_s,
                event="recovery_start",
                state=RECOVERY,
                segment_index=self.current_segment.index,
                detail=f"attempt {self.recovery_attempts}",
                turn_direction=self._last_turn_direction,
            )
        )
        return VelocityCommand()

    def _recovery_command(self, sim_time_s: float, events: list[ControllerEvent]) -> VelocityCommand:
        start = self._recovery_start_time if self._recovery_start_time is not None else sim_time_s
        elapsed = sim_time_s - start
        if elapsed < self.config.recovery_stop_s:
            return VelocityCommand()
        if elapsed < self.config.recovery_stop_s + self.config.recovery_reverse_s:
            return VelocityCommand(vx=self.config.recovery_reverse_speed_mps, vy=0.0, yaw_rate=0.0)
        if elapsed < self.config.recovery_stop_s + self.config.recovery_reverse_s + self.config.recovery_arc_s:
            yaw_sign = 1.0 if self._last_turn_direction == "right" else -1.0
            return VelocityCommand(
                vx=self.config.recovery_arc_speed_mps,
                vy=0.0,
                yaw_rate=yaw_sign * self.config.arc_turn_yaw_rate_radps,
            )

        self._recovery_start_time = None
        self._best_progress = float("-inf")
        self._last_progress_time = sim_time_s
        state = self.current_segment.state
        self._set_state(state, sim_time_s, events)
        events.append(
            ControllerEvent(
                time_s=sim_time_s,
                event="recovery_end",
                state=state,
                segment_index=self.current_segment.index,
                detail=f"attempt {self.recovery_attempts} complete",
                turn_direction=self._last_turn_direction,
            )
        )
        return VelocityCommand()

    def _set_state(self, state: str, sim_time_s: float, events: list[ControllerEvent]) -> None:
        if self.state == state and self._state_announced:
            return
        previous = self.state
        self.state = state
        self._state_announced = True
        events.append(
            ControllerEvent(
                time_s=sim_time_s,
                event="state_change",
                state=state,
                segment_index=min(self.segment_index, len(self.path.segments) - 1),
                detail=f"{previous}->{state}",
            )
        )

    def _output(
        self,
        command: VelocityCommand,
        state: str,
        pose: Pose2D,
        events: list[ControllerEvent],
        failure_reason: str | None = None,
    ) -> TurnAwareControllerOutput:
        segment = self.current_segment
        return TurnAwareControllerOutput(
            command=command,
            state=state,
            segment_index=min(self.segment_index, len(self.path.segments)),
            target_xy=segment.end_xy,
            distance_to_target_m=_distance_xy((pose.x, pose.y), segment.end_xy),
            heading_error_rad=wrap_angle(segment.target_heading_rad - pose.yaw),
            progress_m=_segment_progress_m(pose, segment),
            recovery_attempts=self.recovery_attempts,
            events=list(events),
            failure_reason=failure_reason,
        )


def build_turn_aware_path(maze: Maze, cells: list[Cell], config: TurnAwareFollowerConfig) -> TurnAwarePath:
    if len(cells) < 2:
        raise ValueError("Turn-aware path requires at least two cells.")

    points = [cell_to_world_xy(maze, cell) for cell in cells]
    dense_waypoints = [(x, y, 0.0) for x, y in points]
    segments: list[NavigationSegment] = []
    pre_turn_points: list[tuple[float, float]] = []
    post_turn_points: list[tuple[float, float]] = []
    arc_segments: list[NavigationSegment] = []
    current = points[0]
    post_realign_pending = False

    for index in range(1, len(points) - 1):
        previous_point = points[index - 1]
        corner = points[index]
        next_point = points[index + 1]
        incoming = _unit_vector(previous_point, corner)
        outgoing = _unit_vector(corner, next_point)
        if incoming == outgoing:
            continue

        turn_direction = turn_direction_from_vectors(incoming, outgoing)
        incoming_distance = _distance_xy(previous_point, corner)
        outgoing_distance = _distance_xy(corner, next_point)
        turn_start = min(
            max(0.0, config.turn_start_distance_m),
            incoming_distance * 0.65,
            outgoing_distance * 0.65,
        )
        pre_turn = (corner[0] - incoming[0] * turn_start, corner[1] - incoming[1] * turn_start)
        post_turn = (corner[0] + outgoing[0] * turn_start, corner[1] + outgoing[1] * turn_start)

        post_realign_pending = _append_approach_segments(
            segments,
            current,
            pre_turn,
            incoming,
            config,
            post_realign_pending=post_realign_pending,
        )
        pre_turn_points.append(pre_turn)
        post_turn_points.append(post_turn)
        yaw_rate = signed_turn_yaw_rate(turn_direction, config.arc_turn_yaw_rate_radps)
        arc_segment = _make_segment(
            segments,
            ARC_TURN,
            pre_turn,
            post_turn,
            math.atan2(outgoing[1], outgoing[0]),
            turn_direction=turn_direction,
            yaw_rate_radps=yaw_rate,
            corner_xy=corner,
        )
        arc_segments.append(arc_segment)
        current = post_turn
        post_realign_pending = True

    _append_final_segments(segments, current, points[-1], config, post_realign_pending=post_realign_pending)
    if not segments:
        _make_segment(segments, FOLLOW_STRAIGHT, points[0], points[-1], _heading(points[0], points[-1]))

    return TurnAwarePath(
        dense_cells=list(cells),
        dense_waypoints=dense_waypoints,
        segments=segments,
        pre_turn_points=pre_turn_points,
        post_turn_points=post_turn_points,
        arc_segments=arc_segments,
    )


def turn_direction_from_vectors(incoming: tuple[float, float], outgoing: tuple[float, float]) -> TurnDirection:
    cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
    return "left" if cross > 0.0 else "right"


def signed_turn_yaw_rate(turn_direction: TurnDirection, yaw_rate_magnitude: float) -> float:
    magnitude = abs(float(yaw_rate_magnitude))
    return magnitude if turn_direction == "left" else -magnitude


def _append_approach_segments(
    segments: list[NavigationSegment],
    start: tuple[float, float],
    pre_turn: tuple[float, float],
    incoming: tuple[float, float],
    config: TurnAwareFollowerConfig,
    *,
    post_realign_pending: bool,
) -> bool:
    length = _distance_xy(start, pre_turn)
    if length <= 1e-6:
        return post_realign_pending
    slow_distance = min(max(0.0, config.pre_turn_distance_m), length * 0.5)
    slow_start = (pre_turn[0] - incoming[0] * slow_distance, pre_turn[1] - incoming[1] * slow_distance)

    if post_realign_pending:
        realign_end = slow_start if _distance_xy(start, slow_start) > 1e-6 else pre_turn
        _make_segment(segments, POST_TURN_REALIGN, start, realign_end, _heading(start, realign_end))
        start = realign_end
        post_realign_pending = False

    if _distance_xy(start, slow_start) > 1e-6:
        _make_segment(segments, FOLLOW_STRAIGHT, start, slow_start, _heading(start, slow_start))
        start = slow_start
    if _distance_xy(start, pre_turn) > 1e-6:
        _make_segment(segments, PRE_TURN_SLOWDOWN, start, pre_turn, _heading(start, pre_turn))
    return post_realign_pending


def _append_final_segments(
    segments: list[NavigationSegment],
    start: tuple[float, float],
    goal: tuple[float, float],
    config: TurnAwareFollowerConfig,
    *,
    post_realign_pending: bool,
) -> None:
    if _distance_xy(start, goal) <= 1e-6:
        return
    if post_realign_pending:
        length = _distance_xy(start, goal)
        realign_distance = min(max(0.0, config.pre_turn_distance_m), length * 0.5)
        direction = _unit_vector(start, goal)
        realign_end = (start[0] + direction[0] * realign_distance, start[1] + direction[1] * realign_distance)
        if _distance_xy(start, realign_end) > 1e-6:
            _make_segment(segments, POST_TURN_REALIGN, start, realign_end, _heading(start, realign_end))
            start = realign_end
    if _distance_xy(start, goal) > 1e-6:
        _make_segment(segments, FOLLOW_STRAIGHT, start, goal, _heading(start, goal))


def _make_segment(
    segments: list[NavigationSegment],
    state: str,
    start: tuple[float, float],
    end: tuple[float, float],
    target_heading: float,
    *,
    turn_direction: TurnDirection | None = None,
    yaw_rate_radps: float = 0.0,
    corner_xy: tuple[float, float] | None = None,
) -> NavigationSegment:
    segment = NavigationSegment(
        index=len(segments),
        state=state,
        start_xy=start,
        end_xy=end,
        target_heading_rad=target_heading,
        turn_direction=turn_direction,
        yaw_rate_radps=yaw_rate_radps,
        corner_xy=corner_xy,
    )
    segments.append(segment)
    return segment


def _segment_progress_m(pose: Pose2D, segment: NavigationSegment) -> float:
    start = segment.start_xy
    end = segment.end_xy
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-12:
        return 0.0
    px = pose.x - start[0]
    py = pose.y - start[1]
    projection = (px * vx + py * vy) / math.sqrt(length_sq)
    return _clip(projection, 0.0, math.sqrt(length_sq))


def _unit_vector(start: tuple[float, float], end: tuple[float, float]) -> tuple[float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= 1e-12:
        return 0.0, 0.0
    return dx / length, dy / length


def _heading(start: tuple[float, float], end: tuple[float, float]) -> float:
    return math.atan2(end[1] - start[1], end[0] - start[0])


def _distance_xy(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))
