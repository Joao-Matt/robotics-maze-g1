"""Oracle-path feature extraction for the direct MuJoCo velocity controller."""

from __future__ import annotations

from dataclasses import dataclass
import math

from nav.controller import Pose2D, wrap_angle
from nav.oracle_follow import ARC_TURN, NavigationSegment, TurnAwarePath


@dataclass(frozen=True)
class PathProjection:
    """Nearest projection of the robot pose onto the oracle route."""

    segment_index: int
    segment: NavigationSegment
    progress_m: float
    segment_progress_m: float
    lateral_error_m: float
    heading_error_rad: float
    distance_to_segment_end_m: float
    distance_to_goal_m: float
    angle_to_goal_rad: float
    distance_to_next_corner_m: float
    next_corner_angle_rad: float


class OraclePathFeatureExtractor:
    """Compute progress and local route geometry against a turn-aware oracle path."""

    def __init__(self, path: TurnAwarePath) -> None:
        if not path.segments:
            raise ValueError("OraclePathFeatureExtractor requires at least one segment.")
        self.path = path
        self.segment_starts: list[float] = []
        total = 0.0
        for segment in path.segments:
            self.segment_starts.append(total)
            total += segment.length_m
        self.total_length_m = max(total, 1e-6)
        self.goal_xy = path.segments[-1].end_xy

    def project(self, pose: Pose2D) -> PathProjection:
        """Project a pose to the nearest route segment."""
        best: tuple[float, float, float, NavigationSegment] | None = None
        for segment in self.path.segments:
            along, lateral = _project_to_segment((pose.x, pose.y), segment)
            total_progress = self.segment_starts[segment.index] + along
            score = abs(lateral)
            if best is None or score < best[0] or (math.isclose(score, best[0]) and total_progress > best[1]):
                best = (score, total_progress, along, segment)
        if best is None:
            raise RuntimeError("Path projection failed on a non-empty path.")
        _, total_progress, segment_progress, segment = best
        lateral = _signed_lateral_error((pose.x, pose.y), segment)
        heading_error = wrap_angle(segment.target_heading_rad - pose.yaw)
        distance_to_segment_end = math.hypot(segment.end_xy[0] - pose.x, segment.end_xy[1] - pose.y)
        distance_to_goal = math.hypot(self.goal_xy[0] - pose.x, self.goal_xy[1] - pose.y)
        angle_to_goal = wrap_angle(math.atan2(self.goal_xy[1] - pose.y, self.goal_xy[0] - pose.x) - pose.yaw)
        corner_distance, corner_angle = self._next_corner(total_progress, segment.index)
        return PathProjection(
            segment_index=segment.index,
            segment=segment,
            progress_m=_clip(total_progress, 0.0, self.total_length_m),
            segment_progress_m=segment_progress,
            lateral_error_m=lateral,
            heading_error_rad=heading_error,
            distance_to_segment_end_m=distance_to_segment_end,
            distance_to_goal_m=distance_to_goal,
            angle_to_goal_rad=angle_to_goal,
            distance_to_next_corner_m=corner_distance,
            next_corner_angle_rad=corner_angle,
        )

    def _next_corner(self, progress_m: float, current_index: int) -> tuple[float, float]:
        for segment in self.path.segments[current_index:]:
            if segment.state != ARC_TURN:
                continue
            distance = max(0.0, self.segment_starts[segment.index] - progress_m)
            turn = math.pi / 2.0 if segment.turn_direction == "left" else -math.pi / 2.0
            return distance, turn
        return self.total_length_m, 0.0


def _project_to_segment(point: tuple[float, float], segment: NavigationSegment) -> tuple[float, float]:
    sx, sy = segment.start_xy
    ex, ey = segment.end_xy
    vx, vy = ex - sx, ey - sy
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-12:
        return 0.0, math.hypot(point[0] - sx, point[1] - sy)
    px, py = point[0] - sx, point[1] - sy
    fraction = _clip((px * vx + py * vy) / length_sq, 0.0, 1.0)
    projection = (sx + fraction * vx, sy + fraction * vy)
    along = fraction * math.sqrt(length_sq)
    lateral = math.hypot(point[0] - projection[0], point[1] - projection[1])
    return along, lateral


def _signed_lateral_error(point: tuple[float, float], segment: NavigationSegment) -> float:
    sx, sy = segment.start_xy
    ex, ey = segment.end_xy
    vx, vy = ex - sx, ey - sy
    length = math.hypot(vx, vy)
    if length <= 1e-12:
        return 0.0
    px, py = point[0] - sx, point[1] - sy
    cross = vx * py - vy * px
    return cross / length


def _clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))

