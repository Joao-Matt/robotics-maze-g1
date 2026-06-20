"""Path-progress metrics for comparing navigation runs in a maze."""

from __future__ import annotations

import math
from typing import Any, Iterable

Point2D = tuple[float, float]


def path_length(points: Iterable[Point2D]) -> float | None:
    clean = _clean_points(points)
    if len(clean) < 2:
        return None
    total = sum(_distance(a, b) for a, b in zip(clean, clean[1:]))
    return total if total > 0.0 else None


def project_point_to_path(point: Point2D, path: Iterable[Point2D]) -> tuple[float | None, str | None]:
    clean = _clean_points(path)
    length = path_length(clean)
    if length is None:
        return None, "ground-truth path is missing, has fewer than two points, or has zero length"
    px, py = point
    if not math.isfinite(px) or not math.isfinite(py):
        return None, "trajectory point is non-finite"

    best_distance = float("inf")
    best_progress = 0.0
    cumulative = 0.0
    for start, end in zip(clean, clean[1:]):
        sx, sy = start
        ex, ey = end
        dx, dy = ex - sx, ey - sy
        segment_len_sq = dx * dx + dy * dy
        segment_len = math.sqrt(segment_len_sq)
        if segment_len_sq <= 0.0:
            continue
        t = ((px - sx) * dx + (py - sy) * dy) / segment_len_sq
        t = max(0.0, min(1.0, t))
        qx, qy = sx + t * dx, sy + t * dy
        distance = math.hypot(px - qx, py - qy)
        progress = cumulative + t * segment_len
        if distance < best_distance:
            best_distance = distance
            best_progress = progress
        cumulative += segment_len
    return max(0.0, min(length, best_progress)), None


def path_progress_metrics(
    *,
    path_points: Iterable[Point2D],
    trajectory_points: Iterable[Point2D],
    distance_traveled_m: float | None = None,
) -> dict[str, Any]:
    clean_path = _clean_points(path_points)
    clean_trajectory = _clean_points(trajectory_points)
    length = path_length(clean_path)
    result: dict[str, Any] = {
        "ground_truth_path_length_m": length,
        "best_progress_along_path_m": None,
        "final_progress_along_path_m": None,
        "best_path_completion_fraction": None,
        "final_path_completion_fraction": None,
        "remaining_path_distance_m": None,
        "path_efficiency": None,
    }
    warnings: list[str] = []
    if length is None:
        result["path_progress_warning"] = "ground-truth path length is missing or zero"
        return result
    if not clean_trajectory:
        result["path_progress_warning"] = "ground-truth trajectory is missing"
        return result

    progress_values: list[float] = []
    for point in clean_trajectory:
        progress, warning = project_point_to_path(point, clean_path)
        if warning:
            warnings.append(warning)
            continue
        if progress is not None:
            progress_values.append(progress)
    if not progress_values:
        result["path_progress_warning"] = "; ".join(sorted(set(warnings))) or "no valid trajectory points"
        return result

    best = max(progress_values)
    final = progress_values[-1]
    result.update(
        {
            "best_progress_along_path_m": best,
            "final_progress_along_path_m": final,
            "best_path_completion_fraction": best / length,
            "final_path_completion_fraction": final / length,
            "remaining_path_distance_m": max(0.0, length - final),
            "path_efficiency": (
                best / distance_traveled_m
                if distance_traveled_m is not None and distance_traveled_m > 0.0
                else None
            ),
        }
    )
    if warnings:
        result["path_progress_warning"] = "; ".join(sorted(set(warnings)))
    return result


def _clean_points(points: Iterable[Point2D]) -> list[Point2D]:
    clean: list[Point2D] = []
    for point in points:
        try:
            x, y = float(point[0]), float(point[1])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            clean.append((x, y))
    return clean


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])
