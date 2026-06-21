"""Small pure helpers for live KPI aggregation.

These helpers intentionally avoid ROS imports so they can be unit-tested
without a ROS 2 runtime.
"""

from __future__ import annotations

import math
from statistics import median
from typing import Iterable, Sequence


def finite(value: object) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def message_rate(times: Sequence[float], now: float | None = None, window_s: float = 5.0) -> float:
    if len(times) < 2:
        return 0.0
    end = float(times[-1] if now is None else now)
    start = end - max(0.001, float(window_s))
    recent = [float(t) for t in times if float(t) >= start]
    if len(recent) < 2:
        return 0.0
    span = max(1e-9, recent[-1] - recent[0])
    return (len(recent) - 1) / span


def drop_fraction(rate_hz: float, target_hz: float | None) -> float | None:
    if target_hz is None or target_hz <= 0.0:
        return None
    return max(0.0, min(1.0, (float(target_hz) - float(rate_hz)) / float(target_hz)))


def path_length(points: Iterable[tuple[float, float]]) -> float:
    previous = None
    total = 0.0
    for point in points:
        x, y = float(point[0]), float(point[1])
        if previous is not None:
            total += math.hypot(x - previous[0], y - previous[1])
        previous = (x, y)
    return total


def occupancy_stats(data: Sequence[int]) -> dict[str, int | float]:
    total = len(data)
    known = sum(1 for value in data if int(value) >= 0)
    free = sum(1 for value in data if int(value) == 0)
    occupied = sum(1 for value in data if int(value) >= 65)
    unknown = max(0, total - known)
    return {
        "total_cells": total,
        "known_cells": known,
        "free_cells": free,
        "occupied_cells": occupied,
        "unknown_cells": unknown,
        "coverage_fraction": known / total if total else 0.0,
    }


def scan_clearance(ranges: Sequence[float], range_min: float, range_max: float) -> dict[str, float | int | None]:
    valid = [float(value) for value in ranges if math.isfinite(float(value)) and range_min <= float(value) <= range_max]
    invalid_count = len(ranges) - len(valid)
    if not valid:
        return {"min_clearance_m": None, "median_clearance_m": None, "valid_ranges": 0, "invalid_ranges": invalid_count}
    return {
        "min_clearance_m": min(valid),
        "median_clearance_m": float(median(valid)),
        "valid_ranges": len(valid),
        "invalid_ranges": invalid_count,
    }


def command_smoothness(rows: Sequence[tuple[float, float, float]]) -> dict[str, float | None]:
    if len(rows) < 3:
        return {
            "linear_accel_rms_mps2": None,
            "yaw_accel_rms_radps2": None,
            "linear_jerk_rms_mps3": None,
            "yaw_jerk_rms_radps3": None,
        }
    accelerations: list[tuple[float, float, float]] = []
    for previous, current in zip(rows, rows[1:]):
        dt = float(current[0]) - float(previous[0])
        if dt <= 1e-6:
            continue
        accelerations.append((float(current[0]), (float(current[1]) - float(previous[1])) / dt, (float(current[2]) - float(previous[2])) / dt))
    jerks: list[tuple[float, float]] = []
    for previous, current in zip(accelerations, accelerations[1:]):
        dt = float(current[0]) - float(previous[0])
        if dt <= 1e-6:
            continue
        jerks.append(((float(current[1]) - float(previous[1])) / dt, (float(current[2]) - float(previous[2])) / dt))
    return {
        "linear_accel_rms_mps2": _rms([row[1] for row in accelerations]),
        "yaw_accel_rms_radps2": _rms([row[2] for row in accelerations]),
        "linear_jerk_rms_mps3": _rms([row[0] for row in jerks]),
        "yaw_jerk_rms_radps3": _rms([row[1] for row in jerks]),
    }


def localization_metrics(
    odom: Sequence[tuple[float, float, float, float]],
    truth: Sequence[tuple[float, float, float, float]],
    max_time_delta_s: float = 0.25,
) -> dict[str, float | int | None]:
    if not odom or not truth:
        return {"available": False, "aligned_samples": 0}
    truth_rows = sorted((float(t), float(x), float(y), float(yaw)) for t, x, y, yaw in truth)
    odom_rows = sorted((float(t), float(x), float(y), float(yaw)) for t, x, y, yaw in odom)
    rotation = truth_rows[0][3] - odom_rows[0][3]
    c, s = math.cos(rotation), math.sin(rotation)
    aligned_positions: list[float] = []
    yaw_errors: list[float] = []
    matched_truth: list[tuple[float, float]] = []
    predicted_positions: list[tuple[float, float]] = []
    truth_index = 0
    for ot, ox, oy, oyaw in odom_rows:
        while truth_index + 1 < len(truth_rows) and abs(truth_rows[truth_index + 1][0] - ot) <= abs(truth_rows[truth_index][0] - ot):
            truth_index += 1
        tt, tx, ty, tyaw = truth_rows[truth_index]
        if abs(tt - ot) > max_time_delta_s:
            continue
        px = truth_rows[0][1] + c * (ox - odom_rows[0][1]) - s * (oy - odom_rows[0][2])
        py = truth_rows[0][2] + s * (ox - odom_rows[0][1]) + c * (oy - odom_rows[0][2])
        aligned_positions.append(math.hypot(px - tx, py - ty))
        yaw_errors.append(abs(_wrap_angle(oyaw + rotation - tyaw)))
        matched_truth.append((tx, ty))
        predicted_positions.append((px, py))
    if not aligned_positions:
        return {"available": False, "aligned_samples": 0}
    odom_distance = path_length(predicted_positions)
    truth_distance = path_length(matched_truth)
    return {
        "available": True,
        "aligned_samples": len(aligned_positions),
        "position_rmse_m": _rms(aligned_positions),
        "position_mae_m": sum(aligned_positions) / len(aligned_positions),
        "final_position_error_m": aligned_positions[-1],
        "yaw_rmse_rad": _rms(yaw_errors),
        "odom_distance_m": odom_distance,
        "truth_distance_m": truth_distance,
        "distance_scale": odom_distance / truth_distance if truth_distance > 1e-6 else None,
    }


def _rms(values: Sequence[float]) -> float | None:
    if not values:
        return None
    return math.sqrt(sum(float(value) * float(value) for value in values) / len(values))


def _wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi
