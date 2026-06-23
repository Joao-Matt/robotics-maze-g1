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


def free_space_coverage_stats(slam_data: Sequence[int], truth_data: Sequence[int]) -> dict[str, int | float | str]:
    """Measure discovered SLAM cells only over ground-truth traversable maze space."""
    if len(slam_data) != len(truth_data):
        raise ValueError("SLAM and truth grids must have the same number of cells")
    pairs = [(int(slam), int(truth)) for slam, truth in zip(slam_data, truth_data)]
    valid_truth = [(slam, truth) for slam, truth in pairs if truth >= 0]
    truth_free = [(slam, truth) for slam, truth in valid_truth if truth == 0]
    truth_wall = [(slam, truth) for slam, truth in valid_truth if truth >= 65]
    known_free_space = sum(1 for slam, _ in truth_free if slam >= 0)
    known_truth_wall = sum(1 for slam, _ in truth_wall if slam >= 0)
    correct_free_space = sum(1 for slam, _ in truth_free if slam == 0)
    false_obstacle_in_free_space = sum(1 for slam, _ in truth_free if slam >= 65)
    total_free = len(truth_free)
    return {
        "coverage_scope": "ground_truth_free_cells",
        "coverage_fraction": known_free_space / total_free if total_free else 0.0,
        "free_space_coverage_fraction": known_free_space / total_free if total_free else 0.0,
        "truth_free_cells": total_free,
        "known_free_space_cells": known_free_space,
        "unknown_free_space_cells": max(0, total_free - known_free_space),
        "correct_free_space_cells": correct_free_space,
        "false_obstacle_in_free_space_cells": false_obstacle_in_free_space,
        "truth_wall_cells": len(truth_wall),
        "known_truth_wall_cells": known_truth_wall,
        "truth_evaluated_cells": len(valid_truth),
    }


def projected_free_space_coverage_stats(
    slam_data: Sequence[int],
    *,
    slam_width: int,
    slam_height: int,
    slam_origin_x: float,
    slam_origin_y: float,
    truth_data: Sequence[int],
    truth_width: int,
    truth_height: int,
    truth_origin_x: float,
    truth_origin_y: float,
    resolution: float,
) -> dict[str, int | float | str]:
    """Project a SLAM grid onto a fixed truth grid before free-space scoring."""
    if slam_width <= 0 or slam_height <= 0 or truth_width <= 0 or truth_height <= 0 or resolution <= 0.0:
        raise ValueError("grid dimensions and resolution must be positive")
    if len(slam_data) != slam_width * slam_height:
        raise ValueError("SLAM data length does not match SLAM grid dimensions")
    if len(truth_data) != truth_width * truth_height:
        raise ValueError("truth data length does not match truth grid dimensions")
    projected = [-1] * len(truth_data)
    for row in range(slam_height):
        y = slam_origin_y + (row + 0.5) * resolution
        truth_row = math.floor((y - truth_origin_y) / resolution)
        if truth_row < 0 or truth_row >= truth_height:
            continue
        for col in range(slam_width):
            x = slam_origin_x + (col + 0.5) * resolution
            truth_col = math.floor((x - truth_origin_x) / resolution)
            if 0 <= truth_col < truth_width:
                projected[truth_row * truth_width + truth_col] = int(slam_data[row * slam_width + col])
    stats = free_space_coverage_stats(projected, truth_data)
    stats["coverage_scope"] = "full_ground_truth_free_cells"
    return stats


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
    latency_search_s: float = 1.50,
    latency_step_s: float = 0.05,
) -> dict[str, float | int | None]:
    if not odom or not truth:
        return {"available": False, "aligned_samples": 0}
    truth_rows = sorted((float(t), float(x), float(y), float(yaw)) for t, x, y, yaw in truth)
    odom_rows = sorted((float(t), float(x), float(y), float(yaw)) for t, x, y, yaw in odom)
    rotation = truth_rows[0][3] - odom_rows[0][3]
    c, s = math.cos(rotation), math.sin(rotation)
    def aligned(time_offset_s: float) -> dict[str, object]:
        position_errors: list[float] = []
        x_errors: list[float] = []
        y_errors: list[float] = []
        yaw_errors: list[float] = []
        matched_truth: list[tuple[float, float, float]] = []
        predicted_positions: list[tuple[float, float]] = []
        matched_odom: list[tuple[float, float, float, float]] = []
        truth_index = 0
        for ot, ox, oy, oyaw in odom_rows:
            target_t = ot + time_offset_s
            while truth_index + 1 < len(truth_rows) and abs(truth_rows[truth_index + 1][0] - target_t) <= abs(truth_rows[truth_index][0] - target_t):
                truth_index += 1
            tt, tx, ty, tyaw = truth_rows[truth_index]
            if abs(tt - target_t) > max_time_delta_s:
                continue
            px = truth_rows[0][1] + c * (ox - odom_rows[0][1]) - s * (oy - odom_rows[0][2])
            py = truth_rows[0][2] + s * (ox - odom_rows[0][1]) + c * (oy - odom_rows[0][2])
            ex, ey = px - tx, py - ty
            position_errors.append(math.hypot(ex, ey))
            x_errors.append(ex); y_errors.append(ey)
            yaw_errors.append(abs(_wrap_angle(oyaw + rotation - tyaw)))
            matched_truth.append((tx, ty, tyaw))
            predicted_positions.append((px, py))
            matched_odom.append((ot, px, py, oyaw + rotation))
        return {
            "position_errors": position_errors,
            "x_errors": x_errors,
            "y_errors": y_errors,
            "yaw_errors": yaw_errors,
            "matched_truth": matched_truth,
            "predicted_positions": predicted_positions,
            "matched_odom": matched_odom,
        }

    zero = aligned(0.0)
    aligned_positions = list(zero["position_errors"])
    yaw_errors = list(zero["yaw_errors"])
    matched_truth = list(zero["matched_truth"])
    predicted_positions = list(zero["predicted_positions"])
    matched_odom = list(zero["matched_odom"])
    if not aligned_positions:
        return {"available": False, "aligned_samples": 0}
    odom_distance = path_length(predicted_positions)
    truth_distance = path_length([(row[0], row[1]) for row in matched_truth])
    latency_step = max(0.01, abs(float(latency_step_s)))
    steps = int(max(0.0, float(latency_search_s)) / latency_step)
    best_offset = 0.0
    best_rmse = _rms(aligned_positions)
    for step in range(-steps, steps + 1):
        offset = step * latency_step
        candidate = aligned(offset)
        candidate_errors = list(candidate["position_errors"])
        if len(candidate_errors) < max(3, min(10, len(aligned_positions) // 2)):
            continue
        candidate_rmse = _rms(candidate_errors)
        if candidate_rmse is not None and (best_rmse is None or candidate_rmse < best_rmse):
            best_rmse = candidate_rmse
            best_offset = offset
    odom_steps = []
    odom_speeds = []
    odom_yaw_steps = []
    odom_yaw_rates = []
    for previous, current in zip(matched_odom, matched_odom[1:]):
        dt = max(0.0, current[0] - previous[0])
        step = math.hypot(current[1] - previous[1], current[2] - previous[2])
        yaw_step = abs(_wrap_angle(current[3] - previous[3]))
        odom_steps.append(step)
        odom_yaw_steps.append(yaw_step)
        if dt > 1e-6:
            odom_speeds.append(step / dt)
            odom_yaw_rates.append(yaw_step / dt)
    truth_turn = sum(abs(_wrap_angle(current[2] - previous[2])) for previous, current in zip(matched_truth, matched_truth[1:]))
    position_rmse = _rms(aligned_positions)
    yaw_rmse = _rms(yaw_errors)
    return {
        "available": True,
        "aligned_samples": len(aligned_positions),
        "position_rmse_m": position_rmse,
        "position_mae_m": sum(aligned_positions) / len(aligned_positions),
        "position_p95_m": _percentile(aligned_positions, 0.95),
        "final_position_error_m": aligned_positions[-1],
        "x_rmse_m": _rms(list(zero["x_errors"])),
        "y_rmse_m": _rms(list(zero["y_errors"])),
        "final_x_error_m": list(zero["x_errors"])[-1],
        "final_y_error_m": list(zero["y_errors"])[-1],
        "position_rmse_per_meter": position_rmse / truth_distance if position_rmse is not None and truth_distance > 1e-6 else None,
        "final_position_error_per_meter": aligned_positions[-1] / truth_distance if truth_distance > 1e-6 else None,
        "yaw_rmse_rad": yaw_rmse,
        "yaw_p95_rad": _percentile(yaw_errors, 0.95),
        "yaw_rmse_deg": math.degrees(yaw_rmse) if yaw_rmse is not None else None,
        "yaw_p95_deg": math.degrees(_percentile(yaw_errors, 0.95)) if yaw_errors else None,
        "final_yaw_error_deg": math.degrees(yaw_errors[-1]),
        "truth_turn_magnitude_deg": math.degrees(truth_turn),
        "yaw_rmse_deg_per_meter": math.degrees(yaw_rmse) / truth_distance if yaw_rmse is not None and truth_distance > 1e-6 else None,
        "odom_distance_m": odom_distance,
        "truth_distance_m": truth_distance,
        "distance_scale": odom_distance / truth_distance if truth_distance > 1e-6 else None,
        "estimated_time_offset_s": best_offset,
        "latency_corrected_position_rmse_m": best_rmse,
        "max_odom_step_m": max(odom_steps) if odom_steps else None,
        "max_odom_speed_mps": max(odom_speeds) if odom_speeds else None,
        "max_odom_yaw_step_deg": math.degrees(max(odom_yaw_steps)) if odom_yaw_steps else None,
        "max_odom_yaw_rate_degps": math.degrees(max(odom_yaw_rates)) if odom_yaw_rates else None,
        "sudden_translation_jump_count": sum(1 for step in odom_steps if step > 0.35),
        "sudden_yaw_jump_count": sum(1 for step in odom_yaw_steps if step > math.radians(25.0)),
    }


def _rms(values: Sequence[float]) -> float | None:
    if not values:
        return None
    return math.sqrt(sum(float(value) * float(value) for value in values) / len(values))


def _percentile(values: Sequence[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, float(q))) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def _wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi
