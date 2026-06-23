"""Episode metrics for direct MuJoCo velocity-controller training and evaluation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Any
import csv
import json
import math


METRIC_FIELDS = (
    "checkpoint",
    "suite_index",
    "episode_id",
    "seed",
    "stage",
    "success",
    "final_status",
    "failure_phase",
    "failure_reason",
    "goal_time_s",
    "time_to_goal_s",
    "collision_count",
    "turn_collision_count",
    "straight_collision_count",
    "fall_count",
    "turn_fall_count",
    "straight_fall_count",
    "distance_travelled_m",
    "distance_traveled_m",
    "path_efficiency",
    "average_speed_mps",
    "turn_average_speed_mps",
    "straight_average_speed_mps",
    "max_speed_mps",
    "turn_max_speed_mps",
    "straight_max_speed_mps",
    "turn_entry_speed_mps",
    "turn_exit_speed_mps",
    "recovery_time_after_stuck_s",
    "route_turn_segment_count",
    "route_straight_segment_count",
    "completed_turn_segment_count",
    "completed_straight_segment_count",
    "failed_turn_segment_count",
    "failed_straight_segment_count",
    "backward_progress_events",
    "backward_progress_m",
    "max_backward_step_m",
    "backward_time_s",
    "reverse_command_steps",
    "reverse_command_time_s",
    "final_odom_position_error_m",
    "mean_odom_position_error_m",
    "max_odom_position_error_m",
    "final_odom_yaw_error_deg",
    "mean_odom_yaw_error_deg",
    "max_odom_yaw_error_deg",
    "command_jerk",
    "corridor_width_m",
    "friction",
    "scan_noise_std_m",
)


@dataclass
class EpisodeMetrics:
    """Serializable metrics emitted at the end of one RL episode."""

    suite_index: int
    episode_id: str
    seed: int
    stage: str
    success: bool
    final_status: str
    failure_phase: str
    failure_reason: str
    goal_time_s: float | None
    time_to_goal_s: float
    collision_count: int
    turn_collision_count: int
    straight_collision_count: int
    fall_count: int
    turn_fall_count: int
    straight_fall_count: int
    distance_travelled_m: float
    distance_traveled_m: float
    path_efficiency: float
    average_speed_mps: float
    turn_average_speed_mps: float
    straight_average_speed_mps: float
    max_speed_mps: float
    turn_max_speed_mps: float
    straight_max_speed_mps: float
    turn_entry_speed_mps: float
    turn_exit_speed_mps: float
    recovery_time_after_stuck_s: float
    route_turn_segment_count: int
    route_straight_segment_count: int
    completed_turn_segment_count: int
    completed_straight_segment_count: int
    failed_turn_segment_count: int
    failed_straight_segment_count: int
    backward_progress_events: int
    backward_progress_m: float
    max_backward_step_m: float
    backward_time_s: float
    reverse_command_steps: int
    reverse_command_time_s: float
    final_odom_position_error_m: float
    mean_odom_position_error_m: float
    max_odom_position_error_m: float
    final_odom_yaw_error_deg: float
    mean_odom_yaw_error_deg: float
    max_odom_yaw_error_deg: float
    command_jerk: float
    corridor_width_m: float
    friction: float
    scan_noise_std_m: float
    checkpoint: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def command_jerk(actions: Iterable[tuple[float, tuple[float, float, float]]]) -> float:
    """Return RMS action jerk from timestamped normalized actions."""
    rows = [(float(t), tuple(float(v) for v in action)) for t, action in actions]
    if len(rows) < 3:
        return 0.0
    accelerations: list[tuple[float, tuple[float, float, float]]] = []
    for previous, current in zip(rows, rows[1:]):
        dt = current[0] - previous[0]
        if dt <= 1e-9:
            continue
        accelerations.append((current[0], tuple((current[1][i] - previous[1][i]) / dt for i in range(3))))
    jerks: list[float] = []
    for previous, current in zip(accelerations, accelerations[1:]):
        dt = current[0] - previous[0]
        if dt <= 1e-9:
            continue
        jerk_vec = [(current[1][i] - previous[1][i]) / dt for i in range(3)]
        jerks.append(math.sqrt(sum(value * value for value in jerk_vec)))
    if not jerks:
        return 0.0
    return math.sqrt(sum(value * value for value in jerks) / len(jerks))


def summarize_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Aggregate per-episode metrics for one checkpoint."""
    if not rows:
        return {"episodes": 0, "success_rate": 0.0, "crash_or_fall_rate": 0.0, "score": float("inf")}
    successes = [row for row in rows if _as_bool(row.get("success"))]
    crash_or_fall = [
        row for row in rows
        if int(row.get("collision_count", 0)) > 0 or int(row.get("fall_count", 0)) > 0
    ]
    success_rate = len(successes) / len(rows)
    crash_or_fall_rate = len(crash_or_fall) / len(rows)
    goal_times = [
        float(row["goal_time_s"])
        for row in successes
        if _has_number(row.get("goal_time_s"))
    ]
    avg_success_time = _mean(goal_times) if goal_times else None
    avg_speed = _mean(float(row.get("average_speed_mps", 0.0)) for row in rows)
    return {
        "episodes": len(rows),
        "success_rate": success_rate,
        "successful_episodes": len(successes),
        "crash_or_fall_rate": crash_or_fall_rate,
        "collision_count": sum(int(row.get("collision_count", 0)) for row in rows),
        "turn_collision_count": sum(int(row.get("turn_collision_count", 0)) for row in rows),
        "straight_collision_count": sum(int(row.get("straight_collision_count", 0)) for row in rows),
        "fall_count": sum(int(row.get("fall_count", 0)) for row in rows),
        "turn_fall_count": sum(int(row.get("turn_fall_count", 0)) for row in rows),
        "straight_fall_count": sum(int(row.get("straight_fall_count", 0)) for row in rows),
        "route_turn_segment_count": sum(int(row.get("route_turn_segment_count", 0)) for row in rows),
        "route_straight_segment_count": sum(int(row.get("route_straight_segment_count", 0)) for row in rows),
        "completed_turn_segment_count": sum(int(row.get("completed_turn_segment_count", 0)) for row in rows),
        "completed_straight_segment_count": sum(int(row.get("completed_straight_segment_count", 0)) for row in rows),
        "failed_turn_segment_count": sum(int(row.get("failed_turn_segment_count", 0)) for row in rows),
        "failed_straight_segment_count": sum(int(row.get("failed_straight_segment_count", 0)) for row in rows),
        "backward_progress_events": sum(int(row.get("backward_progress_events", 0)) for row in rows),
        "backward_progress_m": sum(float(row.get("backward_progress_m", 0.0)) for row in rows),
        "reverse_command_steps": sum(int(row.get("reverse_command_steps", 0)) for row in rows),
        "reverse_command_time_s": sum(float(row.get("reverse_command_time_s", 0.0)) for row in rows),
        "avg_final_odom_position_error_m": _mean(float(row.get("final_odom_position_error_m", 0.0)) for row in rows),
        "avg_mean_odom_position_error_m": _mean(float(row.get("mean_odom_position_error_m", 0.0)) for row in rows),
        "max_odom_position_error_m": max((float(row.get("max_odom_position_error_m", 0.0)) for row in rows), default=0.0),
        "avg_final_odom_yaw_error_deg": _mean(float(row.get("final_odom_yaw_error_deg", 0.0)) for row in rows),
        "avg_mean_odom_yaw_error_deg": _mean(float(row.get("mean_odom_yaw_error_deg", 0.0)) for row in rows),
        "max_odom_yaw_error_deg": max((float(row.get("max_odom_yaw_error_deg", 0.0)) for row in rows), default=0.0),
        "avg_success_time_s": avg_success_time,
        "avg_goal_time_s": avg_success_time,
        "avg_distance_travelled_m": _mean(float(row.get("distance_travelled_m", 0.0)) for row in rows),
        "avg_speed_mps": avg_speed,
        "avg_turn_speed_mps": _mean(float(row.get("turn_average_speed_mps", 0.0)) for row in rows),
        "avg_straight_speed_mps": _mean(float(row.get("straight_average_speed_mps", 0.0)) for row in rows),
        "max_speed_mps": max((float(row.get("max_speed_mps", 0.0)) for row in rows), default=0.0),
        "turn_max_speed_mps": max((float(row.get("turn_max_speed_mps", 0.0)) for row in rows), default=0.0),
        "straight_max_speed_mps": max((float(row.get("straight_max_speed_mps", 0.0)) for row in rows), default=0.0),
        "avg_command_jerk": _mean(float(row.get("command_jerk", 0.0)) for row in rows),
        "status_counts": _counts(str(row.get("final_status", "")) for row in rows),
        "failure_phase_counts": _counts(str(row.get("failure_phase", "")) for row in rows if row.get("failure_phase")),
        "score": avg_success_time if avg_success_time is not None else float("inf"),
    }


def rank_checkpoint_summaries(
    summaries: dict[str, dict[str, Any]],
    *,
    min_success_rate: float,
    max_crash_or_fall_rate: float,
) -> list[dict[str, Any]]:
    """Rank checkpoints by fastest average successful run after safety gates."""
    ranked = []
    for checkpoint, summary in summaries.items():
        gated = (
            float(summary.get("success_rate", 0.0)) >= min_success_rate
            and float(summary.get("crash_or_fall_rate", 1.0)) <= max_crash_or_fall_rate
        )
        ranked.append({"checkpoint": checkpoint, "passes_gates": gated, **summary})
    return sorted(
        ranked,
        key=lambda row: (
            not bool(row["passes_gates"]),
            float(row.get("score", float("inf"))),
            -float(row.get("success_rate", 0.0)),
            float(row.get("crash_or_fall_rate", 1.0)),
        ),
    )


def write_metrics_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=METRIC_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _mean(values: Iterable[float]) -> float:
    rows = list(values)
    if not rows:
        return 0.0
    return sum(rows) / len(rows)


def _as_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "success"}
    return bool(value)


def _has_number(value: Any) -> bool:
    if value is None:
        return False
    if isinstance(value, str) and not value.strip():
        return False
    return True


def _counts(values: Iterable[str]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for value in values:
        counts[value] = counts.get(value, 0) + 1
    return counts
