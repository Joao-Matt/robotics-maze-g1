"""Reusable Lucky Walker start-to-goal motion session for ROS and scripted runs."""

from __future__ import annotations

from pathlib import Path
import math

from maze.generator import generate_maze_from_config
from nav.controller import pose_from_base_state
from nav.oracle_follow import FAILED, GOAL_REACHED, TurnAwareFollowerConfig, TurnAwareOracleFollower, build_turn_aware_path
from nav.planner import plan_oracle_path
from sim.locomotion_policy_adapter import VelocityCommand, create_policy_adapter
from sim.locomotion_sandbox import base_state, config_from_dict, determine_status
from sim.world_builder import cell_to_world_xy


def oracle_stop_decision(*, follower_state: str, fallen: bool, zero_duration_s: float,
                         zero_timeout_s: float, timed_out: bool) -> tuple[str, str] | None:
    """Apply deterministic stop precedence: safety, goal/failure, stall, duration."""
    if fallen:
        return "FALL_DETECTED", "fall_detected"
    if follower_state == GOAL_REACHED:
        return GOAL_REACHED, "goal_reached"
    if follower_state == FAILED:
        return FAILED, "follower_failed"
    if zero_duration_s >= zero_timeout_s:
        return "ZERO_COMMAND_TIMEOUT", "zero_oracle_command_timeout"
    if timed_out:
        return "TIMEOUT", "duration_timeout"
    return None


def follower_config(config: dict) -> TurnAwareFollowerConfig:
    values = config.get("oracle", {})
    return TurnAwareFollowerConfig(
        approach_tolerance_m=float(values.get("approach_tolerance_m", 0.35)),
        waypoint_tolerance_m=float(values.get("waypoint_tolerance_m", 0.75)),
        goal_tolerance_m=float(values.get("goal_tolerance_m", 0.5)),
        heading_threshold_rad=float(values.get("heading_threshold_rad", 0.45)),
        forward_speed_mps=float(values.get("forward_speed_mps", 0.8)),
        heading_gain=float(values.get("heading_gain", 1.4)),
        max_yaw_rate_radps=float(values.get("max_yaw_rate_radps", 0.8)),
        turn_start_distance_m=float(values.get("turn_start_distance_m", 0.8)),
        pre_turn_distance_m=float(values.get("pre_turn_distance_m", 0.8)),
        arc_turn_forward_speed_mps=float(values.get("arc_turn_forward_speed_mps", 0.4)),
        arc_turn_yaw_rate_radps=float(values.get("arc_turn_yaw_rate_radps", 0.8)),
        post_turn_heading_tolerance_rad=float(values.get("post_turn_heading_tolerance_rad", 0.3)),
        stuck_timeout_s=float(values.get("stuck_timeout_s", 8.0)),
        stuck_min_progress_m=float(values.get("stuck_min_progress_m", 0.08)),
        max_recovery_attempts=int(values.get("max_recovery_attempts", 2)),
    )


class OracleMotionSession:
    def __init__(self, mujoco, model, data, config: dict, seed: int, lucky_repo: Path, model_xml: Path, duration_s: float, zero_command_timeout_s: float = 20.0) -> None:
        self.mujoco, self.model, self.data = mujoco, model, data
        self.config = config
        self.maze = generate_maze_from_config(config, seed)
        values = config.get("oracle", {})
        plan = plan_oracle_path(
            self.maze, safety_radius_m=float(config["robot"]["safety_radius_m"]),
            simplify=False, planner="heading_astar", turn_penalty_cost=float(values.get("turn_penalty_cost", 2.0)),
        )
        self.path = build_turn_aware_path(self.maze, plan.cells, follower_config(config))
        self.follower = TurnAwareOracleFollower(self.path, follower_config(config))
        self.adapter = create_policy_adapter("lucky_walker", lucky_g1_repo=lucky_repo)
        report = self.adapter.compatibility_report(model, model_xml)
        if report.errors:
            raise RuntimeError("; ".join(report.errors))
        start_x, start_y = cell_to_world_xy(self.maze, self.maze.spec.start_cell)
        self.adapter.reset_at_pose(model, data, start_x, start_y, self.path.segments[0].target_heading_rad)
        mujoco.mj_forward(model, data)
        self.sandbox = config_from_dict(config)
        self.control_dt = 1.0 / self.sandbox.control_rate_hz
        self.substeps = max(1, round(self.control_dt / float(model.opt.timestep)))
        self.end_time = float(data.time) + duration_s
        self.status = "RUNNING"
        self.start_xy = (start_x, start_y)
        self.distance_traveled_m = 0.0
        self.last_xy = self.start_xy
        self.last_command = VelocityCommand()
        self.zero_command_timeout_s = zero_command_timeout_s
        self.zero_command_since: float | None = None
        self.stop_reason = "running"

    def step(self) -> None:
        if self.status != "RUNNING":
            return
        state = base_state(self.data)
        output = self.follower.update(pose_from_base_state(state), float(self.data.time))
        command = output.command
        command_is_zero = abs(command.vx) < 0.001 and abs(command.yaw_rate) < 0.001
        if command_is_zero:
            if self.zero_command_since is None:
                self.zero_command_since = float(self.data.time)
        else:
            self.zero_command_since = None
        zero_duration = float(self.data.time) - self.zero_command_since if self.zero_command_since is not None else 0.0
        fallen = determine_status(command, state, self.sandbox) == "fallen"
        decision = oracle_stop_decision(
            follower_state=output.state, fallen=fallen, zero_duration_s=zero_duration,
            zero_timeout_s=self.zero_command_timeout_s, timed_out=self.data.time >= self.end_time,
        )
        if decision is not None:
            self.status, self.stop_reason = decision
        if fallen:
            command = VelocityCommand()
            self.data.ctrl[:] = 0.0
        elif decision is None:
            self.adapter.step(self.model, self.data, command, self.control_dt)
        self.last_command = command
        for _ in range(self.substeps):
            self.mujoco.mj_step(self.model, self.data)
        state = base_state(self.data)
        xy = (float(state["base_x"]), float(state["base_y"]))
        self.distance_traveled_m += math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
        self.last_xy = xy
        if self.data.time >= self.end_time and self.status == "RUNNING":
            self.status, self.stop_reason = oracle_stop_decision(
                follower_state=output.state, fallen=False, zero_duration_s=zero_duration,
                zero_timeout_s=self.zero_command_timeout_s, timed_out=True,
            )

    def summary(self) -> dict[str, object]:
        return {
            "status": self.status,
            "distance_traveled_m": self.distance_traveled_m,
            "segment_index": self.follower.segment_index,
            "segment_count": len(self.path.segments),
            "sim_time_s": float(self.data.time),
            "stop_reason": self.stop_reason,
            "oracle_command": {"linear_x": self.last_command.vx, "angular_z": self.last_command.yaw_rate},
        }
