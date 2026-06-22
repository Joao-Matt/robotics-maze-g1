"""Gymnasium environment for direct MuJoCo G1 maze velocity-control RL."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any
import json
import math

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from maze.validator import raise_for_invalid, validate_maze
from nav.controller import Pose2D, pose_from_base_state
from nav.oracle_follow import ARC_TURN, build_turn_aware_path
from nav.planner import plan_oracle_path
from rl_velocity import ACTION_DIM, OBSERVATION_DIM
from rl_velocity.config import load_rl_config
from rl_velocity.curriculum import StageSpec, config_for_stage, load_stage_specs, maze_for_stage, select_stage
from rl_velocity.metrics import EpisodeMetrics, command_jerk
from rl_velocity.path_features import OraclePathFeatureExtractor, PathProjection
from sim.config import load_config
from sim.locomotion_policy_adapter import VelocityCommand, create_policy_adapter
from sim.locomotion_sandbox import base_state, config_from_dict, determine_status, save_render
from sim.mujoco_runner import PROJECT_ROOT, import_mujoco
from sim.oracle_motion_session import follower_config
from sim.world_builder import build_maze_world, cell_to_world_xy


class G1MazeVelocityEnv(gym.Env):
    """Oracle-conditioned high-level velocity controller for the Unitree G1 walker."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        *,
        config_path: str | Path = PROJECT_ROOT / "configs" / "default.yaml",
        rl_config_path: str | Path = PROJECT_ROOT / "configs" / "rl_velocity_controller.yaml",
        run_dir: str | Path | None = None,
        stage: str | None = None,
        seed: int | None = None,
        unitree_rl_gym_repo: str | Path | None = None,
        training: bool = True,
        record_trajectory: bool = False,
        episode_plan: list[dict[str, Any]] | None = None,
        locomotion_calibration_path: str | Path | None = None,
    ) -> None:
        super().__init__()
        self.config_path = Path(config_path)
        self.rl_config_path = Path(rl_config_path)
        self.base_config = load_config(self.config_path)
        self.rl_config = load_rl_config(self.rl_config_path)
        self.stages = load_stage_specs(self.rl_config)
        self.requested_stage = stage
        self.training = bool(training)
        self.record_trajectory = bool(record_trajectory)
        self.episode_plan = list(episode_plan or [])
        self.episode_plan_index = 0
        self.run_dir = Path(run_dir) if run_dir is not None else PROJECT_ROOT / ".tmp" / "rl_velocity_env"
        self.world_cache_dir = self.run_dir / "world_cache"
        self.unitree_rl_gym_repo = Path(unitree_rl_gym_repo) if unitree_rl_gym_repo is not None else None
        self.rng = np.random.default_rng(seed)
        self.base_seed = int(seed if seed is not None else 0)
        self.episode_index = 0

        self.command_limits = dict(self.rl_config.get("command_limits", {}))
        self.locomotion_calibration_path = Path(locomotion_calibration_path) if locomotion_calibration_path else None
        self.locomotion_calibration: dict[str, Any] | None = None
        if self.locomotion_calibration_path is not None:
            self.locomotion_calibration = load_locomotion_calibration(self.locomotion_calibration_path)
            self.command_limits = apply_locomotion_calibration_to_command_limits(
                self.command_limits,
                self.locomotion_calibration,
            )
        self.reward_weights = self.rl_config.get("rewards", {})
        self.termination = self.rl_config.get("termination", {})
        self.sensor_config = self.rl_config.get("sensors", {})
        self.action_rate_hz = float(self.rl_config.get("action_rate_hz", 10.0))
        self.action_dt = 1.0 / max(1e-6, self.action_rate_hz)

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(ACTION_DIM,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=np.full((OBSERVATION_DIM,), -np.inf, dtype=np.float32),
            high=np.full((OBSERVATION_DIM,), np.inf, dtype=np.float32),
            dtype=np.float32,
        )

        self.mujoco: Any | None = None
        self.model: Any | None = None
        self.data: Any | None = None
        self.adapter: Any | None = None
        self.active_config: dict[str, Any] | None = None
        self.active_stage: StageSpec | None = None
        self.features: OraclePathFeatureExtractor | None = None
        self.projection: PathProjection | None = None
        self.previous_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self.last_episode_metrics: EpisodeMetrics | None = None
        self.trajectory: list[dict[str, float | str | int]] = []

        self._reset_episode_bookkeeping()

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        options = options or {}
        planned = self._next_planned_episode()
        requested_stage = str(
            options.get("stage", planned.get("stage", self.requested_stage or ""))
        ) or None
        self.active_stage = select_stage(self.stages, self.rng, requested_stage)
        if "corridor_width_m" in planned:
            self.active_stage = replace(self.active_stage, cell_size_m=float(planned["corridor_width_m"]))
        episode_seed = int(options.get("seed", planned.get("seed", self.rng.integers(0, 2**31 - 1))))
        episode_id = str(planned.get("id", f"{self.active_stage.name}_seed-{episode_seed}"))
        suite_index = int(planned.get("suite_index", self.episode_plan_index - 1 if planned else -1))
        self._load_episode_world(self.active_stage, episode_seed)
        self._reset_episode_bookkeeping(seed=episode_seed)
        self.episode_id = episode_id
        self.suite_index = suite_index
        self.projection = self._project_pose()
        self.previous_progress_m = self.projection.progress_m
        self.best_progress_m = self.projection.progress_m
        self.last_progress_time_s = float(self.data.time)
        observation = self._build_observation()
        info = {
            "seed": episode_seed,
            "stage": self.active_stage.name,
            "episode_id": self.episode_id,
            "suite_index": self.suite_index,
            "oracle_path_length_m": self.features.total_length_m,
            "command_limits": dict(self.command_limits),
            "locomotion_calibration_path": str(self.locomotion_calibration_path) if self.locomotion_calibration_path else None,
        }
        return observation, info

    def step(self, action: np.ndarray):
        self._ensure_ready()
        normalized = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        if self.active_stage and self.active_stage.action_noise_std > 0.0 and self.training:
            normalized = np.clip(
                normalized + self.rng.normal(0.0, self.active_stage.action_noise_std, size=ACTION_DIM),
                -1.0,
                1.0,
            ).astype(np.float32)
        command = self._action_to_command(normalized)
        previous_projection = self.projection or self._project_pose()
        previous_xy = self._xy()
        previous_time = float(self.data.time)
        self._simulate_command(command)
        state = base_state(self.data)
        pose = pose_from_base_state(state)
        projection = self.features.project(pose)
        progress_delta = projection.progress_m - previous_projection.progress_m
        self.projection = projection
        self.max_segment_index_seen = max(self.max_segment_index_seen, projection.segment_index)
        now = float(self.data.time)
        self.episode_step_count += 1
        self.elapsed_s = now - self.episode_start_time_s
        distance_delta = math.hypot(self._xy()[0] - previous_xy[0], self._xy()[1] - previous_xy[1])
        dt = max(0.0, now - previous_time)
        self.distance_traveled_m += distance_delta
        self.max_speed_mps = max(self.max_speed_mps, self._ground_speed())
        self._record_motion_sample(projection, distance_delta, dt)
        self._record_backward_sample(progress_delta, command, dt)
        self.action_history.append((now, tuple(float(value) for value in normalized)))
        self._update_turn_metrics(projection)
        self._update_stuck_watch(progress_delta, command, now)

        fallen = determine_status(command, state, self.sandbox_config) == "fallen"
        success = self._success(projection)
        timed_out = self.elapsed_s >= float(self.termination.get("max_episode_time_s", 120.0))
        stuck = self.stuck_timeout_exceeded
        collision_terminal = bool(self.termination.get("terminate_on_wall_contact", True)) and self.wall_contact_this_step
        terminated = bool(success or fallen or collision_terminal or stuck)
        truncated = bool(timed_out and not terminated)
        status = self._final_status(success, fallen, collision_terminal, stuck, truncated)
        reward = self._reward(
            projection=projection,
            progress_delta=progress_delta,
            action=normalized,
            command=command,
            status=status,
            terminated=terminated,
            truncated=truncated,
        )

        if fallen:
            self.fall_count += 1
            if self._is_turn_projection(projection):
                self.turn_fall_count += 1
            else:
                self.straight_fall_count += 1
        self.previous_action = normalized.copy()
        observation = self._build_observation()
        info: dict[str, Any] = {
            "status": status,
            "success": success,
            "stage": self.active_stage.name if self.active_stage else "",
            "seed": self.episode_seed,
            "episode_id": self.episode_id,
            "suite_index": self.suite_index,
            "progress_m": projection.progress_m,
            "distance_to_goal_m": projection.distance_to_goal_m,
            "wall_contact": self.wall_contact_this_step,
        }
        if self.record_trajectory:
            self._record_trajectory_row(status, command, reward)
        if terminated or truncated:
            metrics = self._episode_metrics(status=status, success=success)
            self.last_episode_metrics = metrics
            info["episode_metrics"] = metrics.to_dict()
        return observation, float(reward), terminated, truncated, info

    def close(self) -> None:
        self.model = None
        self.data = None

    def _next_planned_episode(self) -> dict[str, Any]:
        if not self.episode_plan:
            return {}
        index = self.episode_plan_index % len(self.episode_plan)
        planned = dict(self.episode_plan[index])
        planned.setdefault("suite_index", index)
        self.episode_plan_index += 1
        return planned

    def save_final_render(self, path: Path, width: int = 960, height: int = 720) -> None:
        """Save the current MuJoCo camera render to a PNG path."""
        self._ensure_ready()
        save_render(self.mujoco, self.model, self.data, path, width, height)

    def _load_episode_world(self, stage: StageSpec, episode_seed: int) -> None:
        self.active_config = config_for_stage(self.base_config, stage)
        maze = maze_for_stage(stage, episode_seed)
        validation = validate_maze(
            maze,
            safety_radius_m=float(self.active_config["robot"]["safety_radius_m"]),
            min_corridor_width_m=float(self.active_config["maze"]["min_corridor_width_m"]),
            max_corridor_width_m=float(self.active_config["maze"].get("max_corridor_width_m", 4.0)),
        )
        raise_for_invalid(validation)
        oracle_values = self.active_config.get("oracle", {})
        plan = plan_oracle_path(
            maze,
            safety_radius_m=float(self.active_config["robot"]["safety_radius_m"]),
            simplify=False,
            planner="heading_astar",
            turn_penalty_cost=float(oracle_values.get("turn_penalty_cost", 2.0)),
        )
        turn_path = build_turn_aware_path(maze, plan.cells, follower_config(self.active_config))
        self.features = OraclePathFeatureExtractor(turn_path)
        width_label = f"width-{stage.cell_size_m:.2f}".replace(".", "_")
        world_dir = self.world_cache_dir / stage.name / width_label / f"seed-{episode_seed}"
        world = build_maze_world(self.active_config, episode_seed, world_dir, maze=maze)
        self.mujoco = self.mujoco or import_mujoco()
        self.model = self.mujoco.MjModel.from_xml_path(str(world.model_xml_path))
        self.data = self.mujoco.MjData(self.model)
        self._apply_random_friction(stage)
        self.adapter = self.adapter or create_policy_adapter(
            "unitree_rl_gym_native",
            unitree_rl_gym_repo=self.unitree_rl_gym_repo,
        )
        report = self.adapter.compatibility_report(self.model, Path(world.model_xml_path))
        if report.errors:
            raise RuntimeError("; ".join(report.errors))
        start_x, start_y = cell_to_world_xy(maze, maze.spec.start_cell)
        start_yaw = turn_path.segments[0].target_heading_rad
        self._reset_policy_at_pose(start_x, start_y, start_yaw)
        self.mujoco.mj_forward(self.model, self.data)
        self.sandbox_config = config_from_dict(self.active_config)
        self.episode_seed = episode_seed
        self.sampled_friction = getattr(self, "sampled_friction", 0.8)

    def _reset_policy_at_pose(self, x: float, y: float, yaw: float) -> None:
        self.adapter.reset(self.model, self.data)
        base_height = float(self.active_config.get("robot", {}).get("initial_base_height_m", self.data.qpos[2]))
        self.data.qpos[0] = x
        self.data.qpos[1] = y
        self.data.qpos[2] = base_height
        self.data.qpos[3:7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]

    def _reset_episode_bookkeeping(self, seed: int = 0) -> None:
        self.episode_seed = int(seed)
        self.episode_id = ""
        self.suite_index = -1
        self.episode_start_time_s = float(self.data.time) if self.data is not None else 0.0
        self.elapsed_s = 0.0
        self.episode_step_count = 0
        self.previous_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self.previous_progress_m = 0.0
        self.max_segment_index_seen = 0
        self.best_progress_m = 0.0
        self.last_progress_time_s = 0.0
        self.stuck_timeout_exceeded = False
        self.wall_contact_active = False
        self.wall_contact_this_step = False
        self.collision_count = 0
        self.turn_collision_count = 0
        self.straight_collision_count = 0
        self.fall_count = 0
        self.turn_fall_count = 0
        self.straight_fall_count = 0
        self.distance_traveled_m = 0.0
        self.turn_distance_m = 0.0
        self.straight_distance_m = 0.0
        self.turn_time_s = 0.0
        self.straight_time_s = 0.0
        self.max_speed_mps = 0.0
        self.turn_max_speed_mps = 0.0
        self.straight_max_speed_mps = 0.0
        self.backward_progress_events = 0
        self.backward_progress_m = 0.0
        self.max_backward_step_m = 0.0
        self.backward_time_s = 0.0
        self.reverse_command_steps = 0
        self.reverse_command_time_s = 0.0
        self.turn_entry_speeds: list[float] = []
        self.turn_exit_speeds: list[float] = []
        self.previous_segment_state = ""
        self.stuck_recovery_time_s = 0.0
        self.stuck_active_since: float | None = None
        self.action_history: list[tuple[float, tuple[float, float, float]]] = []
        self.trajectory = []
        self.last_episode_metrics = None

    def _simulate_command(self, command: VelocityCommand) -> None:
        self.wall_contact_this_step = False
        action_steps = max(1, round(self.action_dt / float(self.model.opt.timestep)))
        substep_dt = float(self.model.opt.timestep)
        requires_substep = bool(getattr(self.adapter, "requires_substep_control", False))
        if not requires_substep:
            self.adapter.step(self.model, self.data, command, self.action_dt)
        for _ in range(action_steps):
            if requires_substep:
                self.adapter.step(self.model, self.data, command, substep_dt)
            self.mujoco.mj_step(self.model, self.data)
            self._update_wall_contacts()

    def _build_observation(self) -> np.ndarray:
        self._ensure_ready()
        state = base_state(self.data)
        pose = pose_from_base_state(state)
        projection = self.features.project(pose)
        rays = self._ray_distances()
        side_width = self._local_corridor_width(rays)
        body_vx, body_vy, yaw_rate = self._body_velocity()
        max_ray = float(self.sensor_config.get("ray_range_max_m", 8.0))
        max_path = max(self.features.total_length_m, 1e-6)
        obs = np.zeros(OBSERVATION_DIM, dtype=np.float32)
        obs[0:5] = np.asarray([body_vx, body_vy, yaw_rate, state["roll"], state["pitch"]], dtype=np.float32)
        obs[5] = _norm_clip(projection.distance_to_goal_m, max_path)
        obs[6] = math.sin(projection.angle_to_goal_rad)
        obs[7] = math.cos(projection.angle_to_goal_rad)
        obs[8] = _norm_clip(projection.distance_to_segment_end_m, max_path)
        obs[9] = math.sin(projection.heading_error_rad)
        obs[10] = math.cos(projection.heading_error_rad)
        obs[11] = _norm_clip(projection.distance_to_next_corner_m, max_path)
        obs[12] = math.sin(projection.next_corner_angle_rad)
        obs[13] = math.cos(projection.next_corner_angle_rad)
        obs[14] = _norm_clip(side_width, max_ray)
        obs[15] = _norm_clip(projection.progress_m, max_path)
        obs[16] = _signed_norm_clip(projection.lateral_error_m, max(0.1, side_width * 0.5))
        obs[17:53] = np.asarray(rays, dtype=np.float32) / max_ray
        obs[53:56] = self.previous_action
        return np.nan_to_num(obs, nan=0.0, posinf=1.0, neginf=-1.0).astype(np.float32)

    def _ray_distances(self) -> np.ndarray:
        bins = int(self.sensor_config.get("ray_count", 36))
        if bins != 36:
            bins = 36
        range_min = float(self.sensor_config.get("ray_range_min_m", 0.10))
        range_max = float(self.sensor_config.get("ray_range_max_m", 8.0))
        mount = np.asarray(self.sensor_config.get("ray_mount_pos_m", [0.05, 0.0, 0.30]), dtype=np.float64)
        pelvis_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        yaw = base_state(self.data)["yaw"]
        cy, sy = math.cos(yaw), math.sin(yaw)
        planar_rotation = np.asarray(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)), dtype=np.float64)
        origin = self.data.xpos[pelvis_id] + planar_rotation @ mount
        geom_groups = np.asarray([0, 0, 0, 0, 1, 0], dtype=np.uint8)
        geom_id = np.asarray([-1], dtype=np.int32)
        ranges = np.full(bins, range_max, dtype=np.float32)
        for index in range(bins):
            angle = -math.pi + index * (2.0 * math.pi / bins) + yaw
            direction = np.asarray((math.cos(angle), math.sin(angle), 0.0), dtype=np.float64)
            distance = float(self.mujoco.mj_ray(self.model, self.data, origin, direction, geom_groups, 1, -1, geom_id))
            if range_min <= distance <= range_max:
                ranges[index] = distance
        noise_std = self.active_stage.scan_noise_std_m if self.active_stage else 0.0
        if noise_std > 0.0 and self.training:
            ranges += self.rng.normal(0.0, noise_std, size=bins).astype(np.float32)
            ranges = np.clip(ranges, range_min, range_max)
        return ranges

    def _reward(
        self,
        *,
        projection: PathProjection,
        progress_delta: float,
        action: np.ndarray,
        command: VelocityCommand,
        status: str,
        terminated: bool,
        truncated: bool,
    ) -> float:
        weights = self.reward_weights
        reward = float(weights.get("progress", 8.0)) * progress_delta
        aligned_speed = max(0.0, progress_delta / max(self.action_dt, 1e-6)) * max(0.0, math.cos(projection.heading_error_rad))
        reward += float(weights.get("aligned_speed", 0.25)) * aligned_speed * self.action_dt
        tilt = max(0.0, abs(base_state(self.data)["roll"]) - 0.35) + max(0.0, abs(base_state(self.data)["pitch"]) - 0.35)
        reward -= float(weights.get("tilt", 0.35)) * tilt
        jerk = float(np.linalg.norm(action - self.previous_action))
        reward -= float(weights.get("command_jerk", 0.05)) * jerk
        if self.wall_contact_this_step:
            reward -= float(weights.get("wall_contact", 8.0))
        if abs(command.yaw_rate) > float(self.termination.get("spin_yaw_rate_radps", 0.75)) and progress_delta < 0.005:
            reward -= float(weights.get("spinning_without_progress", 0.08))
        if status == "SUCCESS":
            reward += float(weights.get("goal_bonus", 60.0))
        elif status == "FALL":
            reward -= float(weights.get("fall", 50.0))
        elif status == "COLLISION":
            reward -= float(weights.get("collision", 40.0))
        elif status == "STUCK":
            reward -= float(weights.get("stuck", 12.0))
        elif truncated:
            reward -= float(weights.get("timeout", 8.0))
        return reward

    def _episode_metrics(self, *, status: str, success: bool) -> EpisodeMetrics:
        elapsed = max(float(self.elapsed_s), 1e-6)
        progress = self.projection.progress_m if self.projection is not None else 0.0
        efficiency = progress / max(self.distance_traveled_m, 1e-6)
        segment_counts = self._segment_counts(success=success)
        failure_phase = "" if success else ("turn" if self._is_turn_phase() else "straight")
        return EpisodeMetrics(
            suite_index=int(self.suite_index),
            episode_id=str(self.episode_id),
            seed=int(self.episode_seed),
            stage=self.active_stage.name if self.active_stage else "",
            success=bool(success),
            final_status=status,
            failure_phase=failure_phase,
            failure_reason="" if success else status,
            goal_time_s=float(elapsed) if success else None,
            time_to_goal_s=float(elapsed),
            collision_count=int(self.collision_count),
            turn_collision_count=int(self.turn_collision_count),
            straight_collision_count=int(self.straight_collision_count),
            fall_count=int(self.fall_count),
            turn_fall_count=int(self.turn_fall_count),
            straight_fall_count=int(self.straight_fall_count),
            distance_travelled_m=float(self.distance_traveled_m),
            distance_traveled_m=float(self.distance_traveled_m),
            path_efficiency=float(efficiency),
            average_speed_mps=float(self.distance_traveled_m / elapsed),
            turn_average_speed_mps=float(self.turn_distance_m / self.turn_time_s) if self.turn_time_s > 1e-6 else 0.0,
            straight_average_speed_mps=float(self.straight_distance_m / self.straight_time_s) if self.straight_time_s > 1e-6 else 0.0,
            max_speed_mps=float(self.max_speed_mps),
            turn_max_speed_mps=float(self.turn_max_speed_mps),
            straight_max_speed_mps=float(self.straight_max_speed_mps),
            turn_entry_speed_mps=float(_mean(self.turn_entry_speeds)),
            turn_exit_speed_mps=float(_mean(self.turn_exit_speeds)),
            recovery_time_after_stuck_s=float(self.stuck_recovery_time_s),
            route_turn_segment_count=segment_counts["route_turn_segment_count"],
            route_straight_segment_count=segment_counts["route_straight_segment_count"],
            completed_turn_segment_count=segment_counts["completed_turn_segment_count"],
            completed_straight_segment_count=segment_counts["completed_straight_segment_count"],
            failed_turn_segment_count=segment_counts["failed_turn_segment_count"],
            failed_straight_segment_count=segment_counts["failed_straight_segment_count"],
            backward_progress_events=int(self.backward_progress_events),
            backward_progress_m=float(self.backward_progress_m),
            max_backward_step_m=float(self.max_backward_step_m),
            backward_time_s=float(self.backward_time_s),
            reverse_command_steps=int(self.reverse_command_steps),
            reverse_command_time_s=float(self.reverse_command_time_s),
            command_jerk=float(command_jerk(self.action_history)),
            corridor_width_m=float(self.active_stage.cell_size_m if self.active_stage else 0.0),
            friction=float(getattr(self, "sampled_friction", 0.0)),
            scan_noise_std_m=float(self.active_stage.scan_noise_std_m if self.active_stage else 0.0),
        )

    def _update_wall_contacts(self) -> None:
        wall_touch = False
        for index in range(int(getattr(self.data, "ncon", 0))):
            contact = self.data.contact[index]
            names = [
                self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or "",
                self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or "",
            ]
            if _is_robot_wall_contact(names[0], names[1]):
                wall_touch = True
                break
        self.wall_contact_this_step = self.wall_contact_this_step or wall_touch
        if wall_touch and not self.wall_contact_active:
            self.collision_count += 1
            if self._is_current_pose_turn_phase():
                self.turn_collision_count += 1
            else:
                self.straight_collision_count += 1
            self.wall_contact_active = True
        elif not wall_touch:
            self.wall_contact_active = False

    def _update_turn_metrics(self, projection: PathProjection) -> None:
        state = projection.segment.state
        speed = self._ground_speed()
        if state == ARC_TURN and self.previous_segment_state != ARC_TURN:
            self.turn_entry_speeds.append(speed)
        elif self.previous_segment_state == ARC_TURN and state != ARC_TURN:
            self.turn_exit_speeds.append(speed)
        self.previous_segment_state = state

    def _record_motion_sample(self, projection: PathProjection, distance_delta: float, dt: float) -> None:
        if dt <= 1e-9:
            return
        speed = self._ground_speed()
        if self._is_turn_projection(projection):
            self.turn_distance_m += distance_delta
            self.turn_time_s += dt
            self.turn_max_speed_mps = max(self.turn_max_speed_mps, speed)
        else:
            self.straight_distance_m += distance_delta
            self.straight_time_s += dt
            self.straight_max_speed_mps = max(self.straight_max_speed_mps, speed)

    def _record_backward_sample(self, progress_delta: float, command: VelocityCommand, dt: float) -> None:
        threshold = float(self.termination.get("backward_progress_threshold_m", 0.03))
        if progress_delta < -threshold:
            backward_m = abs(progress_delta)
            self.backward_progress_events += 1
            self.backward_progress_m += backward_m
            self.max_backward_step_m = max(self.max_backward_step_m, backward_m)
            self.backward_time_s += max(0.0, dt)
        if command.vx < -0.03:
            self.reverse_command_steps += 1
            self.reverse_command_time_s += max(0.0, dt)

    def _update_stuck_watch(self, progress_delta: float, command: VelocityCommand, now: float) -> None:
        min_progress = float(self.termination.get("stuck_min_progress_m", 0.02))
        commanded = abs(command.vx) > 0.03 or abs(command.vy) > 0.03 or abs(command.yaw_rate) > 0.05
        if progress_delta > min_progress:
            self.best_progress_m = max(self.best_progress_m, self.projection.progress_m)
            self.last_progress_time_s = now
            if self.stuck_active_since is not None:
                self.stuck_recovery_time_s += now - self.stuck_active_since
                self.stuck_active_since = None
            return
        if not commanded:
            return
        timeout = float(self.termination.get("stuck_timeout_s", 6.0))
        if now - self.last_progress_time_s > timeout:
            self.stuck_timeout_exceeded = True
            if self.stuck_active_since is None:
                self.stuck_active_since = self.last_progress_time_s

    def _success(self, projection: PathProjection) -> bool:
        tolerance = float(self.termination.get("goal_tolerance_m", self.active_config.get("robot", {}).get("goal_tolerance_m", 0.5)))
        return projection.distance_to_goal_m <= tolerance or projection.progress_m >= self.features.total_length_m - tolerance

    def _final_status(self, success: bool, fallen: bool, collision: bool, stuck: bool, truncated: bool) -> str:
        if success:
            return "SUCCESS"
        if fallen:
            return "FALL"
        if collision:
            return "COLLISION"
        if stuck:
            return "STUCK"
        if truncated:
            return "TIMEOUT"
        return "RUNNING"

    def _action_to_command(self, action: np.ndarray) -> VelocityCommand:
        limits = self.command_limits
        vx = _scale(float(action[0]), float(limits.get("vx_min_mps", -0.35)), float(limits.get("vx_max_mps", 0.80)))
        vy = _scale(float(action[1]), float(limits.get("vy_min_mps", -0.20)), float(limits.get("vy_max_mps", 0.20)))
        yaw = _scale(
            float(action[2]),
            -float(limits.get("yaw_rate_max_radps", 1.20)),
            float(limits.get("yaw_rate_max_radps", 1.20)),
        )
        return VelocityCommand(vx=vx, vy=vy, yaw_rate=yaw)

    def _project_pose(self) -> PathProjection:
        return self.features.project(self._pose())

    def _pose(self) -> Pose2D:
        return pose_from_base_state(base_state(self.data))

    def _xy(self) -> tuple[float, float]:
        return float(self.data.qpos[0]), float(self.data.qpos[1])

    def _ground_speed(self) -> float:
        return math.hypot(float(self.data.qvel[0]), float(self.data.qvel[1]))

    def _body_velocity(self) -> tuple[float, float, float]:
        yaw = base_state(self.data)["yaw"]
        world_vx, world_vy = float(self.data.qvel[0]), float(self.data.qvel[1])
        body_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
        body_vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
        return body_vx, body_vy, float(self.data.qvel[5])

    def _local_corridor_width(self, rays: np.ndarray) -> float:
        left_index = 27
        right_index = 9
        return float(rays[left_index] + rays[right_index])

    def _segment_counts(self, *, success: bool) -> dict[str, int]:
        segments = list(self.features.path.segments)
        completed_indices = set(range(len(segments))) if success else set(range(max(0, self.max_segment_index_seen)))
        route_turn = sum(1 for segment in segments if segment.state == ARC_TURN)
        route_straight = len(segments) - route_turn
        completed_turn = sum(1 for segment in segments if segment.index in completed_indices and segment.state == ARC_TURN)
        completed_straight = sum(1 for segment in segments if segment.index in completed_indices and segment.state != ARC_TURN)
        failed_turn = 0
        failed_straight = 0
        if not success and self.projection is not None:
            if self.projection.segment.state == ARC_TURN:
                failed_turn = 1
            else:
                failed_straight = 1
        return {
            "route_turn_segment_count": route_turn,
            "route_straight_segment_count": route_straight,
            "completed_turn_segment_count": completed_turn,
            "completed_straight_segment_count": completed_straight,
            "failed_turn_segment_count": failed_turn,
            "failed_straight_segment_count": failed_straight,
        }

    def _is_turn_phase(self) -> bool:
        return self.projection is not None and self._is_turn_projection(self.projection)

    def _is_current_pose_turn_phase(self) -> bool:
        if self.features is None or self.data is None:
            return self._is_turn_phase()
        return self._is_turn_projection(self._project_pose())

    @staticmethod
    def _is_turn_projection(projection: PathProjection) -> bool:
        return projection.segment.state == ARC_TURN

    def _apply_random_friction(self, stage: StageSpec) -> None:
        low, high = stage.friction_range
        friction = float(self.rng.uniform(min(low, high), max(low, high)))
        self.sampled_friction = friction
        for geom_id in range(int(self.model.ngeom)):
            name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
            if name == "maze_floor" or name.startswith("maze_wall_"):
                self.model.geom_friction[geom_id][0] = friction

    def _record_trajectory_row(self, status: str, command: VelocityCommand, reward: float) -> None:
        state = base_state(self.data)
        self.trajectory.append(
            {
                "time_s": float(self.elapsed_s),
                "x": float(state["base_x"]),
                "y": float(state["base_y"]),
                "yaw": float(state["yaw"]),
                "vx": float(command.vx),
                "vy": float(command.vy),
                "yaw_rate": float(command.yaw_rate),
                "progress_m": float(self.projection.progress_m if self.projection else 0.0),
                "reward": float(reward),
                "status": status,
                "segment_index": int(self.projection.segment_index if self.projection else 0),
            }
        )

    def _ensure_ready(self) -> None:
        if self.mujoco is None or self.model is None or self.data is None or self.features is None:
            raise RuntimeError("Environment must be reset before stepping.")


def _scale(value: float, low: float, high: float) -> float:
    return low + (float(value) + 1.0) * 0.5 * (high - low)


def load_locomotion_calibration(path: Path) -> dict[str, Any]:
    values = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(values, dict):
        raise ValueError(f"Locomotion calibration must be a JSON object: {path}")
    return values


def apply_locomotion_calibration_to_command_limits(
    command_limits: dict[str, Any],
    calibration: dict[str, Any],
) -> dict[str, Any]:
    """Return RL action limits capped by measured locomotion calibration scalars."""
    limits = dict(command_limits)
    recommended = calibration.get("recommended_safe_limits", {})
    calibrated_limits = calibration.get("command_limits", {})
    if not isinstance(recommended, dict):
        recommended = {}
    if not isinstance(calibrated_limits, dict):
        calibrated_limits = {}

    max_forward = _first_finite(
        recommended.get("max_safe_vx"),
        calibration.get("selected_max_forward_mps"),
        calibrated_limits.get("max_forward_mps"),
    )
    if max_forward is not None:
        current = float(limits.get("vx_max_mps", max_forward))
        limits["vx_max_mps"] = min(current, max(0.0, max_forward))

    max_reverse = _first_finite(calibrated_limits.get("max_reverse_mps"))
    if max_reverse is not None and max_reverse < 0.0:
        current = float(limits.get("vx_min_mps", max_reverse))
        limits["vx_min_mps"] = max(current, max_reverse)

    max_yaw = _first_finite(recommended.get("max_safe_wz"), calibrated_limits.get("max_yaw_rate_radps"))
    if max_yaw is not None:
        current = abs(float(limits.get("yaw_rate_max_radps", max_yaw)))
        limits["yaw_rate_max_radps"] = min(current, max(0.0, abs(max_yaw)))

    return limits


def _first_finite(*values: object) -> float | None:
    for value in values:
        try:
            if value is None:
                continue
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            return number
    return None


def _norm_clip(value: float, scale: float) -> float:
    return max(0.0, min(1.0, float(value) / max(float(scale), 1e-6)))


def _signed_norm_clip(value: float, scale: float) -> float:
    return max(-1.0, min(1.0, float(value) / max(float(scale), 1e-6)))


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _is_robot_wall_contact(geom1: str, geom2: str) -> bool:
    return (geom1.startswith("maze_wall_") and _is_robot_geom(geom2)) or (
        geom2.startswith("maze_wall_") and _is_robot_geom(geom1)
    )


def _is_robot_geom(name: str) -> bool:
    return bool(name) and not (
        name.startswith("maze_")
        or name.startswith("oracle_path_marker_")
        or name in {"floor", "maze_floor", "world", "groundplane"}
    )
