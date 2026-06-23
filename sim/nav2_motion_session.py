"""Safety-gated Nav2 command application for locomotion policies."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path

import numpy as np

from sim.locomotion_policy_adapter import VelocityCommand, create_policy_adapter
from sim.locomotion_sandbox import base_state, config_from_dict, determine_status


@dataclass
class NavigationLimits:
    command_timeout_s: float = 0.50
    wall_contact_abort_s: float = 0.50
    stuck_timeout_s: float = 12.0
    stuck_min_progress_m: float = 0.08
    commanded_stuck_timeout_s: float = 30.0
    commanded_stuck_min_translation_m: float = 0.08
    commanded_stuck_min_yaw_rad: float = 0.10
    max_forward_mps: float = 0.30
    min_forward_mps: float = 0.10
    max_reverse_mps: float = -0.30
    max_yaw_rate_radps: float = 1.00
    max_linear_accel_mps2: float = 2.00
    max_linear_decel_mps2: float = 2.00
    max_yaw_accel_radps2: float = 1.20
    max_yaw_decel_radps2: float = 1.50
    turn_slowdown_start_radps: float = 0.25
    turn_slowdown_full_radps: float = 0.65
    turn_slowdown_min_forward_mps: float = 0.05

    @classmethod
    def from_config(cls, config: dict) -> "NavigationLimits":
        values = config.get("nav2_navigation", {})
        return cls(**{key: value for key, value in values.items() if key in cls.__annotations__})


class Nav2MotionSession:
    """Apply Nav2 commands while retaining physical state only for evaluation."""

    def __init__(
        self,
        mujoco,
        model,
        data,
        config: dict,
        model_xml: Path,
        duration_s: float,
        start_x: float,
        start_y: float,
        start_yaw: float,
        ground_truth_used_for_navigation: bool = False,
        policy: str | None = None,
        unitree_rl_gym_repo: Path | None = None,
    ) -> None:
        self.mujoco, self.model, self.data = mujoco, model, data
        self.limits = NavigationLimits.from_config(config)
        self.sandbox = config_from_dict(config)
        nav_config = config.get("nav2_navigation", {})
        policy_name = policy or str(nav_config.get("locomotion_policy", "unitree_rl_gym_native"))
        self.adapter = create_policy_adapter(
            policy_name,
            unitree_rl_gym_repo=unitree_rl_gym_repo,
        )
        report = self.adapter.compatibility_report(model, model_xml)
        if report.errors:
            raise RuntimeError("; ".join(report.errors))
        self._reset_policy_at_pose(config, start_x, start_y, start_yaw)
        mujoco.mj_forward(model, data)
        self.control_dt = 1.0 / self.sandbox.control_rate_hz
        self.substeps = max(1, round(self.control_dt / float(model.opt.timestep)))
        self.end_time = float(data.time) + duration_s
        self.raw_command = VelocityCommand()
        self.last_command = VelocityCommand()
        self.last_command_time: float | None = None
        self.command_received = False
        self.armed = False
        self.status = "WAITING_FOR_CMD"
        self.stop_reason = "waiting_for_nav2_command"
        self.nav_x, self.nav_y, self.nav_yaw = start_x, start_y, start_yaw
        self.start_xy = (start_x, start_y)
        self.last_actual_xy = self.start_xy
        self.distance_traveled_m = 0.0
        self.progress_anchor_xy = self.start_xy
        self.progress_anchor_time = float(data.time)
        self.progress_watch_active = False
        self.commanded_motion_anchor_xy = self.start_xy
        self.commanded_motion_anchor_yaw = start_yaw
        self.commanded_motion_anchor_time = float(data.time)
        self.commanded_motion_watch_active = False
        self.wall_contact_since: float | None = None
        self.contact_counts = {"foot_floor": 0, "wall": 0, "self": 0, "other": 0}
        self.max_contact_force_n = 0.0
        self.recovery_events: list[dict[str, object]] = []
        self.achieved = VelocityCommand()
        self.ground_truth_used_for_navigation = bool(ground_truth_used_for_navigation)
        self.requires_substep_control = bool(getattr(self.adapter, "requires_substep_control", False))

    def _reset_policy_at_pose(self, config: dict, start_x: float, start_y: float, start_yaw: float) -> None:
        reset_at_pose = getattr(self.adapter, "reset_at_pose", None)
        if callable(reset_at_pose):
            reset_at_pose(self.model, self.data, start_x, start_y, start_yaw)
            return

        self.adapter.reset(self.model, self.data)
        base_height = float(config.get("robot", {}).get("initial_base_height_m", self.data.qpos[2]))
        self.data.qpos[0] = start_x
        self.data.qpos[1] = start_y
        self.data.qpos[2] = base_height
        self.data.qpos[3:7] = [math.cos(start_yaw / 2.0), 0.0, 0.0, math.sin(start_yaw / 2.0)]

    def set_command(self, command: VelocityCommand) -> None:
        if not all(math.isfinite(value) for value in (command.vx, command.vy, command.yaw_rate)):
            self._stop("NAV2_ABORTED", "non_finite_nav2_command")
            return
        self.raw_command = command
        self.last_command_time = float(self.data.time)
        self.command_received = True
        if self.status == "WAITING_FOR_CMD":
            self.status, self.stop_reason = "RUNNING", "navigation_active"

    def set_armed(self, armed: bool) -> None:
        self.armed = bool(armed)
        if not self.armed:
            self.raw_command = VelocityCommand()
            self.last_command_time = float(self.data.time)

    def step(self) -> None:
        if self.status not in {"WAITING_FOR_CMD", "RUNNING"}:
            self._step_physics(VelocityCommand())
            return
        now = float(self.data.time)
        if now >= self.end_time:
            self._stop("TIMEOUT", "duration_timeout")
        stale = self.armed and self.command_received and self.last_command_time is not None and now - self.last_command_time > self.limits.command_timeout_s
        if stale:
            self.armed=False
            self.raw_command=VelocityCommand()
            self.last_command=VelocityCommand()
            self.recovery_events.append({"time_s":now,"event":"command_timeout_disarm"})
        applied = self._limited_command(self.raw_command) if self.status == "RUNNING" else VelocityCommand()
        if self.status == "RUNNING" and not stale:
            applied = self._slewed_command(applied)
        state = base_state(self.data)
        if determine_status(applied, state, self.sandbox) == "fallen":
            self._stop("FALL_DETECTED", "fall_detected")
        self._update_contacts(now)
        if self.wall_contact_since is not None:
            applied = VelocityCommand()
            self.armed = False
            self._stop("COLLISION_ABORT", "wall_contact_immediate_stop")
        requires_substep = bool(getattr(self, "requires_substep_control", False))
        if self.status in {"WAITING_FOR_CMD", "RUNNING"} and not requires_substep:
            self.adapter.step(self.model, self.data, applied, self.control_dt)
        elif self.status not in {"WAITING_FOR_CMD", "RUNNING"}:
            applied = VelocityCommand()
            self.data.ctrl[:] = 0.0
        self.last_command = applied
        self._step_physics(applied)
        self._update_motion(now)

    def _step_physics(self, applied: VelocityCommand) -> None:
        dt = self.control_dt
        midpoint = self.nav_yaw + 0.5 * applied.yaw_rate * dt
        self.nav_x += applied.vx * math.cos(midpoint) * dt
        self.nav_y += applied.vx * math.sin(midpoint) * dt
        self.nav_yaw = math.atan2(math.sin(self.nav_yaw + applied.yaw_rate * dt), math.cos(self.nav_yaw + applied.yaw_rate * dt))
        substep_dt = float(self.model.opt.timestep)
        for _ in range(self.substeps):
            if self.status in {"WAITING_FOR_CMD", "RUNNING"} and bool(getattr(self, "requires_substep_control", False)):
                self.adapter.step(self.model, self.data, applied, substep_dt)
            self.mujoco.mj_step(self.model, self.data)

    def _limited_command(self, command: VelocityCommand) -> VelocityCommand:
        yaw_rate = max(-self.limits.max_yaw_rate_radps, min(self.limits.max_yaw_rate_radps, command.yaw_rate))
        forward_limit = max(0.0, self.limits.max_forward_mps)
        reverse_limit = min(0.0, self.limits.max_reverse_mps)
        abs_yaw = abs(yaw_rate)
        start = max(0.0, self.limits.turn_slowdown_start_radps)
        full = max(start + 1e-6, self.limits.turn_slowdown_full_radps)
        minimum = max(
            0.0,
            min(
                max(self.limits.min_forward_mps, self.limits.turn_slowdown_min_forward_mps),
                forward_limit,
            ),
        )
        if abs_yaw <= start:
            coupled_forward_limit = forward_limit
        elif abs_yaw >= full:
            coupled_forward_limit = minimum
        else:
            fraction = (abs_yaw - start) / (full - start)
            coupled_forward_limit = forward_limit + fraction * (minimum - forward_limit)
        vx = command.vx
        if vx >= 0.0:
            if abs_yaw > start:
                vx = max(vx, minimum)
            vx = min(vx, coupled_forward_limit)
        else:
            vx = max(vx, reverse_limit)
        return VelocityCommand(vx=vx, vy=0.0, yaw_rate=yaw_rate)

    def _slewed_command(self, target: VelocityCommand) -> VelocityCommand:
        dt = float(getattr(self, "control_dt", 0.02))
        current = getattr(self, "last_command", VelocityCommand())
        return VelocityCommand(
            vx=self._slew_axis(
                current.vx,
                target.vx,
                self.limits.max_linear_accel_mps2,
                self.limits.max_linear_decel_mps2,
                dt,
            ),
            vy=0.0,
            yaw_rate=self._slew_axis(
                current.yaw_rate,
                target.yaw_rate,
                self.limits.max_yaw_accel_radps2,
                self.limits.max_yaw_decel_radps2,
                dt,
            ),
        )

    @staticmethod
    def _slew_axis(current: float, target: float, accel_limit: float, decel_limit: float, dt: float) -> float:
        if not all(math.isfinite(value) for value in (current, target, accel_limit, decel_limit, dt)):
            return target
        if dt <= 0.0:
            return target
        same_direction = current == 0.0 or target == 0.0 or math.copysign(1.0, current) == math.copysign(1.0, target)
        speeding_up = same_direction and abs(target) > abs(current)
        limit = max(0.0, accel_limit if speeding_up else decel_limit)
        max_delta = limit * dt
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        return current + math.copysign(max_delta, delta)

    def _update_motion(self, now: float) -> None:
        state = base_state(self.data)
        xy = (float(state["base_x"]), float(state["base_y"]))
        delta = math.hypot(xy[0] - self.last_actual_xy[0], xy[1] - self.last_actual_xy[1])
        self.distance_traveled_m += delta
        self.last_actual_xy = xy
        yaw = float(state["yaw"])
        world_vx, world_vy = float(self.data.qvel[0]), float(self.data.qvel[1])
        self.achieved = VelocityCommand(
            vx=math.cos(yaw) * world_vx + math.sin(yaw) * world_vy,
            vy=-math.sin(yaw) * world_vx + math.cos(yaw) * world_vy,
            yaw_rate=float(self.data.qvel[5]),
        )
        forward_commanded = abs(self.last_command.vx) > 0.03
        if forward_commanded and not self.progress_watch_active:
            # Nav2 may start many seconds after simulation initialization.
            # Begin the watchdog when actual forward motion is first applied,
            # not at world startup, and restart it after command pauses.
            self.progress_anchor_xy, self.progress_anchor_time = xy, now
            self.progress_watch_active = True
        elif not forward_commanded:
            self.progress_watch_active = False
        if math.hypot(xy[0] - self.progress_anchor_xy[0], xy[1] - self.progress_anchor_xy[1]) >= self.limits.stuck_min_progress_m:
            self.progress_anchor_xy, self.progress_anchor_time = xy, now
        self._update_commanded_stuck_watch(now, xy, yaw)

    def _update_commanded_stuck_watch(self, now: float, xy: tuple[float, float], yaw: float) -> None:
        raw = getattr(self, "raw_command", VelocityCommand())
        nonzero_command = (
            abs(raw.vx) > 0.03
            or abs(raw.vy) > 0.03
            or abs(raw.yaw_rate) > 0.05
        )
        if self.status != "RUNNING" or not nonzero_command:
            self.commanded_motion_watch_active = False
            return

        if not getattr(self, "commanded_motion_watch_active", False):
            self.commanded_motion_anchor_xy = xy
            self.commanded_motion_anchor_yaw = yaw
            self.commanded_motion_anchor_time = now
            self.commanded_motion_watch_active = True
            return

        translation = math.hypot(
            xy[0] - self.commanded_motion_anchor_xy[0],
            xy[1] - self.commanded_motion_anchor_xy[1],
        )
        yaw_delta = abs(math.atan2(
            math.sin(yaw - self.commanded_motion_anchor_yaw),
            math.cos(yaw - self.commanded_motion_anchor_yaw),
        ))
        if (
            translation >= self.limits.commanded_stuck_min_translation_m
            or yaw_delta >= self.limits.commanded_stuck_min_yaw_rad
        ):
            self.commanded_motion_anchor_xy = xy
            self.commanded_motion_anchor_yaw = yaw
            self.commanded_motion_anchor_time = now
            return

        if now - self.commanded_motion_anchor_time >= self.limits.commanded_stuck_timeout_s:
            self.recovery_events.append({
                "time_s": now,
                "event": "commanded_stuck",
                "duration_s": now - self.commanded_motion_anchor_time,
                "translation_m": translation,
                "yaw_rad": yaw_delta,
            })
            self._stop("STUCK", "commanded_no_ground_truth_motion")

    def _update_contacts(self, now: float) -> None:
        wall = False
        force = np.zeros(6, dtype=np.float64)
        for index in range(int(self.data.ncon)):
            contact = self.data.contact[index]
            names = []
            for geom_id in (int(contact.geom1), int(contact.geom2)):
                name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
                names.append(name.lower())
            joined = " ".join(names)
            if "wall" in joined:
                kind, wall = "wall", True
            elif ("foot" in joined or "ankle" in joined) and ("floor" in joined or "ground" in joined):
                kind = "foot_floor"
            elif all(name and "floor" not in name and "wall" not in name for name in names):
                kind = "self"
            else:
                kind = "other"
            self.contact_counts[kind] += 1
            try:
                self.mujoco.mj_contactForce(self.model, self.data, index, force)
                self.max_contact_force_n = max(self.max_contact_force_n, math.sqrt(sum(float(v) ** 2 for v in force[:3])))
            except Exception:
                pass
        if wall and self.wall_contact_since is None:
            self.wall_contact_since = now
            self.recovery_events.append({"time_s": now, "event": "wall_contact"})
        elif not wall:
            self.wall_contact_since = None

    def _stop(self, status: str, reason: str) -> None:
        if self.status in {"WAITING_FOR_CMD", "RUNNING"}:
            self.status, self.stop_reason = status, reason
            self.raw_command = VelocityCommand()

    def navigation_pose(self) -> tuple[float, float, float]:
        return self.nav_x, self.nav_y, self.nav_yaw

    def summary(self) -> dict[str, object]:
        return {
            "status": self.status,
            "stop_reason": self.stop_reason,
            "cmd_vel_application": f"{self.adapter.adapter_name}_policy",
            "raw_command": asdict(self.raw_command),
            "applied_command": asdict(self.last_command),
            "command_limiter": {
                "max_forward_mps": self.limits.max_forward_mps,
                "min_forward_mps": self.limits.min_forward_mps,
                "max_reverse_mps": self.limits.max_reverse_mps,
                "max_yaw_rate_radps": self.limits.max_yaw_rate_radps,
                "max_linear_accel_mps2": self.limits.max_linear_accel_mps2,
                "max_linear_decel_mps2": self.limits.max_linear_decel_mps2,
                "max_yaw_accel_radps2": self.limits.max_yaw_accel_radps2,
                "max_yaw_decel_radps2": self.limits.max_yaw_decel_radps2,
                "turn_slowdown_start_radps": self.limits.turn_slowdown_start_radps,
                "turn_slowdown_full_radps": self.limits.turn_slowdown_full_radps,
                "turn_slowdown_min_forward_mps": self.limits.turn_slowdown_min_forward_mps,
            },
            "commanded_stuck_watchdog": {
                "timeout_s": self.limits.commanded_stuck_timeout_s,
                "min_translation_m": self.limits.commanded_stuck_min_translation_m,
                "min_yaw_rad": self.limits.commanded_stuck_min_yaw_rad,
                "active": self.commanded_motion_watch_active,
            },
            "achieved_velocity_ground_truth": asdict(self.achieved),
            "distance_traveled_m": self.distance_traveled_m,
            "contact_counts": dict(self.contact_counts),
            "max_contact_force_n": self.max_contact_force_n,
            "recovery_events": list(self.recovery_events),
            "ground_truth_used_for_navigation": self.ground_truth_used_for_navigation,
            "navigation_armed": self.armed,
        }
