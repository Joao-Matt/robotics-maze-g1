"""Robot-facing wrapper around MuJoCo state and command adapters."""

from __future__ import annotations

from typing import Any, Dict
import math


class RobotInterface:
    """Minimal robot interface for simulator bring-up and command plumbing.

    Direct high-level velocity walking for the Unitree G1 is not implemented yet.
    Velocity commands are accepted and recorded so controller code can be wired
    safely before a real locomotion adapter exists.
    """

    def __init__(self, mujoco_module: Any, model: Any, data: Any) -> None:
        self._mujoco = mujoco_module
        self._model = model
        self._data = data
        self.last_command: Dict[str, Any] | None = None
        self.last_command_result: Dict[str, Any] | None = None

    def apply_command(self, command: Dict[str, Any] | None) -> None:
        """Accept a command and route velocity commands through the adapter."""
        command = command or {}
        if command.get("type") == "velocity":
            self.apply_velocity_command(command)
            return
        self.last_command = command
        self.last_command_result = {
            "adapter": "unitree_g1_placeholder",
            "applied": False,
            "reason": "Only high-level velocity command plumbing exists so far.",
        }

    def apply_velocity_command(self, command: Any) -> Dict[str, Any]:
        """Accept a high-level velocity command without actuating G1 yet."""
        normalized = _normalize_velocity_command(command)
        self.last_command = {"type": "velocity", **normalized}
        self.last_command_result = {
            "adapter": "unitree_g1_velocity_placeholder",
            "applied": False,
            "reason": (
                "Direct humanoid velocity control is not available yet; "
                "Milestone 5 validates this command through point-robot mode."
            ),
            "command": normalized,
        }
        return self.last_command_result

    def get_state(self) -> Dict[str, Any]:
        """Return a compact, JSON-serializable state summary."""
        qpos = self._data.qpos
        qvel = self._data.qvel

        base_pose = None
        if self._model.nq >= 7:
            base_pose = {
                "position_xyz": [float(value) for value in qpos[:3]],
                "orientation_wxyz": [float(value) for value in qpos[3:7]],
            }

        base_velocity = None
        if self._model.nv >= 6:
            base_velocity = {
                "linear_xyz": [float(value) for value in qvel[:3]],
                "angular_xyz": [float(value) for value in qvel[3:6]],
            }

        base_height = base_pose["position_xyz"][2] if base_pose else None

        return {
            "time_s": float(self._data.time),
            "nq": int(self._model.nq),
            "nv": int(self._model.nv),
            "nu": int(self._model.nu),
            "joint_names": self._joint_names(),
            "base_pose": base_pose,
            "base_velocity": base_velocity,
            "qpos_sample": [float(value) for value in qpos[: min(10, self._model.nq)]],
            "qvel_sample": [float(value) for value in qvel[: min(10, self._model.nv)]],
            "fall_check": {
                "base_height_m": base_height,
                "fall_detected": bool(base_height is not None and base_height < 0.35),
            },
            "last_command": self.last_command,
            "last_command_result": self.last_command_result,
        }

    def _joint_names(self) -> list[str]:
        names: list[str] = []
        for joint_id in range(self._model.njnt):
            name = self._mujoco.mj_id2name(
                self._model,
                self._mujoco.mjtObj.mjOBJ_JOINT,
                joint_id,
            )
            names.append(name or f"joint_{joint_id}")
        return names


class GroundTruthPoseProvider:
    """MuJoCo ground-truth pose provider for MODE=oracle debugging only."""

    def __init__(self, model: Any, data: Any, mode: str) -> None:
        if mode != "oracle":
            raise ValueError("GroundTruthPoseProvider is allowed only in MODE=oracle/debug.")
        self._model = model
        self._data = data
        self.mode = mode

    def get_pose2d(self) -> Dict[str, float]:
        if self._model.nq < 7:
            raise ValueError("MuJoCo model does not expose a floating-base qpos pose.")
        qpos = self._data.qpos
        return {
            "x": float(qpos[0]),
            "y": float(qpos[1]),
            "yaw": _yaw_from_quat_wxyz(float(qpos[3]), float(qpos[4]), float(qpos[5]), float(qpos[6])),
            "source": "ground_truth_oracle",
        }


def _normalize_velocity_command(command: Any) -> Dict[str, float]:
    if hasattr(command, "to_dict"):
        command = command.to_dict()
    command = command or {}
    return {
        "vx": float(command.get("vx", 0.0)),
        "vy": float(command.get("vy", 0.0)),
        "wz": float(command.get("wz", 0.0)),
    }


def _yaw_from_quat_wxyz(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
