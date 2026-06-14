"""Robot-facing wrapper around MuJoCo state for simulator bring-up."""

from __future__ import annotations

from typing import Any, Dict


class RobotInterface:
    """Minimal robot interface for Milestone 1.

    This class intentionally exposes MuJoCo state directly because Milestone 1 is
    simulator bring-up. Later autonomy code should depend on a narrower robot
    observable interface instead of raw simulator state.
    """

    def __init__(self, mujoco_module: Any, model: Any, data: Any) -> None:
        self._mujoco = mujoco_module
        self._model = model
        self._data = data
        self.last_command: Dict[str, Any] | None = None

    def apply_command(self, command: Dict[str, Any] | None) -> None:
        """Accept a command placeholder without applying actuation yet."""
        self.last_command = command or {}

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
