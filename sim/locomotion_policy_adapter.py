"""Locomotion policy adapter boundary for the production Unitree G1 runtime."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any
import json
import os

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class LocomotionPolicyError(RuntimeError):
    """Raised when a locomotion policy cannot be used safely."""


@dataclass
class VelocityCommand:
    """Desired base velocity command from teleop."""

    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0


@dataclass
class PolicyCompatibilityReport:
    """Structured compatibility result for a policy/model pair."""

    policy: str
    model_xml: str
    adapter: str
    loaded: bool = False
    real_locomotion: bool = False
    expected_observation_dim: int | None = None
    actual_observation_dim: int | None = None
    expected_action_dim: int | None = None
    actual_action_dim: int | None = None
    model_nu: int | None = None
    joint_names_checked: bool = False
    actuator_names_checked: bool = False
    compatible: bool | None = False
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def write_json(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")


class LocomotionPolicyAdapter:
    """Base interface for teleop velocity command to MuJoCo control adapters."""

    adapter_name = "base"
    real_locomotion = False

    def __init__(self, policy: str) -> None:
        self.policy = policy
        self.report: PolicyCompatibilityReport | None = None

    def reset(self, model: Any, data: Any) -> None:
        """Reset policy state for a new MuJoCo rollout."""

    def step(self, model: Any, data: Any, command: VelocityCommand, dt: float) -> None:
        """Apply or return controls for one policy step."""

    def compatibility_report(self, model: Any, model_xml: Path) -> PolicyCompatibilityReport:
        return PolicyCompatibilityReport(
            policy=self.policy,
            model_xml=str(model_xml),
            adapter=self.adapter_name,
            loaded=True,
            real_locomotion=self.real_locomotion,
            model_nu=int(getattr(model, "nu", 0)),
            compatible=True,
        )

    def prepare_model_xml(self, output_dir: Path) -> Path | None:
        """Optionally provide a policy-specific model XML before MuJoCo loads."""
        return None


class PlaceholderPolicyAdapter(LocomotionPolicyAdapter):
    """Safe non-walking placeholder that holds the configured standing control."""

    adapter_name = "placeholder"
    real_locomotion = False

    def __init__(self, policy: str = "placeholder") -> None:
        super().__init__(policy)
        self._stand_ctrl: Any | None = None

    def reset(self, model: Any, data: Any) -> None:
        key_id = _find_keyframe_id(model, "stand")
        if key_id is not None and hasattr(model, "key_ctrl"):
            self._stand_ctrl = model.key_ctrl[key_id].copy()
            if len(self._stand_ctrl) == getattr(model, "nu", len(self._stand_ctrl)):
                data.ctrl[:] = self._stand_ctrl

    def step(self, model: Any, data: Any, command: VelocityCommand, dt: float) -> None:
        if self._stand_ctrl is not None and len(self._stand_ctrl) == getattr(model, "nu", len(self._stand_ctrl)):
            data.ctrl[:] = self._stand_ctrl

    def compatibility_report(self, model: Any, model_xml: Path) -> PolicyCompatibilityReport:
        report = PolicyCompatibilityReport(
            policy=self.policy,
            model_xml=str(model_xml),
            adapter=self.adapter_name,
            loaded=True,
            real_locomotion=False,
            actual_action_dim=int(getattr(model, "nu", 0)),
            model_nu=int(getattr(model, "nu", 0)),
            joint_names_checked=False,
            actuator_names_checked=False,
            compatible=True,
            warnings=[
                "No real walking policy loaded. This mode validates viewer, teleop input, recording, and logging only."
            ],
        )
        self.report = report
        return report


class UnitreeRLGymG1PolicyAdapter(LocomotionPolicyAdapter):
    """Adapter for Unitree RL Gym's native G1 TorchScript policy and native XML."""

    real_locomotion = True
    requires_substep_control = True

    leg_joint_names = [
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
    ]

    def __init__(
        self,
        policy: str = "unitree_rl_gym_native",
        repo_root: str | Path | None = None,
    ) -> None:
        super().__init__(policy)
        self.native_model = True
        self.adapter_name = "unitree_rl_gym_native"
        default_repo = PROJECT_ROOT / "third_party" / "unitree_rl_gym"
        self.repo_root = Path(repo_root or os.environ.get("UNITREE_RL_GYM_REPO", default_repo)).expanduser()
        self.policy_path = self.repo_root / "deploy" / "pre_train" / "g1" / "motion.pt"
        self.config_path = self.repo_root / "deploy" / "deploy_mujoco" / "configs" / "g1.yaml"
        self.native_xml_path = self.repo_root / "resources" / "robots" / "g1_description" / "scene.xml"
        self.torch: Any | None = None
        self.policy_model: Any | None = None
        self.kps = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40], dtype=np.float32)
        self.kds = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2], dtype=np.float32)
        self.default_angles = np.array(
            [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0],
            dtype=np.float32,
        )
        self.ang_vel_scale = 0.25
        self.dof_pos_scale = 1.0
        self.dof_vel_scale = 0.05
        self.action_scale = 0.25
        self.cmd_scale = np.array([2.0, 2.0, 0.25], dtype=np.float32)
        self.action = np.zeros(12, dtype=np.float32)
        self.target_dof_pos = self.default_angles.copy()
        self.control_counter = 0
        self.stand_ctrl: Any | None = None
        self.actuator_ids: list[int] = []
        self.joint_qpos_indices: list[int] = []
        self.joint_qvel_indices: list[int] = []

    def prepare_model_xml(self, output_dir: Path) -> Path | None:
        return self.native_xml_path if self.native_xml_path.exists() else None

    def reset(self, model: Any, data: Any) -> None:
        if not self.report or not self.report.compatible:
            report = self.compatibility_report(model, Path(""))
            if not report.compatible:
                raise LocomotionPolicyError("; ".join(report.errors))

        key_id = _find_keyframe_id(model, "stand")
        if key_id is not None:
            import mujoco

            mujoco.mj_resetDataKeyframe(model, data, key_id)
            self.stand_ctrl = model.key_ctrl[key_id].copy() if hasattr(model, "key_ctrl") else None
        else:
            data.qpos[:] = 0.0
            data.qvel[:] = 0.0
            data.qpos[2] = 0.79
            data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

        for index, qpos_index in enumerate(self.joint_qpos_indices):
            data.qpos[qpos_index] = self.default_angles[index]

        if self.stand_ctrl is not None and len(self.stand_ctrl) == getattr(model, "nu", len(self.stand_ctrl)):
            data.ctrl[:] = self.stand_ctrl
        for index, actuator_id in enumerate(self.actuator_ids):
            data.ctrl[actuator_id] = 0.0

        self.action[:] = 0.0
        self.target_dof_pos = self.default_angles.copy()
        self.control_counter = 0

    def step(self, model: Any, data: Any, command: VelocityCommand, dt: float) -> None:
        if self.policy_model is None or self.torch is None:
            raise LocomotionPolicyError("Unitree RL Gym G1 policy has not been loaded.")

        if self.stand_ctrl is not None and len(self.stand_ctrl) == getattr(model, "nu", len(self.stand_ctrl)):
            data.ctrl[:] = self.stand_ctrl

        if self.control_counter % 10 == 0:
            obs = self._build_observation(data, command)
            obs_tensor = self.torch.from_numpy(obs).unsqueeze(0)
            with self.torch.no_grad():
                self.action = self.policy_model(obs_tensor).detach().cpu().numpy().squeeze().astype(np.float32)
            if self.action.shape != (12,):
                raise LocomotionPolicyError(f"Unitree RL Gym policy returned {self.action.shape}, expected (12,).")
            self.target_dof_pos = self.action * self.action_scale + self.default_angles

        q = np.array([data.qpos[index] for index in self.joint_qpos_indices], dtype=np.float32)
        dq = np.array([data.qvel[index] for index in self.joint_qvel_indices], dtype=np.float32)
        tau = (self.target_dof_pos - q) * self.kps + (0.0 - dq) * self.kds
        for index, actuator_id in enumerate(self.actuator_ids):
            data.ctrl[actuator_id] = tau[index]
        self.control_counter += 1

    def compatibility_report(self, model: Any, model_xml: Path) -> PolicyCompatibilityReport:
        report = PolicyCompatibilityReport(
            policy=str(self.policy_path),
            model_xml=str(model_xml),
            adapter=self.adapter_name,
            loaded=False,
            real_locomotion=True,
            expected_observation_dim=47,
            actual_observation_dim=47,
            expected_action_dim=12,
            actual_action_dim=12,
            model_nu=int(getattr(model, "nu", 0)),
            compatible=False,
            warnings=[],
        )

        required_paths = [self.repo_root, self.policy_path, self.config_path, self.native_xml_path]
        for path in required_paths:
            if not path.exists():
                report.errors.append(f"Required Unitree RL Gym asset does not exist: {path}")
        if report.errors:
            report.errors.append("Run `make fetch-unitree-rl-gym-policy` before POLICY=unitree_rl_gym_native.")
            self.report = report
            return report

        actual_actuators = _mujoco_names(model, "actuator")
        if actual_actuators[:12] == self.leg_joint_names:
            report.actuator_names_checked = True
        else:
            report.errors.append("First 12 MuJoCo actuators do not match Unitree RL Gym G1 leg joint order.")
        if int(getattr(model, "nu", 0)) != 12:
            report.errors.append(f"Native Unitree RL Gym model must have 12 actuators, got {getattr(model, 'nu', None)}.")

        actual_joints = _mujoco_names(model, "joint")
        missing_joints = [name for name in self.leg_joint_names if name not in actual_joints]
        report.joint_names_checked = len(missing_joints) == 0
        if missing_joints:
            report.errors.append(f"MuJoCo model is missing Unitree RL Gym leg joints: {missing_joints}")

        try:
            import torch

            self.torch = torch
            self.policy_model = torch.jit.load(str(self.policy_path), map_location="cpu")
            self.policy_model.eval()
            report.loaded = True
        except ModuleNotFoundError:
            report.errors.append(
                "torch is not installed. Install a CPU Torch wheel before using POLICY=unitree_rl_gym_native."
            )
        except Exception as exc:
            report.errors.append(f"Failed to load Unitree RL Gym TorchScript policy: {exc}")

        if not report.errors:
            self._build_static_arrays(model)
            report.compatible = True
        self.report = report
        return report

    def _build_static_arrays(self, model: Any) -> None:
        self.actuator_ids = [_mujoco_id(model, "actuator", name) for name in self.leg_joint_names]
        self.joint_qpos_indices = [_joint_qpos_index(model, name) for name in self.leg_joint_names]
        self.joint_qvel_indices = [_joint_qvel_index(model, name) for name in self.leg_joint_names]

    def _build_observation(self, data: Any, command: VelocityCommand) -> np.ndarray:
        obs = np.zeros(47, dtype=np.float32)
        qj = np.array([data.qpos[index] for index in self.joint_qpos_indices], dtype=np.float32)
        dqj = np.array([data.qvel[index] for index in self.joint_qvel_indices], dtype=np.float32)
        quat = data.qpos[3:7]
        omega = data.qvel[3:6].astype(np.float32)

        qj = (qj - self.default_angles) * self.dof_pos_scale
        dqj = dqj * self.dof_vel_scale
        gravity_orientation = _unitree_gravity_orientation(quat)
        omega = omega * self.ang_vel_scale
        cmd = np.array([command.vx, command.vy, command.yaw_rate], dtype=np.float32)

        period = 0.8
        phase = (self.control_counter * 0.002) % period / period
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)

        obs[:3] = omega
        obs[3:6] = gravity_orientation
        obs[6:9] = cmd * self.cmd_scale
        obs[9:21] = qj
        obs[21:33] = dqj
        obs[33:45] = self.action
        obs[45:47] = np.array([sin_phase, cos_phase], dtype=np.float32)
        return obs


def create_policy_adapter(
    policy: str | None,
    unitree_rl_gym_repo: str | Path | None = None,
) -> LocomotionPolicyAdapter:
    """Create an adapter from a policy name/path."""
    policy_value = (policy or "placeholder").strip()
    if not policy_value or policy_value == "placeholder":
        return PlaceholderPolicyAdapter("placeholder")

    if policy_value == "unitree_rl_gym_native":
        return UnitreeRLGymG1PolicyAdapter(policy_value, repo_root=unitree_rl_gym_repo)

    raise LocomotionPolicyError(
        f"Unknown policy adapter for {policy_value!r}. Production supports unitree_rl_gym_native."
    )


def _find_keyframe_id(model: Any, name: str) -> int | None:
    names = getattr(model, "names", None)
    if names is None or not hasattr(model, "name_keyadr"):
        return None
    for index in range(getattr(model, "nkey", 0)):
        start = int(model.name_keyadr[index])
        end = names.find(b"\x00", start)
        if names[start:end].decode("utf-8") == name:
            return index
    return None


def _mujoco_names(model: Any, kind: str) -> list[str]:
    try:
        import mujoco
    except ModuleNotFoundError:
        return []

    if kind == "actuator":
        obj_type = mujoco.mjtObj.mjOBJ_ACTUATOR
        count = int(getattr(model, "nu", 0))
    elif kind == "joint":
        obj_type = mujoco.mjtObj.mjOBJ_JOINT
        count = int(getattr(model, "njnt", 0))
    else:
        raise ValueError(f"Unknown MuJoCo object kind: {kind}")

    names = []
    for index in range(count):
        name = mujoco.mj_id2name(model, obj_type, index)
        names.append(name or "")
    return names


def _mujoco_id(model: Any, kind: str, name: str) -> int:
    import mujoco

    obj_types = {
        "actuator": mujoco.mjtObj.mjOBJ_ACTUATOR,
        "joint": mujoco.mjtObj.mjOBJ_JOINT,
    }
    return mujoco.mj_name2id(model, obj_types[kind], name)


def _joint_qpos_index(model: Any, name: str) -> int:
    joint_id = _mujoco_id(model, "joint", name)
    if joint_id < 0:
        raise LocomotionPolicyError(f"Model is missing joint {name}")
    return int(model.jnt_qposadr[joint_id])


def _joint_qvel_index(model: Any, name: str) -> int:
    joint_id = _mujoco_id(model, "joint", name)
    if joint_id < 0:
        raise LocomotionPolicyError(f"Model is missing joint {name}")
    return int(model.jnt_dofadr[joint_id])


def _unitree_gravity_orientation(quaternion: Any) -> np.ndarray:
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3, dtype=np.float32)
    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)
    return gravity_orientation
