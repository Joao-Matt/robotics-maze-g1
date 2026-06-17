"""Locomotion policy adapter boundary for the visual G1 sandbox."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any
import importlib
import json
import math
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


class OnnxPolicyAdapter(LocomotionPolicyAdapter):
    """Strict ONNX policy adapter with metadata-driven validation."""

    adapter_name = "onnx"
    real_locomotion = True

    def __init__(self, policy_path: str, metadata_path: Path | None = None) -> None:
        super().__init__(policy_path)
        self.policy_path = Path(policy_path).expanduser()
        self.metadata_path = metadata_path or self.policy_path.with_suffix(".json")
        self.session: Any | None = None
        self.input_name: str | None = None
        self.output_name: str | None = None
        self.metadata: dict[str, Any] = {}
        self._compatible = False

    def reset(self, model: Any, data: Any) -> None:
        report = self.compatibility_report(model, Path(""))
        if not report.compatible:
            raise LocomotionPolicyError("; ".join(report.errors) or "ONNX policy is not compatible.")

    def step(self, model: Any, data: Any, command: VelocityCommand, dt: float) -> None:
        if not self._compatible:
            raise LocomotionPolicyError("ONNX policy was not validated as compatible.")
        raise LocomotionPolicyError(
            "ONNX execution is metadata-gated but observation construction is not implemented for this policy yet."
        )

    def compatibility_report(self, model: Any, model_xml: Path) -> PolicyCompatibilityReport:
        report = PolicyCompatibilityReport(
            policy=str(self.policy_path),
            model_xml=str(model_xml),
            adapter=self.adapter_name,
            loaded=False,
            real_locomotion=True,
            model_nu=int(getattr(model, "nu", 0)),
            compatible=False,
        )

        if not self.policy_path.exists():
            report.errors.append(f"ONNX policy file does not exist: {self.policy_path}")
            self.report = report
            return report

        try:
            import onnxruntime as ort
        except ModuleNotFoundError:
            report.errors.append(
                "onnxruntime is not installed. Install it as an optional dependency before loading ONNX policies."
            )
            self.report = report
            return report

        try:
            self.session = ort.InferenceSession(str(self.policy_path), providers=["CPUExecutionProvider"])
            inputs = self.session.get_inputs()
            outputs = self.session.get_outputs()
            if len(inputs) != 1 or len(outputs) != 1:
                report.errors.append(
                    f"Expected one ONNX input and one output, got {len(inputs)} input(s), {len(outputs)} output(s)."
                )
            else:
                self.input_name = inputs[0].name
                self.output_name = outputs[0].name
                report.loaded = True
                report.expected_observation_dim = _last_static_dim(inputs[0].shape)
                report.expected_action_dim = _last_static_dim(outputs[0].shape)
        except Exception as exc:
            report.errors.append(f"Failed to load ONNX policy: {exc}")
            self.report = report
            return report

        if not self.metadata_path.exists():
            report.errors.append(
                "Missing ONNX metadata JSON next to the policy. Required fields include observation_dim, "
                "action_dim, actuator_names, control_rate_hz, and action_scale."
            )
            self.report = report
            return report

        try:
            self.metadata = json.loads(self.metadata_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            report.errors.append(f"Failed to parse ONNX metadata JSON: {exc}")
            self.report = report
            return report

        metadata_obs_dim = self.metadata.get("observation_dim")
        metadata_action_dim = self.metadata.get("action_dim")
        if metadata_obs_dim is not None:
            report.actual_observation_dim = int(metadata_obs_dim)
        if metadata_action_dim is not None:
            report.actual_action_dim = int(metadata_action_dim)

        if report.expected_observation_dim is not None and metadata_obs_dim != report.expected_observation_dim:
            report.errors.append(
                f"Observation dim mismatch: ONNX expects {report.expected_observation_dim}, "
                f"metadata says {metadata_obs_dim}."
            )
        if report.expected_action_dim is not None and metadata_action_dim != report.expected_action_dim:
            report.errors.append(
                f"Action dim mismatch: ONNX outputs {report.expected_action_dim}, metadata says {metadata_action_dim}."
            )
        if metadata_action_dim != getattr(model, "nu", None):
            report.errors.append(
                f"Action dim must match MuJoCo actuator count: metadata action_dim={metadata_action_dim}, "
                f"model.nu={getattr(model, 'nu', None)}."
            )

        expected_actuators = self.metadata.get("actuator_names")
        if expected_actuators:
            report.actuator_names_checked = True
            actual_actuators = _mujoco_names(model, "actuator")
            if list(expected_actuators) != actual_actuators:
                report.errors.append("Actuator names/order do not match the MuJoCo model.")
        else:
            report.errors.append("Metadata must provide actuator_names; refusing to guess actuator order.")

        expected_joints = self.metadata.get("joint_names")
        if expected_joints:
            report.joint_names_checked = True
            actual_joints = _mujoco_names(model, "joint")
            missing = [name for name in expected_joints if name not in actual_joints]
            if missing:
                report.errors.append(f"Metadata joint_names include joints missing from model: {missing}")
        else:
            report.warnings.append("Metadata did not provide joint_names; joint order was not checked.")

        for required in ("control_rate_hz", "action_scale"):
            if required not in self.metadata:
                report.errors.append(f"Metadata must provide {required}.")

        report.compatible = len(report.errors) == 0
        self._compatible = bool(report.compatible)
        self.report = report
        return report


class LuckyWalkerPolicyAdapter(LocomotionPolicyAdapter):
    """Adapter for luckyrobots/g1-manipulation-challenge walker.onnx."""

    adapter_name = "lucky_walker"
    real_locomotion = True

    def __init__(self, policy: str = "lucky_walker", repo_root: str | Path | None = None) -> None:
        super().__init__(policy)
        default_repo = PROJECT_ROOT / "third_party" / "g1-manipulation-challenge"
        self.repo_root = Path(repo_root or os.environ.get("LUCKY_G1_REPO", default_repo)).expanduser()
        self.policy_path = self.repo_root / "walker.onnx"
        self.config_path = self.repo_root / "model_config.json"
        self.base_xml_path = self.repo_root / "g1.xml"
        self.external_data_path = self.repo_root / "walker.onnx.data"
        self.config: dict[str, Any] = {}
        self.session: Any | None = None
        self.input_name: str | None = None
        self.output_name: str | None = None
        self.joint_names: list[str] = []
        self.default_joint_pos: np.ndarray | None = None
        self.action_scales: np.ndarray | None = None
        self.last_action = np.zeros(29, dtype=np.float32)
        self.actuator_ids: list[int] = []
        self.joint_qpos_indices: list[int] = []
        self.joint_qvel_indices: list[int] = []

    def prepare_model_xml(self, output_dir: Path) -> Path | None:
        if not self.base_xml_path.exists():
            return None
        scene_path = self.repo_root / "flat_scene_locomotion_sandbox.xml"
        scene_path.write_text(
            f"""<mujoco model="lucky_g1_flat_scene">
  <include file="g1.xml"/>
  <statistic center="0 0 0.9" extent="1.6"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.2 0.2 0.2" specular="0.8 0.8 0.8"/>
    <global azimuth="140" elevation="-20"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
""",
            encoding="utf-8",
        )
        return scene_path

    def reset(self, model: Any, data: Any) -> None:
        if not self.report or not self.report.compatible:
            report = self.compatibility_report(model, Path(""))
            if not report.compatible:
                raise LocomotionPolicyError("; ".join(report.errors))
        self._initialize_model_state(model, data, x=0.0, y=0.0, yaw=0.0)
        self.last_action[:] = 0.0

    def reset_at_pose(self, model: Any, data: Any, x: float, y: float, yaw: float) -> None:
        """Reset Lucky G1 at an explicit world pose for maze/oracle runs."""
        if not self.report or not self.report.compatible:
            report = self.compatibility_report(model, Path(""))
            if not report.compatible:
                raise LocomotionPolicyError("; ".join(report.errors))
        self._initialize_model_state(model, data, x=x, y=y, yaw=yaw)
        self.last_action[:] = 0.0

    def step(self, model: Any, data: Any, command: VelocityCommand, dt: float) -> None:
        if self.session is None or self.default_joint_pos is None or self.action_scales is None:
            raise LocomotionPolicyError("Lucky walker policy has not been loaded.")

        obs = self._build_observation(data, command)
        action = self.session.run([self.output_name], {self.input_name: obs.reshape(1, -1)})[0][0].astype(np.float32)
        if action.shape != (len(self.joint_names),):
            raise LocomotionPolicyError(f"Lucky walker returned action shape {action.shape}, expected {(len(self.joint_names),)}.")

        target_pos = self.default_joint_pos + action * self.action_scales
        for index, name in enumerate(self.joint_names):
            if _is_arm_joint(name):
                target_pos[index] = self.default_joint_pos[index]

        for index, actuator_id in enumerate(self.actuator_ids):
            if actuator_id >= 0:
                data.ctrl[actuator_id] = target_pos[index]
        self.last_action = action.copy()

    def compatibility_report(self, model: Any, model_xml: Path) -> PolicyCompatibilityReport:
        report = PolicyCompatibilityReport(
            policy=str(self.policy_path),
            model_xml=str(model_xml),
            adapter=self.adapter_name,
            loaded=False,
            real_locomotion=True,
            expected_observation_dim=99,
            expected_action_dim=29,
            actual_observation_dim=99,
            actual_action_dim=29,
            model_nu=int(getattr(model, "nu", 0)),
            compatible=False,
        )

        for path in (self.repo_root, self.policy_path, self.external_data_path, self.config_path, self.base_xml_path):
            if not path.exists():
                report.errors.append(f"Required Lucky G1 policy asset does not exist: {path}")
        if report.errors:
            report.errors.append("Run `make fetch-lucky-g1-policy` before POLICY=lucky_walker.")
            self.report = report
            return report

        try:
            self.config = json.loads(self.config_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            report.errors.append(f"Failed to parse Lucky model_config.json: {exc}")
            self.report = report
            return report

        self.joint_names = list(self.config.get("joint_names", []))
        walker_config = self.config.get("walker", {})
        if walker_config.get("input_dim") != 99 or walker_config.get("output_dim") != 29:
            report.errors.append(
                f"Lucky walker metadata expected 99->29, got {walker_config.get('input_dim')}->{walker_config.get('output_dim')}."
            )
        if len(self.joint_names) != 29:
            report.errors.append(f"Lucky config must contain 29 body joint names, got {len(self.joint_names)}.")

        actual_actuators = _mujoco_names(model, "actuator")
        if actual_actuators[: len(self.joint_names)] == self.joint_names:
            report.actuator_names_checked = True
        else:
            report.errors.append("First 29 MuJoCo actuators do not match Lucky walker joint order.")

        actual_joints = _mujoco_names(model, "joint")
        missing_joints = [name for name in self.joint_names if name not in actual_joints]
        report.joint_names_checked = len(missing_joints) == 0
        if missing_joints:
            report.errors.append(f"MuJoCo model is missing Lucky body joints: {missing_joints}")
        if int(getattr(model, "nu", 0)) > 29:
            report.warnings.append(
                f"Model has {int(getattr(model, 'nu', 0))} actuators; Lucky walker controls the first 29 body actuators only."
            )

        try:
            import onnxruntime as ort

            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            options.inter_op_num_threads = 1
            self.session = ort.InferenceSession(str(self.policy_path), options, providers=["CPUExecutionProvider"])
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            report.loaded = True
        except ModuleNotFoundError:
            report.errors.append("onnxruntime is not installed. Run `make setup` after updating requirements.")
        except Exception as exc:
            report.errors.append(f"Failed to load Lucky walker ONNX policy: {exc}")

        if not report.errors:
            self._build_static_arrays(model)
            report.compatible = True
        self.report = report
        return report

    def _build_static_arrays(self, model: Any) -> None:
        default_positions = self.config["default_joint_pos"]
        action_scales = self.config["action_scales"]
        self.default_joint_pos = np.array([default_positions[name] for name in self.joint_names], dtype=np.float32)
        self.action_scales = np.array([action_scales[name] for name in self.joint_names], dtype=np.float32)
        self.actuator_ids = [_mujoco_id(model, "actuator", name) for name in self.joint_names]
        self.joint_qpos_indices = [_joint_qpos_index(model, name) for name in self.joint_names]
        self.joint_qvel_indices = [_joint_qvel_index(model, name) for name in self.joint_names]

    def _initialize_model_state(self, model: Any, data: Any, x: float, y: float, yaw: float) -> None:
        model.opt.timestep = 0.005
        _set_lucky_armature(model, self.joint_names)
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.qpos[0] = float(x)
        data.qpos[1] = float(y)
        data.qpos[2] = 0.76
        data.qpos[3:7] = _yaw_to_quat(float(yaw))
        for index, qpos_index in enumerate(self.joint_qpos_indices):
            data.qpos[qpos_index] = self.default_joint_pos[index]
        for index, actuator_id in enumerate(self.actuator_ids):
            if actuator_id >= 0:
                data.ctrl[actuator_id] = self.default_joint_pos[index]

    def _build_observation(self, data: Any, command: VelocityCommand) -> np.ndarray:
        base_quat = data.qpos[3:7].copy()
        lin_vel = _quat_apply_inverse(base_quat, data.qvel[:3].copy())
        ang_vel = data.qvel[3:6].copy()
        projected_gravity = _quat_apply_inverse(base_quat, np.array([0.0, 0.0, -1.0], dtype=np.float32))
        joint_pos = np.array(
            [data.qpos[index] - self.default_joint_pos[i] for i, index in enumerate(self.joint_qpos_indices)],
            dtype=np.float32,
        )
        joint_vel = np.array([data.qvel[index] for index in self.joint_qvel_indices], dtype=np.float32)
        cmd = np.array([command.vx, command.vy, command.yaw_rate], dtype=np.float32)
        return np.concatenate(
            [lin_vel, ang_vel, projected_gravity, joint_pos, joint_vel, self.last_action, cmd]
        ).astype(np.float32)


class UnitreeRLGymG1PolicyAdapter(LocomotionPolicyAdapter):
    """Experimental regular-G1 bridge for unitreerobotics/unitree_rl_gym motion.pt."""

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
        policy: str = "unitree_rl_gym_g1",
        repo_root: str | Path | None = None,
        native_model: bool = False,
    ) -> None:
        super().__init__(policy)
        self.native_model = native_model
        self.adapter_name = "unitree_rl_gym_native" if native_model else "unitree_rl_gym_g1"
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
        return self.native_xml_path if self.native_model and self.native_xml_path.exists() else None

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
            if self.native_model:
                data.ctrl[actuator_id] = 0.0
            else:
                data.ctrl[actuator_id] = self.default_angles[index]

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

        if self.native_model:
            q = np.array([data.qpos[index] for index in self.joint_qpos_indices], dtype=np.float32)
            dq = np.array([data.qvel[index] for index in self.joint_qvel_indices], dtype=np.float32)
            tau = (self.target_dof_pos - q) * self.kps + (0.0 - dq) * self.kds
            for index, actuator_id in enumerate(self.actuator_ids):
                data.ctrl[actuator_id] = tau[index]
        else:
            for index, actuator_id in enumerate(self.actuator_ids):
                data.ctrl[actuator_id] = self.target_dof_pos[index]
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
            warnings=[] if self.native_model else [
                "Experimental bridge: Unitree RL Gym G1 policy was trained/deployed with Unitree's 12-DoF "
                "torque-actuated XML; this adapter maps its leg target positions onto the regular Menagerie "
                "G1 position actuators and holds the upper body standing."
            ],
        )

        required_paths = [self.repo_root, self.policy_path, self.config_path]
        if self.native_model:
            required_paths.append(self.native_xml_path)
        for path in required_paths:
            if not path.exists():
                report.errors.append(f"Required Unitree RL Gym asset does not exist: {path}")
        if report.errors:
            report.errors.append("Run `make fetch-unitree-rl-gym-policy` before POLICY=unitree_rl_gym_g1.")
            self.report = report
            return report

        actual_actuators = _mujoco_names(model, "actuator")
        expected_actuators = self.leg_joint_names if self.native_model else self.leg_joint_names
        if actual_actuators[:12] == expected_actuators:
            report.actuator_names_checked = True
        else:
            report.errors.append("First 12 MuJoCo actuators do not match Unitree RL Gym G1 leg joint order.")
        if self.native_model and int(getattr(model, "nu", 0)) != 12:
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
                "torch is not installed. Install a CPU Torch wheel before using POLICY=unitree_rl_gym_g1."
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


class ExternalPythonPolicyAdapter(LocomotionPolicyAdapter):
    """Adapter stub for a future Python policy module."""

    adapter_name = "external_python"
    real_locomotion = True

    def __init__(self, policy: str) -> None:
        super().__init__(policy)
        self.module: Any | None = None

    def reset(self, model: Any, data: Any) -> None:
        report = self.compatibility_report(model, Path(""))
        if not report.compatible:
            raise LocomotionPolicyError("; ".join(report.errors))

    def step(self, model: Any, data: Any, command: VelocityCommand, dt: float) -> None:
        raise LocomotionPolicyError("external_python adapter is a documented stub until a policy module contract is set.")

    def compatibility_report(self, model: Any, model_xml: Path) -> PolicyCompatibilityReport:
        report = PolicyCompatibilityReport(
            policy=self.policy,
            model_xml=str(model_xml),
            adapter=self.adapter_name,
            loaded=False,
            real_locomotion=True,
            model_nu=int(getattr(model, "nu", 0)),
            compatible=False,
            errors=["external_python adapter is a stub. Provide a module contract before using it for control."],
        )
        module_name = self.policy.removeprefix("module:")
        if module_name and module_name != "external_python":
            try:
                self.module = importlib.import_module(module_name)
                report.loaded = True
            except Exception as exc:
                report.errors.append(f"Failed to import external policy module {module_name}: {exc}")
        self.report = report
        return report


def create_policy_adapter(
    policy: str | None,
    lucky_g1_repo: str | Path | None = None,
    unitree_rl_gym_repo: str | Path | None = None,
) -> LocomotionPolicyAdapter:
    """Create an adapter from a policy name/path."""
    policy_value = (policy or "placeholder").strip()
    if not policy_value or policy_value == "placeholder":
        return PlaceholderPolicyAdapter("placeholder")

    if policy_value == "lucky_walker":
        return LuckyWalkerPolicyAdapter(policy_value, repo_root=lucky_g1_repo)

    if policy_value == "unitree_rl_gym_g1":
        return UnitreeRLGymG1PolicyAdapter(policy_value, repo_root=unitree_rl_gym_repo)
    if policy_value == "unitree_rl_gym_native":
        return UnitreeRLGymG1PolicyAdapter(policy_value, repo_root=unitree_rl_gym_repo, native_model=True)

    policy_path = Path(policy_value).expanduser()
    if policy_path.name == "walker.onnx" and (policy_path.parent / "model_config.json").exists():
        return LuckyWalkerPolicyAdapter(policy_value, repo_root=policy_path.parent)

    if policy_value.endswith(".onnx") or policy_path.suffix.lower() == ".onnx":
        return OnnxPolicyAdapter(policy_value)

    if policy_value == "external_python" or policy_value.startswith("module:"):
        return ExternalPythonPolicyAdapter(policy_value)

    raise LocomotionPolicyError(
        f"Unknown policy adapter for {policy_value!r}. Use placeholder, a .onnx path, or module:<python.module>."
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


def _last_static_dim(shape: list[Any]) -> int | None:
    if not shape:
        return None
    last = shape[-1]
    return int(last) if isinstance(last, int) and last > 0 else None


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


def _is_arm_joint(name: str) -> bool:
    return any(part in name for part in ("shoulder", "elbow", "wrist"))


def _yaw_to_quat(yaw: float) -> list[float]:
    half = yaw / 2.0
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def _quat_apply_inverse(quat: Any, vec: Any) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float32)
    vec = np.asarray(vec, dtype=np.float32)
    w = quat[0]
    xyz = quat[1:4]
    t = np.cross(xyz, vec) * 2.0
    return vec - w * t + np.cross(xyz, t)


def _set_lucky_armature(model: Any, joint_names: list[str]) -> None:
    arm_5020 = 0.00360972
    arm_7520_14 = 0.01017752
    arm_7520_22 = 0.02510192
    arm_4010 = 0.00425000
    arm_2x5020 = 0.00721945

    for name in joint_names:
        joint_id = _mujoco_id(model, "joint", name)
        if joint_id < 0:
            continue
        dof = int(model.jnt_dofadr[joint_id])
        if "elbow" in name or "shoulder" in name or "wrist_roll" in name:
            model.dof_armature[dof] = arm_5020
        elif "hip_pitch" in name or "hip_yaw" in name or name == "waist_yaw_joint":
            model.dof_armature[dof] = arm_7520_14
        elif "hip_roll" in name or "knee" in name:
            model.dof_armature[dof] = arm_7520_22
        elif "wrist_pitch" in name or "wrist_yaw" in name:
            model.dof_armature[dof] = arm_4010
        elif "ankle" in name or name in ("waist_pitch_joint", "waist_roll_joint"):
            model.dof_armature[dof] = arm_2x5020
        else:
            model.dof_armature[dof] = arm_5020


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
