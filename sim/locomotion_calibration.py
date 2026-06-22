"""Direct MuJoCo command-sweep calibration for the Unitree G1 walking policy."""

from __future__ import annotations

from dataclasses import dataclass
from html import escape
from pathlib import Path
from typing import Any, Iterable
import csv
import hashlib
import json
import math
import statistics

import yaml

from sim.config import load_config
from sim.locomotion_policy_adapter import VelocityCommand, create_policy_adapter
from sim.locomotion_sandbox import base_state, save_render
from sim.mujoco_runner import PROJECT_ROOT, import_mujoco, resolve_project_path


RESULT_FIELDS = (
    "group",
    "cmd_vx",
    "cmd_wz",
    "duration_s",
    "actual_vx_mean",
    "actual_vx_median",
    "actual_vx_std",
    "actual_wz_mean",
    "actual_wz_median",
    "actual_wz_std",
    "distance_travelled_m",
    "yaw_changed_rad",
    "lateral_drift_m",
    "cmd_arc_radius_m",
    "actual_arc_radius_m",
    "fell",
    "stuck",
    "contact",
    "non_floor_contact",
    "failure_reason",
    "vx_tracking_error_mean_abs",
    "vx_tracking_error_rms",
    "wz_tracking_error_mean_abs",
    "wz_tracking_error_rms",
    "max_roll_rad",
    "max_pitch_rad",
    "min_base_height_m",
    "stability_score",
    "stable",
)


@dataclass(frozen=True)
class CalibrationCommand:
    group: str
    vx: float
    wz: float


@dataclass(frozen=True)
class CalibrationSuite:
    policy: str
    profile: str
    timing: dict[str, float]
    stability: dict[str, float]
    commands: list[CalibrationCommand]
    raw_config: dict[str, Any]


def load_calibration_suite(path: Path, profile: str | None = None) -> CalibrationSuite:
    """Load a calibration suite config and expand the requested command profile."""
    values = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(values, dict) or not isinstance(values.get("g1_locomotion_calibration"), dict):
        raise ValueError(f"Calibration config must contain g1_locomotion_calibration: {path}")
    config = values["g1_locomotion_calibration"]
    selected_profile = str(profile or config.get("default_profile", "balanced"))
    timing = dict(config.get("timing", {}))
    stability = dict(config.get("stability", {}))

    if selected_profile == "smoke":
        smoke = config.get("smoke", {})
        timing.update(smoke.get("timing", {}))
        commands = [
            CalibrationCommand(str(row["group"]), float(row["vx"]), float(row["wz"]))
            for row in smoke.get("commands", [])
        ]
    elif selected_profile == "balanced":
        commands = expand_command_grid(config.get("sweeps", {}))
    else:
        raise ValueError(f"Unsupported calibration profile {selected_profile!r}. Use balanced or smoke.")
    if not commands:
        raise ValueError(f"Calibration profile {selected_profile!r} produced no commands.")

    return CalibrationSuite(
        policy=str(config.get("policy", "unitree_rl_gym_native")),
        profile=selected_profile,
        timing={key: float(value) for key, value in timing.items()},
        stability={key: float(value) for key, value in stability.items()},
        commands=commands,
        raw_config=config,
    )


def expand_command_grid(sweeps: dict[str, Any]) -> list[CalibrationCommand]:
    """Expand YAML sweep groups into deterministic command rows."""
    commands: list[CalibrationCommand] = []
    for group in ("straight", "reverse_recovery", "pure_rotation", "arc"):
        values = sweeps.get(group, {})
        for vx in values.get("vx", []):
            for wz in values.get("wz", []):
                commands.append(CalibrationCommand(group=group, vx=float(vx), wz=float(wz)))
    return commands


def arc_radius(vx: float, wz: float, epsilon: float = 0.05) -> float | str:
    """Return vx/wz, or the JSON-safe string 'inf' for near-straight commands."""
    if abs(float(wz)) < float(epsilon):
        return "inf"
    return float(vx) / float(wz)


def summarize_trial(
    *,
    command: CalibrationCommand,
    duration_s: float,
    samples: list[dict[str, float | bool]],
    stability: dict[str, float],
    fell: bool = False,
    runtime_error: str = "",
) -> dict[str, Any]:
    """Compute one command-row metric payload from sampled MuJoCo state."""
    epsilon = float(stability.get("arc_infinity_wz_epsilon", 0.05))
    vx_values = [float(row["actual_vx"]) for row in samples]
    wz_values = [float(row["actual_wz"]) for row in samples]
    z_values = [float(row["base_z"]) for row in samples]
    roll_values = [abs(float(row["roll"])) for row in samples]
    pitch_values = [abs(float(row["pitch"])) for row in samples]
    contact = any(bool(row.get("contact")) for row in samples)
    non_floor_contact = any(bool(row.get("non_floor_contact")) for row in samples)
    nonfinite = bool(runtime_error) or any(
        not math.isfinite(float(value))
        for row in samples
        for value in (
            row.get("actual_vx", 0.0),
            row.get("actual_wz", 0.0),
            row.get("base_z", 0.0),
            row.get("roll", 0.0),
            row.get("pitch", 0.0),
        )
    )
    distance = float(samples[-1]["distance_m"]) if samples else 0.0
    yaw_changed = float(samples[-1]["yaw_changed_rad"]) if samples else 0.0
    lateral_drift = float(samples[-1]["lateral_drift_m"]) if samples else 0.0
    stuck = _is_stuck(command, duration_s, samples, stability)
    vx_error_abs = [abs(value - command.vx) for value in vx_values]
    wz_error_abs = [abs(value - command.wz) for value in wz_values]

    row: dict[str, Any] = {
        "group": command.group,
        "cmd_vx": command.vx,
        "cmd_wz": command.wz,
        "duration_s": float(duration_s),
        "actual_vx_mean": _mean(vx_values),
        "actual_vx_median": _median(vx_values),
        "actual_vx_std": _std(vx_values),
        "actual_wz_mean": _mean(wz_values),
        "actual_wz_median": _median(wz_values),
        "actual_wz_std": _std(wz_values),
        "distance_travelled_m": distance,
        "yaw_changed_rad": yaw_changed,
        "lateral_drift_m": lateral_drift,
        "cmd_arc_radius_m": arc_radius(command.vx, command.wz, epsilon),
        "actual_arc_radius_m": arc_radius(_mean(vx_values), _mean(wz_values), epsilon),
        "fell": bool(fell),
        "stuck": bool(stuck),
        "contact": bool(contact),
        "non_floor_contact": bool(non_floor_contact),
        "failure_reason": "",
        "vx_tracking_error_mean_abs": _mean(vx_error_abs),
        "vx_tracking_error_rms": _rms([value - command.vx for value in vx_values]),
        "wz_tracking_error_mean_abs": _mean(wz_error_abs),
        "wz_tracking_error_rms": _rms([value - command.wz for value in wz_values]),
        "max_roll_rad": max(roll_values, default=0.0),
        "max_pitch_rad": max(pitch_values, default=0.0),
        "min_base_height_m": min(z_values, default=0.0),
        "stability_score": 0.0,
        "stable": False,
    }
    score = stability_score(row)
    row["stability_score"] = score
    threshold = float(stability.get("stable_score_threshold", 75.0))
    row["stable"] = bool(
        not row["fell"]
        and not row["non_floor_contact"]
        and not row["stuck"]
        and not nonfinite
        and score >= threshold
    )
    row["failure_reason"] = failure_reason(row, runtime_error=runtime_error, nonfinite=nonfinite)
    return row


def stability_score(row: dict[str, Any]) -> float:
    """Score a command row on a 0-100 stability scale."""
    if row.get("fell") or row.get("runtime_error"):
        return 0.0
    score = 100.0
    score -= min(35.0, abs(float(row.get("vx_tracking_error_mean_abs", 0.0))) * 35.0)
    score -= min(35.0, abs(float(row.get("wz_tracking_error_mean_abs", 0.0))) * 22.0)
    score -= min(18.0, max(0.0, float(row.get("max_roll_rad", 0.0)) - 0.35) * 45.0)
    score -= min(18.0, max(0.0, float(row.get("max_pitch_rad", 0.0)) - 0.35) * 45.0)
    score -= min(15.0, max(0.0, 0.75 - float(row.get("min_base_height_m", 0.75))) * 45.0)
    score -= min(12.0, abs(float(row.get("lateral_drift_m", 0.0))) * 10.0)
    score -= min(12.0, float(row.get("actual_vx_std", 0.0)) * 12.0)
    score -= min(12.0, float(row.get("actual_wz_std", 0.0)) * 7.0)
    if row.get("contact"):
        score -= 3.0
    if row.get("non_floor_contact"):
        score -= 50.0
    if row.get("stuck"):
        score -= 40.0
    return max(0.0, min(100.0, score))


def failure_reason(row: dict[str, Any], *, runtime_error: str = "", nonfinite: bool = False) -> str:
    if runtime_error:
        return f"runtime_error:{runtime_error}"
    if nonfinite:
        return "non_finite_state"
    if bool(row.get("fell")):
        return "fall"
    if bool(row.get("non_floor_contact")):
        return "non_floor_contact"
    if bool(row.get("stuck")):
        return "stuck"
    if not bool(row.get("stable")):
        return "low_stability_score"
    return ""


def recommend_safe_limits(rows: list[dict[str, Any]], stability: dict[str, float]) -> dict[str, Any]:
    """Select conservative command recommendations from measured rows."""
    max_vx_error = float(stability.get("preferred_max_vx_error_mps", 0.35))
    max_wz_error = float(stability.get("preferred_max_wz_error_radps", 0.35))
    max_drift = float(stability.get("preferred_max_lateral_drift_m", 0.50))

    stable_straight = [
        row for row in rows
        if row.get("group") == "straight"
        and _row_stable(row)
        and float(row.get("cmd_vx", 0.0)) > 0.0
        and float(row.get("vx_tracking_error_mean_abs", 0.0)) <= max_vx_error
        and abs(float(row.get("lateral_drift_m", 0.0))) <= max_drift
    ]
    max_safe_vx = max((float(row["cmd_vx"]) for row in stable_straight), default=0.0)

    stable_rotation = [
        row for row in rows
        if row.get("group") == "pure_rotation"
        and _row_stable(row)
        and float(row.get("wz_tracking_error_mean_abs", 0.0)) <= max_wz_error
    ]
    positive_wz = max((float(row["cmd_wz"]) for row in stable_rotation if float(row["cmd_wz"]) > 0.0), default=0.0)
    negative_wz = max((abs(float(row["cmd_wz"])) for row in stable_rotation if float(row["cmd_wz"]) < 0.0), default=0.0)
    max_safe_wz = min(positive_wz, negative_wz) if positive_wz and negative_wz else max(positive_wz, negative_wz)
    turn_start = round(max(0.0, max_safe_wz * 0.35), 6)
    turn_full = round(max(turn_start, max_safe_wz * 0.80), 6)

    stable_arcs = [
        row for row in rows
        if row.get("group") == "arc" and _row_stable(row)
    ]
    ranked_arcs = sorted(
        stable_arcs,
        key=lambda row: (-float(row.get("stability_score", 0.0)), -abs(float(row.get("actual_vx_mean", 0.0)))),
    )
    preferred_arc_commands = {
        "wide_curve": [_command_summary(row) for row in ranked_arcs if _abs_radius(row.get("actual_arc_radius_m")) >= 1.0][:8],
        "tight_turn": [_command_summary(row) for row in ranked_arcs if _abs_radius(row.get("actual_arc_radius_m")) < 1.0][:8],
    }
    recovery_rows = sorted(
        [row for row in rows if row.get("group") == "reverse_recovery" and _row_stable(row)],
        key=lambda row: (-float(row.get("stability_score", 0.0)), abs(float(row.get("cmd_vx", 0.0)))),
    )
    unsafe = [
        {
            "group": row.get("group"),
            "cmd_vx": row.get("cmd_vx"),
            "cmd_wz": row.get("cmd_wz"),
            "stability_score": row.get("stability_score"),
            "failure_reason": row.get("failure_reason") or "low_stability_score",
        }
        for row in rows
        if not _row_stable(row)
    ]
    return {
        "max_safe_vx": max_safe_vx,
        "max_safe_wz": max_safe_wz,
        "turn_slowdown_start_radps": turn_start,
        "turn_slowdown_full_radps": turn_full,
        "preferred_arc_commands": preferred_arc_commands,
        "unsafe_commands": unsafe,
        "recovery_safe_commands": [_command_summary(row) for row in recovery_rows[:8]],
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=RESULT_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


class G1LocomotionCalibrationRunner:
    """Run a command sweep against the frozen Unitree G1 locomotion policy."""

    def __init__(
        self,
        *,
        project_config_path: Path,
        calibration_config_path: Path,
        suite: CalibrationSuite,
        run_dir: Path,
        seed: int,
        unitree_rl_gym_repo: Path,
        friction_scale: float = 1.0,
    ) -> None:
        self.project_config_path = project_config_path
        self.calibration_config_path = calibration_config_path
        self.suite = suite
        self.run_dir = run_dir
        self.seed = int(seed)
        self.unitree_rl_gym_repo = unitree_rl_gym_repo
        self.friction_scale = float(friction_scale)
        self.project_config = load_config(project_config_path)

    def run(self) -> dict[str, Any]:
        self.run_dir.mkdir(parents=True, exist_ok=True)
        mujoco = import_mujoco()
        model_xml = self._model_xml_path()
        model = mujoco.MjModel.from_xml_path(str(model_xml))
        _scale_model_friction(model, self.friction_scale)
        data = mujoco.MjData(model)
        adapter = create_policy_adapter(self.suite.policy, unitree_rl_gym_repo=self.unitree_rl_gym_repo)
        report = adapter.compatibility_report(model, model_xml)
        write_json(self.run_dir / "policy_compatibility.json", report.to_dict())
        if report.errors:
            raise RuntimeError("; ".join(report.errors))

        rows = []
        for index, command in enumerate(self.suite.commands, start=1):
            print(
                f"calibration {index}/{len(self.suite.commands)} "
                f"{command.group} vx={command.vx:.3f} wz={command.wz:.3f}",
                flush=True,
            )
            rows.append(self._run_trial(mujoco, model, data, adapter, command))

        recommendations = recommend_safe_limits(rows, self.suite.stability)
        summary = self._summary(rows, recommendations, model_xml)
        write_csv(self.run_dir / "command_results.csv", rows)
        write_json(self.run_dir / "command_results.json", rows)
        write_json(self.run_dir / "summary.json", summary)
        write_json(self.run_dir / "locomotion_calibration.json", summary["locomotion_calibration"])
        (self.run_dir / "report.md").write_text(render_markdown_report(summary, rows), encoding="utf-8")
        (self.run_dir / "dashboard.html").write_text(render_html_report(summary, rows), encoding="utf-8")
        save_render(
            mujoco,
            model,
            data,
            self.run_dir / "final_render.png",
            int(self.project_config.get("locomotion_sandbox", {}).get("render_width", 640)),
            int(self.project_config.get("locomotion_sandbox", {}).get("render_height", 480)),
        )
        return summary

    def _run_trial(self, mujoco: Any, model: Any, data: Any, adapter: Any, command: CalibrationCommand) -> dict[str, Any]:
        self._reset_trial(mujoco, model, data, adapter)
        runtime_error = ""
        fell = False
        try:
            fell = self._step_duration(mujoco, model, data, adapter, VelocityCommand(), self._timing("warmup_s"), None)
            samples = []
            if not fell:
                fell = self._step_duration(
                    mujoco,
                    model,
                    data,
                    adapter,
                    VelocityCommand(vx=command.vx, yaw_rate=command.wz),
                    self._timing("command_s"),
                    samples,
                )
            self._step_duration(mujoco, model, data, adapter, VelocityCommand(), self._timing("settle_s"), None)
        except Exception as exc:  # MuJoCo or Torch failures should become row-level evidence.
            runtime_error = str(exc)
            samples = []
            fell = True
        return summarize_trial(
            command=command,
            duration_s=self._timing("command_s"),
            samples=samples,
            stability=self.suite.stability,
            fell=fell,
            runtime_error=runtime_error,
        )

    def _step_duration(
        self,
        mujoco: Any,
        model: Any,
        data: Any,
        adapter: Any,
        command: VelocityCommand,
        duration_s: float,
        samples: list[dict[str, float | bool]] | None,
    ) -> bool:
        if duration_s <= 0.0:
            return False
        control_dt = 1.0 / max(1e-6, self._timing("control_rate_hz"))
        control_steps = max(1, int(round(duration_s / control_dt)))
        substeps = max(1, round(control_dt / float(model.opt.timestep)))
        requires_substep = bool(getattr(adapter, "requires_substep_control", False))
        sample_origin = _SampleOrigin.from_state(base_state(data))
        previous_xy = sample_origin.xy
        previous_yaw = sample_origin.yaw
        distance = 0.0
        yaw_changed = 0.0
        low_motion_s = 0.0
        fell = False

        for _ in range(control_steps):
            if not requires_substep:
                adapter.step(model, data, command, control_dt)
            substep_dt = float(model.opt.timestep)
            for _substep in range(substeps):
                if requires_substep:
                    adapter.step(model, data, command, substep_dt)
                mujoco.mj_step(model, data)
            state = base_state(data)
            xy = (float(state["base_x"]), float(state["base_y"]))
            distance += math.hypot(xy[0] - previous_xy[0], xy[1] - previous_xy[1])
            yaw_delta = _angle_delta(float(state["yaw"]), previous_yaw)
            yaw_changed += yaw_delta
            previous_xy = xy
            previous_yaw = float(state["yaw"])
            actual_vx, actual_wz = _body_vx_and_yaw_rate(data, state)
            low_motion = (
                abs(actual_vx) < float(self.suite.stability.get("stuck_speed_threshold_mps", 0.03))
                and abs(actual_wz) < float(self.suite.stability.get("stuck_yaw_threshold_radps", 0.05))
                and (abs(command.vx) > 0.03 or abs(command.yaw_rate) > 0.05)
            )
            low_motion_s = low_motion_s + control_dt if low_motion else 0.0
            contacts = _contact_info(mujoco, model, data)
            lateral = _lateral_drift(sample_origin, xy)
            if samples is not None:
                samples.append(
                    {
                        "actual_vx": actual_vx,
                        "actual_wz": actual_wz,
                        "base_z": float(state["base_z"]),
                        "roll": float(state["roll"]),
                        "pitch": float(state["pitch"]),
                        "distance_m": distance,
                        "yaw_changed_rad": yaw_changed,
                        "lateral_drift_m": lateral,
                        "contact": contacts["contact"],
                        "non_floor_contact": contacts["non_floor_contact"],
                        "low_motion_s": low_motion_s,
                    }
                )
            if _fallen(state, self.suite.stability):
                fell = True
                break
        return fell

    def _reset_trial(self, mujoco: Any, model: Any, data: Any, adapter: Any) -> None:
        adapter.reset(model, data)
        data.qvel[:] = 0.0
        data.qpos[0] = 0.0
        data.qpos[1] = 0.0
        data.qpos[2] = float(self.project_config.get("robot", {}).get("initial_base_height_m", data.qpos[2]))
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(model, data)

    def _model_xml_path(self) -> Path:
        raw = self.project_config.get("robot", {}).get("model_xml_path")
        if raw:
            return resolve_project_path(raw, PROJECT_ROOT)
        return self.unitree_rl_gym_repo / "resources" / "robots" / "g1_description" / "scene.xml"

    def _summary(self, rows: list[dict[str, Any]], recommendations: dict[str, Any], model_xml: Path) -> dict[str, Any]:
        stable_rows = [row for row in rows if _row_stable(row)]
        policy_path = self.unitree_rl_gym_repo / "deploy" / "pre_train" / "g1" / "motion.pt"
        deploy_config = self.unitree_rl_gym_repo / "deploy" / "deploy_mujoco" / "configs" / "g1.yaml"
        policy_hash = _sha256(policy_path)
        model_config_hash = _combined_hash([model_xml, deploy_config])
        cache_key = hashlib.sha256(
            f"{self.suite.policy}:{policy_hash}:{model_config_hash}:{self.suite.profile}:{self.friction_scale:.6f}".encode(
                "utf-8"
            )
        ).hexdigest()[:16]
        selected_max_forward = float(recommendations.get("max_safe_vx", 0.0))
        locomotion_calibration = {
            "schema_version": 2,
            "policy": self.suite.policy,
            "policy_hash": policy_hash,
            "model_config_hash": model_config_hash,
            "cache_key": cache_key,
            "status": "passed" if stable_rows else "failed",
            "source": "direct_mujoco_command_sweep",
            "profile": self.suite.profile,
            "environment": {
                "friction_scale": self.friction_scale,
            },
            "selected_max_forward_mps": selected_max_forward,
            "command_limits": {
                "min_forward_mps": min((float(row["cmd_vx"]) for row in stable_rows if float(row["cmd_vx"]) > 0), default=0.0),
                "max_forward_mps": selected_max_forward,
                "max_reverse_mps": min((float(row["cmd_vx"]) for row in stable_rows if float(row["cmd_vx"]) < 0), default=0.0),
                "min_yaw_rate_radps": 0.0,
                "max_yaw_rate_radps": float(recommendations.get("max_safe_wz", 0.0)),
            },
            "recommended_safe_limits": {
                key: value
                for key, value in recommendations.items()
                if key not in {"preferred_arc_commands", "unsafe_commands", "recovery_safe_commands"}
            },
            "preferred_arc_commands": recommendations["preferred_arc_commands"],
            "unsafe_commands": recommendations["unsafe_commands"],
            "recovery_safe_commands": recommendations["recovery_safe_commands"],
            "ground_truth_used_for_calibration_metrics": True,
        }
        return {
            "schema_version": 1,
            "status": "passed" if stable_rows else "failed",
            "profile": self.suite.profile,
            "seed": self.seed,
            "policy": self.suite.policy,
            "environment": {
                "friction_scale": self.friction_scale,
            },
            "total_commands": len(rows),
            "stable_commands": len(stable_rows),
            "unstable_commands": len(rows) - len(stable_rows),
            "fall_count": sum(1 for row in rows if row.get("fell")),
            "stuck_count": sum(1 for row in rows if row.get("stuck")),
            "non_floor_contact_count": sum(1 for row in rows if row.get("non_floor_contact")),
            "recommended_safe_limits": recommendations,
            "locomotion_calibration": locomotion_calibration,
            "artifacts": {
                "command_results_csv": str(self.run_dir / "command_results.csv"),
                "command_results_json": str(self.run_dir / "command_results.json"),
                "locomotion_calibration_json": str(self.run_dir / "locomotion_calibration.json"),
                "report_md": str(self.run_dir / "report.md"),
                "dashboard_html": str(self.run_dir / "dashboard.html"),
                "final_render": str(self.run_dir / "final_render.png"),
                "policy_compatibility": str(self.run_dir / "policy_compatibility.json"),
            },
        }

    def _timing(self, key: str) -> float:
        return float(self.suite.timing.get(key, 0.0))


@dataclass(frozen=True)
class _SampleOrigin:
    xy: tuple[float, float]
    yaw: float

    @classmethod
    def from_state(cls, state: dict[str, float]) -> "_SampleOrigin":
        return cls((float(state["base_x"]), float(state["base_y"])), float(state["yaw"]))


def render_markdown_report(summary: dict[str, Any], rows: list[dict[str, Any]]) -> str:
    limits = summary["recommended_safe_limits"]
    lines = [
        "# G1 Locomotion Calibration Report",
        "",
        f"- Profile: `{summary['profile']}`",
        f"- Commands: {summary['stable_commands']}/{summary['total_commands']} stable",
        f"- Max safe vx: {limits.get('max_safe_vx', 0.0):.3f} m/s",
        f"- Max safe wz: {limits.get('max_safe_wz', 0.0):.3f} rad/s",
        f"- Falls: {summary['fall_count']}",
        f"- Stuck commands: {summary['stuck_count']}",
        f"- Non-floor contacts: {summary['non_floor_contact_count']}",
        "",
        "## Unsafe Commands",
        "",
    ]
    unsafe = limits.get("unsafe_commands", [])[:20]
    if not unsafe:
        lines.append("No unsafe commands in this sweep.")
    else:
        lines.extend(
            f"- `{row['group']}` vx={float(row['cmd_vx']):.3f}, wz={float(row['cmd_wz']):.3f}: {row['failure_reason']}"
            for row in unsafe
        )
    lines.extend(["", "## Top Stable Commands", ""])
    top_rows = sorted(rows, key=lambda row: -float(row.get("stability_score", 0.0)))[:12]
    lines.extend(
        f"- `{row['group']}` vx={float(row['cmd_vx']):.3f}, wz={float(row['cmd_wz']):.3f}, score={float(row['stability_score']):.1f}, stable={row['stable']}"
        for row in top_rows
    )
    lines.append("")
    lines.append("Ground truth is used only for calibration metrics and scalar limit recommendations, not for live navigation.")
    return "\n".join(lines) + "\n"


def render_html_report(summary: dict[str, Any], rows: list[dict[str, Any]]) -> str:
    limits = summary["recommended_safe_limits"]
    row_html = "".join(
        "<tr>"
        f"<td>{escape(str(row['group']))}</td>"
        f"<td>{float(row['cmd_vx']):.3f}</td>"
        f"<td>{float(row['cmd_wz']):.3f}</td>"
        f"<td>{float(row['actual_vx_mean']):.3f}</td>"
        f"<td>{float(row['actual_wz_mean']):.3f}</td>"
        f"<td>{float(row['stability_score']):.1f}</td>"
        f"<td>{escape(str(row['stable']))}</td>"
        f"<td>{escape(str(row['failure_reason']))}</td>"
        "</tr>"
        for row in rows
    )
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>G1 Locomotion Calibration</title>
  <style>
    body {{ margin: 0; font-family: Arial, sans-serif; color: #1f2933; background: #f7f8fa; }}
    main {{ max-width: 1180px; margin: 0 auto; padding: 28px; }}
    table {{ width: 100%; border-collapse: collapse; background: white; border: 1px solid #d9e0ea; }}
    th, td {{ padding: 8px 10px; border-bottom: 1px solid #edf0f4; text-align: left; }}
    th {{ background: #eef3f8; }}
    .kpis {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; margin: 16px 0; }}
    .kpi {{ background: white; border: 1px solid #d9e0ea; padding: 12px; }}
    .label {{ color: #52606d; font-size: 13px; }}
    .value {{ font-size: 24px; font-weight: 700; margin-top: 4px; }}
  </style>
</head>
<body>
<main>
  <h1>G1 Locomotion Calibration</h1>
  <p>Direct MuJoCo command sweep. Ground truth is used only for calibration metrics and scalar recommendations.</p>
  <section class="kpis">
    <div class="kpi"><div class="label">Stable Commands</div><div class="value">{summary['stable_commands']}/{summary['total_commands']}</div></div>
    <div class="kpi"><div class="label">Max Safe vx</div><div class="value">{float(limits.get('max_safe_vx', 0.0)):.3f}</div></div>
    <div class="kpi"><div class="label">Max Safe wz</div><div class="value">{float(limits.get('max_safe_wz', 0.0)):.3f}</div></div>
    <div class="kpi"><div class="label">Falls</div><div class="value">{summary['fall_count']}</div></div>
  </section>
  <table>
    <thead><tr><th>Group</th><th>cmd vx</th><th>cmd wz</th><th>actual vx</th><th>actual wz</th><th>Score</th><th>Stable</th><th>Reason</th></tr></thead>
    <tbody>{row_html}</tbody>
  </table>
</main>
</body>
</html>
"""


def _scale_model_friction(model: Any, scale: float) -> None:
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError(f"friction_scale must be a positive finite value, got {scale!r}")
    if abs(scale - 1.0) < 1e-9 or not hasattr(model, "geom_friction"):
        return
    model.geom_friction[:, :] *= scale


def _is_stuck(
    command: CalibrationCommand,
    duration_s: float,
    samples: list[dict[str, float | bool]],
    stability: dict[str, float],
) -> bool:
    if not samples or (abs(command.vx) <= 0.03 and abs(command.wz) <= 0.05):
        return False
    fraction = float(stability.get("stuck_expected_fraction", 0.25))
    distance = abs(float(samples[-1]["distance_m"]))
    yaw = abs(float(samples[-1]["yaw_changed_rad"]))
    expected_distance = abs(command.vx) * duration_s
    expected_yaw = abs(command.wz) * duration_s
    distance_failed = expected_distance > 0.05 and distance < expected_distance * fraction
    yaw_failed = expected_yaw > 0.10 and yaw < expected_yaw * fraction
    checks = []
    if expected_distance > 0.05:
        checks.append(distance_failed)
    if expected_yaw > 0.10:
        checks.append(yaw_failed)
    low_motion_timeout = any(
        float(row.get("low_motion_s", 0.0)) >= float(stability.get("stuck_duration_s", 2.0))
        for row in samples
    )
    return (bool(checks) and all(checks)) or low_motion_timeout


def _fallen(state: dict[str, float], stability: dict[str, float]) -> bool:
    if float(state["base_z"]) < float(stability.get("fall_min_base_height_m", 0.45)):
        return True
    return (
        abs(float(state["roll"])) > float(stability.get("fall_max_tilt_rad", 0.9))
        or abs(float(state["pitch"])) > float(stability.get("fall_max_tilt_rad", 0.9))
    )


def _body_vx_and_yaw_rate(data: Any, state: dict[str, float]) -> tuple[float, float]:
    yaw = float(state["yaw"])
    world_vx, world_vy = float(data.qvel[0]), float(data.qvel[1])
    body_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
    return body_vx, float(data.qvel[5])


def _contact_info(mujoco: Any, model: Any, data: Any) -> dict[str, bool]:
    contact_count = int(getattr(data, "ncon", 0))
    non_floor = False
    for index in range(contact_count):
        contact = data.contact[index]
        names = (_geom_name(mujoco, model, int(contact.geom1)), _geom_name(mujoco, model, int(contact.geom2)))
        if not any(_is_floor_name(name) for name in names):
            non_floor = True
            break
    return {"contact": contact_count > 0, "non_floor_contact": non_floor}


def _geom_name(mujoco: Any, model: Any, geom_id: int) -> str:
    if geom_id < 0:
        return ""
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
    if name:
        return str(name)
    body_id = int(model.geom_bodyid[geom_id])
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    return f"geom_{geom_id}:{body_name or f'body_{body_id}'}"


def _is_floor_name(name: str) -> bool:
    return name in {"floor", "groundplane", "maze_floor"} or "floor" in name.lower() or "ground" in name.lower()


def _angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def _lateral_drift(origin: _SampleOrigin, xy: tuple[float, float]) -> float:
    dx = xy[0] - origin.xy[0]
    dy = xy[1] - origin.xy[1]
    return -math.sin(origin.yaw) * dx + math.cos(origin.yaw) * dy


def _row_stable(row: dict[str, Any]) -> bool:
    value = row.get("stable")
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes"}
    return bool(value)


def _command_summary(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "group": row.get("group"),
        "cmd_vx": row.get("cmd_vx"),
        "cmd_wz": row.get("cmd_wz"),
        "actual_vx_mean": row.get("actual_vx_mean"),
        "actual_wz_mean": row.get("actual_wz_mean"),
        "stability_score": row.get("stability_score"),
    }


def _abs_radius(value: Any) -> float:
    if value == "inf":
        return float("inf")
    return abs(float(value))


def _mean(values: Iterable[float]) -> float:
    rows = list(values)
    return sum(rows) / len(rows) if rows else 0.0


def _median(values: Iterable[float]) -> float:
    rows = list(values)
    return float(statistics.median(rows)) if rows else 0.0


def _std(values: Iterable[float]) -> float:
    rows = list(values)
    return float(statistics.pstdev(rows)) if len(rows) > 1 else 0.0


def _rms(values: Iterable[float]) -> float:
    rows = list(values)
    return math.sqrt(sum(value * value for value in rows) / len(rows)) if rows else 0.0


def _sha256(path: Path) -> str | None:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def _combined_hash(paths: list[Path]) -> str | None:
    digest = hashlib.sha256()
    found = False
    for path in paths:
        if path.is_file():
            found = True
            digest.update(path.read_bytes())
    return digest.hexdigest() if found else None
