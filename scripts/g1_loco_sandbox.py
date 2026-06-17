"""Run the visual G1 locomotion policy sandbox."""

from __future__ import annotations

from pathlib import Path
import argparse
import json
import os
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sim.config import ConfigError, load_config
from sim.locomotion_policy_adapter import LocomotionPolicyError, create_policy_adapter
from sim.locomotion_sandbox import (
    CommandLogger,
    RecordingManager,
    SandboxArtifacts,
    StateLogger,
    TeleopController,
    base_state,
    config_from_dict,
    determine_status,
    make_recording_timestamp,
    save_render,
    write_dashboard,
    write_summary,
)
from sim.mujoco_runner import MuJoCoImportError, MuJoCoModelError, import_mujoco, resolve_project_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visual flat-ground Unitree G1 locomotion policy sandbox.")
    parser.add_argument("--policy", default="placeholder", help="Policy name/path: placeholder, .onnx path, or module:name.")
    parser.add_argument("--viewer", action="store_true", help="Open the live MuJoCo viewer with teleop controls.")
    parser.add_argument("--duration", type=float, default=3.0, help="Headless or max live run duration in seconds.")
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "runs" / "visual")
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs" / "default.yaml")
    parser.add_argument(
        "--lucky-g1-repo",
        type=Path,
        default=PROJECT_ROOT / "third_party" / "g1-manipulation-challenge",
        help="Local clone of luckyrobots/g1-manipulation-challenge for POLICY=lucky_walker.",
    )
    parser.add_argument(
        "--unitree-rl-gym-repo",
        type=Path,
        default=PROJECT_ROOT / "third_party" / "unitree_rl_gym",
        help="Local clone of unitreerobotics/unitree_rl_gym for POLICY=unitree_rl_gym_g1.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    artifacts = SandboxArtifacts.latest(args.output_dir)
    artifacts.output_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    summary: dict[str, object] = {
        "policy": args.policy,
        "adapter": "unknown",
        "model_xml_path": None,
        "real_locomotion": False,
        "final_status": "error",
        "fallen": False,
        "recording_used": False,
        "recording_path": None,
        "command_log_path": str(artifacts.commands_csv),
        "state_log_path": str(artifacts.state_csv),
        "final_render_path": str(artifacts.final_render),
        "compatibility_report_path": str(artifacts.compatibility_json),
        "dashboard_path": str(artifacts.dashboard_html),
        "summary_path": str(artifacts.summary_json),
        "rerun_command": f"make {'g1-loco-sandbox' if args.viewer else 'g1-loco-view'} POLICY={args.policy}",
        "viewer_requested": bool(args.viewer),
        "viewer_opened": False,
        "error": None,
    }

    command_logger: CommandLogger | None = None
    state_logger: StateLogger | None = None
    recorder: RecordingManager | None = None

    try:
        config = load_config(args.config)
        sandbox_config = config_from_dict(config)
        adapter = create_policy_adapter(
            args.policy,
            lucky_g1_repo=args.lucky_g1_repo,
            unitree_rl_gym_repo=args.unitree_rl_gym_repo,
        )
        apply_policy_teleop_defaults(adapter.adapter_name, sandbox_config)
        prepared_model_xml = adapter.prepare_model_xml(artifacts.output_dir)
        model_xml = prepared_model_xml or resolve_project_path(config["robot"]["model_xml_path"], PROJECT_ROOT)
        summary["model_xml_path"] = str(model_xml)
        if not model_xml.exists():
            raise MuJoCoModelError(f"G1 flat-ground model XML does not exist: {model_xml}")

        mujoco = import_mujoco()
        model = mujoco.MjModel.from_xml_path(str(model_xml))
        if adapter.adapter_name != "lucky_walker":
            model.opt.timestep = float(config["sim"].get("timestep", model.opt.timestep))
        data = mujoco.MjData(model)
        if adapter.adapter_name != "lucky_walker":
            reset_to_keyframe(mujoco, model, data, config.get("robot", {}).get("initial_keyframe", "stand"))
            mujoco.mj_forward(model, data)
        summary["adapter"] = adapter.adapter_name
        summary["real_locomotion"] = adapter.real_locomotion
        report = adapter.compatibility_report(model, model_xml)
        report.write_json(artifacts.compatibility_json)
        if report.errors and adapter.adapter_name != "placeholder":
            raise LocomotionPolicyError("; ".join(report.errors))
        adapter.reset(model, data)
        mujoco.mj_forward(model, data)

        command_logger = CommandLogger(artifacts.commands_csv)
        state_logger = StateLogger(artifacts.state_csv)
        recorder = RecordingManager(artifacts.output_dir, "g1_loco", sandbox_config.recording_fps)

        print_controls(args.policy, adapter.adapter_name, adapter.real_locomotion)
        if adapter.adapter_name == "placeholder":
            print("No real walking policy loaded. This mode validates viewer, teleop input, recording, and logging only.")

        if args.viewer:
            summary.update(
                run_live_viewer(
                    mujoco=mujoco,
                    model=model,
                    data=data,
                    adapter=adapter,
                    sandbox_config=sandbox_config,
                    command_logger=command_logger,
                    state_logger=state_logger,
                    recorder=recorder,
                    duration_s=args.duration,
                )
            )
        else:
            summary.update(
                run_headless(
                    mujoco=mujoco,
                    model=model,
                    data=data,
                    adapter=adapter,
                    sandbox_config=sandbox_config,
                    command_logger=command_logger,
                    state_logger=state_logger,
                    duration_s=args.duration,
                )
            )

        save_render(
            mujoco,
            model,
            data,
            artifacts.final_render,
            sandbox_config.render_width,
            sandbox_config.render_height,
        )
    except (ConfigError, MuJoCoImportError, MuJoCoModelError, LocomotionPolicyError, OSError, ValueError) as exc:
        summary["error"] = str(exc)
        summary["final_status"] = "error"
        print(f"G1 locomotion sandbox failed: {exc}", file=sys.stderr)
    finally:
        if recorder:
            recorder.stop()
            recorder.write_summary()
            summary["recording_used"] = recorder.used
            summary["recording_path"] = recorder.artifact_path()
            if recorder.summary_path:
                summary["recording_summary_path"] = str(recorder.summary_path)
        if command_logger:
            command_logger.close()
        else:
            ensure_empty_csv(artifacts.commands_csv)
        if state_logger:
            state_logger.close()
        else:
            ensure_empty_csv(artifacts.state_csv)
        summary["elapsed_wall_s"] = round(time.time() - started, 3)
        write_summary(artifacts.summary_json, summary)
        write_dashboard(artifacts.dashboard_html, summary)
        print_artifacts(summary)

    return 1 if summary.get("error") else 0


def run_live_viewer(
    mujoco,
    model,
    data,
    adapter,
    sandbox_config,
    command_logger,
    state_logger,
    recorder,
    duration_s: float,
) -> dict[str, object]:
    try:
        import mujoco.viewer
    except Exception as exc:
        raise LocomotionPolicyError(f"MuJoCo viewer is unavailable in this environment: {exc}") from exc

    teleop = TeleopController(sandbox_config)
    control_dt = 1.0 / sandbox_config.control_rate_hz
    sim_substeps = max(1, round(control_dt / float(model.opt.timestep)))
    end_time = data.time + duration_s if duration_s and duration_s > 0 else float("inf")
    result: dict[str, object] = {"viewer_opened": False, "final_status": "standing", "fallen": False}

    def on_key(key: int) -> None:
        label = teleop.apply_key(key)
        if label:
            print_status(data.time, teleop.command, "key", False, label)

    try:
        with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
            result["viewer_opened"] = True
            last_status_print = 0.0
            while viewer.is_running() and data.time < end_time and not teleop.quit_requested:
                step_start = time.time()
                command = teleop.command_with_timeout(step_start)
                start_recording, stop_recording = teleop.consume_recording_requests()
                if start_recording:
                    frames_dir = recorder.start(make_recording_timestamp())
                    print(f"recording_path: {frames_dir}")
                if stop_recording:
                    recorder.stop()
                    recorder.write_summary()

                state = base_state(data)
                status = determine_status(command, state, sandbox_config)
                if status == "fallen":
                    data.ctrl[:] = 0.0
                    result["fallen"] = True
                    teleop.command.vx = teleop.command.vy = teleop.command.yaw_rate = 0.0
                elif not getattr(adapter, "requires_substep_control", False):
                    adapter.step(model, data, command, control_dt)

                for _ in range(sim_substeps):
                    if status != "fallen" and getattr(adapter, "requires_substep_control", False):
                        adapter.step(model, data, command, float(model.opt.timestep))
                    mujoco.mj_step(model, data)
                viewer.sync()
                recorder.maybe_capture(
                    mujoco,
                    model,
                    data,
                    data.time,
                    sandbox_config.render_width,
                    sandbox_config.render_height,
                )
                command_logger.write(time.time(), data.time, teleop.last_key, command, recorder.active, status)
                state_logger.write(time.time(), data.time, state, status)
                result["final_status"] = status
                if time.time() - last_status_print > 0.5:
                    print_status(data.time, command, status, recorder.active, teleop.last_key)
                    last_status_print = time.time()
                elapsed = time.time() - step_start
                if elapsed < control_dt:
                    time.sleep(control_dt - elapsed)
    except Exception as exc:
        raise LocomotionPolicyError(f"Live MuJoCo viewer failed or closed unexpectedly: {exc}") from exc

    return result


def run_headless(
    mujoco,
    model,
    data,
    adapter,
    sandbox_config,
    command_logger,
    state_logger,
    duration_s: float,
) -> dict[str, object]:
    command = TeleopController(sandbox_config).command
    control_dt = 1.0 / sandbox_config.control_rate_hz
    sim_substeps = max(1, round(control_dt / float(model.opt.timestep)))
    steps = max(1, int(duration_s / control_dt))
    status = "standing"
    fallen = False
    for _ in range(steps):
        state = base_state(data)
        status = determine_status(command, state, sandbox_config)
        if status == "fallen":
            data.ctrl[:] = 0.0
            fallen = True
        elif not getattr(adapter, "requires_substep_control", False):
            adapter.step(model, data, command, control_dt)
        for _ in range(sim_substeps):
            if status != "fallen" and getattr(adapter, "requires_substep_control", False):
                adapter.step(model, data, command, float(model.opt.timestep))
            mujoco.mj_step(model, data)
        command_logger.write(time.time(), data.time, "", command, False, status)
        state_logger.write(time.time(), data.time, state, status)
    return {"viewer_opened": False, "final_status": status, "fallen": fallen}


def reset_to_keyframe(mujoco, model, data, keyframe_name: str | None) -> None:
    if not keyframe_name:
        return
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe_name)
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)


def print_controls(policy: str, adapter: str, real_locomotion: bool) -> None:
    print(f"policy: {policy}")
    print(f"adapter: {adapter}")
    print(f"real_locomotion: {'yes' if real_locomotion else 'no'}")
    print("controls: Up/W forward, Down/Z backward, Left/A yaw left, Right/D yaw right, Space/X stop")
    print("recording: R start, S or T stop; Q quit")
    if adapter == "lucky_walker":
        print("lucky_walker teleop: tap W/Up several times to ramp speed; X or Space stops immediately.")
    if adapter == "unitree_rl_gym_g1":
        print("unitree_rl_gym_g1 teleop: experimental regular-G1 bridge; tap W/Up to ramp speed, X or Space stops.")


def print_status(t_sim: float, command, status: str, recording: bool, key: str) -> None:
    print(
        f"t={t_sim:7.3f} vx={command.vx:+.3f} vy={command.vy:+.3f} "
        f"yaw_rate={command.yaw_rate:+.3f} recording={'yes' if recording else 'no'} "
        f"status={status} key={key}"
    )


def print_artifacts(summary: dict[str, object]) -> None:
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"dashboard_artifact: {summary.get('dashboard_path')}")
    print(f"summary_artifact: {summary.get('summary_path')}")
    print(f"command_log_artifact: {summary.get('command_log_path')}")
    print(f"final_render_artifact: {summary.get('final_render_path')}")
    print(f"compatibility_artifact: {summary.get('compatibility_report_path')}")


def ensure_empty_csv(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        path.write_text("", encoding="utf-8")


def apply_policy_teleop_defaults(adapter_name: str, sandbox_config) -> None:
    """Use policy-trained command ranges when a real backend needs them."""
    if adapter_name not in ("lucky_walker", "unitree_rl_gym_g1", "unitree_rl_gym_native"):
        return
    sandbox_config.max_forward_speed_mps = max(sandbox_config.max_forward_speed_mps, 1.0)
    sandbox_config.max_lateral_speed_mps = max(sandbox_config.max_lateral_speed_mps, 0.6)
    sandbox_config.max_yaw_rate_radps = max(sandbox_config.max_yaw_rate_radps, 1.0)
    sandbox_config.command_timeout_s = max(sandbox_config.command_timeout_s, 2.0)
    sandbox_config.command_step_fraction = max(sandbox_config.command_step_fraction, 0.2)


if __name__ == "__main__":
    code = main()
    if "--viewer" in sys.argv and code == 0:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
    raise SystemExit(code)
