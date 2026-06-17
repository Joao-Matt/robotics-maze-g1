from pathlib import Path
import time

from sim.locomotion_policy_adapter import PlaceholderPolicyAdapter, VelocityCommand
from sim.locomotion_sandbox import (
    ARROW_UP,
    LocomotionSandboxConfig,
    RecordingManager,
    TeleopController,
    clip_command,
    write_dashboard,
)


class FakeModel:
    nu = 29


def test_command_clipping_respects_config_limits():
    config = LocomotionSandboxConfig(
        max_forward_speed_mps=0.25,
        max_lateral_speed_mps=0.15,
        max_yaw_rate_radps=0.5,
    )

    clipped = clip_command(VelocityCommand(vx=1.0, vy=-1.0, yaw_rate=2.0), config)

    assert clipped.vx == 0.25
    assert clipped.vy == -0.15
    assert clipped.yaw_rate == 0.5


def test_key_mapping_updates_velocity_and_recording_requests():
    teleop = TeleopController(LocomotionSandboxConfig(max_forward_speed_mps=0.25, max_yaw_rate_radps=0.5))

    assert teleop.apply_key(ARROW_UP) == "UP"
    assert teleop.command.vx > 0
    teleop.apply_key("A")
    assert teleop.command.yaw_rate > 0
    teleop.apply_key("R")
    start, stop = teleop.consume_recording_requests()
    assert start is True
    assert stop is False
    teleop.apply_key("S")
    start, stop = teleop.consume_recording_requests()
    assert start is False
    assert stop is True
    teleop.apply_key("X")
    assert teleop.command == VelocityCommand()


def test_command_timeout_zeros_command():
    config = LocomotionSandboxConfig(command_timeout_s=0.01)
    teleop = TeleopController(config)
    teleop.apply_key("W", now=10.0)

    command = teleop.command_with_timeout(now=10.02)

    assert command == VelocityCommand()


def test_placeholder_adapter_reports_not_real_locomotion():
    report = PlaceholderPolicyAdapter().compatibility_report(FakeModel(), Path("g1.xml"))

    assert report.loaded is True
    assert report.real_locomotion is False
    assert report.compatible is True
    assert "No real walking policy loaded" in report.warnings[0]


def test_policy_compatibility_report_writer(tmp_path):
    path = tmp_path / "compatibility.json"
    report = PlaceholderPolicyAdapter().compatibility_report(FakeModel(), Path("g1.xml"))

    report.write_json(path)

    rendered = path.read_text(encoding="utf-8")
    assert '"real_locomotion": false' in rendered
    assert '"model_nu": 29' in rendered


def test_dashboard_references_required_artifacts(tmp_path):
    dashboard = tmp_path / "dashboard.html"
    summary = {
        "policy": "placeholder",
        "adapter": "placeholder",
        "model_xml_path": "assets/mujoco_menagerie/unitree_g1/scene.xml",
        "real_locomotion": False,
        "final_status": "standing",
        "fallen": False,
        "recording_used": False,
        "command_log_path": "runs/visual/g1_loco_latest_commands.csv",
        "final_render_path": "runs/visual/g1_loco_latest_final_render.png",
        "compatibility_report_path": "runs/visual/g1_loco_latest_policy_compatibility.json",
        "rerun_command": "make g1-loco-view POLICY=placeholder",
    }

    write_dashboard(dashboard, summary)

    rendered = dashboard.read_text(encoding="utf-8")
    assert "G1 Locomotion Policy Sandbox" in rendered
    assert "runs/visual/g1_loco_latest_commands.csv" in rendered
    assert "runs/visual/g1_loco_latest_policy_compatibility.json" in rendered
    assert "R = start recording" in rendered


def test_recording_path_creation(tmp_path):
    manager = RecordingManager(tmp_path, "g1_loco", fps=30)

    frames_dir = manager.start("20260616_120000")
    manager.stop()
    manager.write_summary()

    assert frames_dir == tmp_path / "g1_loco_20260616_120000_frames"
    assert frames_dir.is_dir()
    assert manager.summary_path is not None
    assert manager.summary_path.exists()
