"""Visual G1 locomotion sandbox helpers."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from html import escape
from pathlib import Path
from typing import Any
import csv
import json
import math
import time

from sim.locomotion_policy_adapter import VelocityCommand
from sim.mujoco_runner import _write_png


ARROW_UP = 265
ARROW_DOWN = 264
ARROW_LEFT = 263
ARROW_RIGHT = 262


@dataclass
class LocomotionSandboxConfig:
    max_forward_speed_mps: float = 0.25
    max_lateral_speed_mps: float = 0.15
    max_yaw_rate_radps: float = 0.5
    command_timeout_s: float = 0.5
    control_rate_hz: float = 50.0
    render_width: int = 640
    render_height: int = 480
    fall_min_base_height_m: float = 0.45
    fall_max_tilt_rad: float = 0.9
    recording_fps: int = 30
    command_step_fraction: float = 0.2


@dataclass
class SandboxArtifacts:
    output_dir: Path
    dashboard_html: Path
    summary_json: Path
    commands_csv: Path
    state_csv: Path
    final_render: Path
    compatibility_json: Path

    @classmethod
    def latest(cls, output_dir: Path) -> "SandboxArtifacts":
        return cls(
            output_dir=output_dir,
            dashboard_html=output_dir / "g1_loco_latest_dashboard.html",
            summary_json=output_dir / "g1_loco_latest_summary.json",
            commands_csv=output_dir / "g1_loco_latest_commands.csv",
            state_csv=output_dir / "g1_loco_latest_state.csv",
            final_render=output_dir / "g1_loco_latest_final_render.png",
            compatibility_json=output_dir / "g1_loco_latest_policy_compatibility.json",
        )


def config_from_dict(config: dict[str, Any]) -> LocomotionSandboxConfig:
    values = dict(config.get("locomotion_sandbox", {}))
    return LocomotionSandboxConfig(**{k: v for k, v in values.items() if k in LocomotionSandboxConfig.__annotations__})


def clip_command(command: VelocityCommand, config: LocomotionSandboxConfig) -> VelocityCommand:
    return VelocityCommand(
        vx=_clip(command.vx, -config.max_forward_speed_mps, config.max_forward_speed_mps),
        vy=_clip(command.vy, -config.max_lateral_speed_mps, config.max_lateral_speed_mps),
        yaw_rate=_clip(command.yaw_rate, -config.max_yaw_rate_radps, config.max_yaw_rate_radps),
    )


class TeleopController:
    """Stateful keyboard teleop mapping for desired velocity commands."""

    def __init__(self, config: LocomotionSandboxConfig) -> None:
        self.config = config
        self.command = VelocityCommand()
        self.last_key = ""
        self.last_input_wall_time = time.time()
        self.quit_requested = False
        self.start_recording_requested = False
        self.stop_recording_requested = False

    def apply_key(self, key: int | str, now: float | None = None) -> str:
        now = time.time() if now is None else now
        label = key_label(key)
        self.last_key = label
        self.last_input_wall_time = now
        step_vx = self.config.max_forward_speed_mps * self.config.command_step_fraction
        step_yaw = self.config.max_yaw_rate_radps * self.config.command_step_fraction

        normalized = normalize_key(key)
        if normalized in ("UP", "W"):
            self.command.vx += step_vx
        elif normalized in ("DOWN", "Z"):
            self.command.vx -= step_vx
        elif normalized in ("LEFT", "A"):
            self.command.yaw_rate += step_yaw
        elif normalized in ("RIGHT", "D"):
            self.command.yaw_rate -= step_yaw
        elif normalized in ("SPACE", "X"):
            self.command = VelocityCommand()
        elif normalized == "Q":
            self.quit_requested = True
        elif normalized == "R":
            self.start_recording_requested = True
        elif normalized in ("S", "T"):
            self.stop_recording_requested = True

        self.command = clip_command(self.command, self.config)
        return label

    def command_with_timeout(self, now: float | None = None) -> VelocityCommand:
        now = time.time() if now is None else now
        if now - self.last_input_wall_time > self.config.command_timeout_s:
            self.command = VelocityCommand()
        return self.command

    def consume_recording_requests(self) -> tuple[bool, bool]:
        start = self.start_recording_requested
        stop = self.stop_recording_requested
        self.start_recording_requested = False
        self.stop_recording_requested = False
        return start, stop


class CommandLogger:
    """CSV logger for teleop commands."""

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.file = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=("t_wall", "t_sim", "key", "vx", "vy", "yaw_rate", "recording", "status"),
        )
        self.writer.writeheader()

    def write(self, t_wall: float, t_sim: float, key: str, command: VelocityCommand, recording: bool, status: str) -> None:
        self.writer.writerow(
            {
                "t_wall": f"{t_wall:.6f}",
                "t_sim": f"{t_sim:.6f}",
                "key": key,
                "vx": f"{command.vx:.6f}",
                "vy": f"{command.vy:.6f}",
                "yaw_rate": f"{command.yaw_rate:.6f}",
                "recording": "yes" if recording else "no",
                "status": status,
            }
        )
        self.file.flush()

    def close(self) -> None:
        self.file.close()


class StateLogger:
    """CSV logger for a compact MuJoCo base state."""

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.file = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=("t_wall", "t_sim", "base_x", "base_y", "base_z", "roll", "pitch", "yaw", "status"),
        )
        self.writer.writeheader()

    def write(self, t_wall: float, t_sim: float, state: dict[str, float], status: str) -> None:
        self.writer.writerow(
            {
                "t_wall": f"{t_wall:.6f}",
                "t_sim": f"{t_sim:.6f}",
                "base_x": f"{state['base_x']:.6f}",
                "base_y": f"{state['base_y']:.6f}",
                "base_z": f"{state['base_z']:.6f}",
                "roll": f"{state['roll']:.6f}",
                "pitch": f"{state['pitch']:.6f}",
                "yaw": f"{state['yaw']:.6f}",
                "status": status,
            }
        )
        self.file.flush()

    def close(self) -> None:
        self.file.close()


class RecordingManager:
    """Frame recording helper that falls back to PNG frames without ffmpeg."""

    def __init__(self, output_dir: Path, prefix: str, fps: int) -> None:
        self.output_dir = output_dir
        self.prefix = prefix
        self.fps = fps
        self.active = False
        self.used = False
        self.frames_dir: Path | None = None
        self.summary_path: Path | None = None
        self.video_path: Path | None = None
        self.frame_count = 0
        self.last_capture_sim_time = -1.0
        self.started_wall_time: float | None = None
        self.stopped_wall_time: float | None = None
        self.message = "Recording was not started."

    def start(self, timestamp: str) -> Path:
        if self.active:
            return self.frames_dir or self.output_dir
        self.used = True
        self.active = True
        self.frame_count = 0
        self.last_capture_sim_time = -1.0
        self.started_wall_time = time.time()
        self.stopped_wall_time = None
        self.frames_dir = self.output_dir / f"{self.prefix}_{timestamp}_frames"
        self.summary_path = self.output_dir / f"{self.prefix}_{timestamp}_recording_summary.json"
        self.video_path = self.output_dir / f"{self.prefix}_{timestamp}_video.mp4"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.message = "Recording frames; MP4 export requires optional ffmpeg/imageio support."
        return self.frames_dir

    def maybe_capture(self, mujoco: Any, model: Any, data: Any, sim_time: float, width: int, height: int) -> None:
        if not self.active or self.frames_dir is None:
            return
        if self.last_capture_sim_time >= 0 and sim_time - self.last_capture_sim_time < 1.0 / max(1, self.fps):
            return
        frame_path = self.frames_dir / f"frame_{self.frame_count:06d}.png"
        save_render(mujoco, model, data, frame_path, width, height)
        self.frame_count += 1
        self.last_capture_sim_time = sim_time

    def stop(self) -> None:
        if self.active:
            self.active = False
            self.stopped_wall_time = time.time()
            self.message = "Saved PNG frames. MP4 was not created because ffmpeg/imageio is not a project dependency."

    def artifact_path(self) -> str | None:
        if self.frame_count > 0 and self.frames_dir:
            return str(self.frames_dir)
        return None

    def write_summary(self) -> None:
        if not self.summary_path:
            return
        summary = {
            "recording_used": self.used,
            "active_at_shutdown": self.active,
            "frames_dir": str(self.frames_dir) if self.frames_dir else None,
            "video_path": None,
            "frame_count": self.frame_count,
            "fps": self.fps,
            "started_wall_time": self.started_wall_time,
            "stopped_wall_time": self.stopped_wall_time,
            "message": self.message,
        }
        self.summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def make_recording_timestamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def base_state(data: Any) -> dict[str, float]:
    qpos = data.qpos
    roll, pitch, yaw = quat_to_euler(float(qpos[3]), float(qpos[4]), float(qpos[5]), float(qpos[6]))
    return {
        "base_x": float(qpos[0]),
        "base_y": float(qpos[1]),
        "base_z": float(qpos[2]),
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
    }


def determine_status(command: VelocityCommand, state: dict[str, float], config: LocomotionSandboxConfig) -> str:
    if state["base_z"] < config.fall_min_base_height_m:
        return "fallen"
    if abs(state["roll"]) > config.fall_max_tilt_rad or abs(state["pitch"]) > config.fall_max_tilt_rad:
        return "fallen"
    if abs(command.vx) > 1e-4 or abs(command.vy) > 1e-4 or abs(command.yaw_rate) > 1e-4:
        return "walking"
    return "standing"


def save_render(mujoco: Any, model: Any, data: Any, path: Path, width: int, height: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    renderer = mujoco.Renderer(model, width=width, height=height)
    try:
        renderer.update_scene(data)
        pixels = renderer.render()
    finally:
        close = getattr(renderer, "close", None)
        if close:
            close()
    _write_png(path, pixels)


def write_summary(path: Path, summary: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_dashboard(path: Path, summary: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    controls = [
        "Up arrow/W = increase forward velocity",
        "Down arrow/Z = decrease forward velocity",
        "Left arrow/A = yaw left",
        "Right arrow/D = yaw right",
        "Space/X = zero command",
        "Q = quit safely",
        "R = start recording",
        "S or T = stop recording",
    ]
    rows = "".join(
        f"<tr><th>{escape(str(key))}</th><td>{escape(str(value))}</td></tr>\n"
        for key, value in summary.items()
        if not isinstance(value, (dict, list))
    )
    controls_html = "".join(f"<li>{escape(item)}</li>" for item in controls)
    final_render = summary.get("final_render_path")
    image_html = ""
    if final_render:
        image_html = f'<img src="{escape(Path(final_render).name)}" alt="G1 locomotion final render">'
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>G1 Locomotion Policy Sandbox</title>
  <style>
    body {{ margin: 0; font-family: Arial, sans-serif; background: #f6f7f9; color: #1d2733; }}
    main {{ max-width: 1120px; margin: 0 auto; padding: 28px; }}
    h1 {{ font-size: 28px; margin: 0 0 16px; }}
    h2 {{ font-size: 18px; margin: 24px 0 10px; }}
    table {{ border-collapse: collapse; width: 100%; background: white; border: 1px solid #d8dee8; }}
    th, td {{ text-align: left; padding: 9px 11px; border-bottom: 1px solid #e5e9f0; vertical-align: top; }}
    th {{ width: 280px; background: #eef2f6; }}
    ul {{ background: white; border: 1px solid #d8dee8; padding: 14px 24px; }}
    img {{ display: block; max-width: 100%; border: 1px solid #d8dee8; background: white; }}
    .notice {{ padding: 12px 14px; background: #fff7d7; border: 1px solid #ebd27c; margin: 12px 0; }}
  </style>
</head>
<body>
<main>
  <h1>G1 Locomotion Policy Sandbox</h1>
  <div class="notice">This sandbox is separate from maze navigation. Placeholder mode does not claim real walking.</div>
  <h2>Run Summary</h2>
  <table>{rows}</table>
  <h2>Teleop Controls</h2>
  <ul>{controls_html}</ul>
  <h2>Final Render</h2>
  {image_html}
</main>
</body>
</html>
"""
    path.write_text(html, encoding="utf-8")


def key_label(key: int | str) -> str:
    if isinstance(key, str):
        return key
    mapping = {
        ARROW_UP: "UP",
        ARROW_DOWN: "DOWN",
        ARROW_LEFT: "LEFT",
        ARROW_RIGHT: "RIGHT",
        32: "SPACE",
    }
    return mapping.get(key, chr(key).upper() if 0 <= key < 128 else str(key))


def normalize_key(key: int | str) -> str:
    label = key_label(key).upper()
    return {" ": "SPACE"}.get(label, label)


def quat_to_euler(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))
