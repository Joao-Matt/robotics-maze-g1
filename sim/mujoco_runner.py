"""MuJoCo runner for Milestone 1 simulator bring-up."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict
import math
import struct
import time
import zlib

from sim.robot_interface import RobotInterface

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class MuJoCoRunnerError(RuntimeError):
    """Raised when MuJoCo bring-up cannot continue."""


class MuJoCoImportError(MuJoCoRunnerError):
    """Raised when the MuJoCo Python package is not installed."""


class MuJoCoModelError(MuJoCoRunnerError):
    """Raised when the configured MuJoCo model cannot be loaded."""


def resolve_project_path(path_value: str | Path, project_root: Path = PROJECT_ROOT) -> Path:
    """Resolve a config path relative to the repository root."""
    path = Path(path_value).expanduser()
    if path.is_absolute():
        return path
    return project_root / path


def import_mujoco() -> Any:
    """Import MuJoCo lazily so non-simulator tests remain import-safe."""
    try:
        import mujoco
    except ModuleNotFoundError as exc:
        raise MuJoCoImportError(
            "MuJoCo is not installed. Run `make setup` from the project root "
            "and confirm `mujoco==3.9.0` is installed in `.venv`."
        ) from exc
    return mujoco


class MuJoCoRunner:
    """Load a configured model and step MuJoCo for a fixed duration."""

    def __init__(self, config: Dict[str, Any], project_root: Path = PROJECT_ROOT) -> None:
        self.config = config
        self.project_root = project_root

    @property
    def model_path(self) -> Path:
        try:
            raw_path = self.config["robot"]["model_xml_path"]
        except KeyError as exc:
            raise MuJoCoModelError(
                "Config is missing `robot.model_xml_path`; expected a MuJoCo XML path."
            ) from exc
        return resolve_project_path(raw_path, self.project_root)

    def run(
        self,
        duration_s: float | None = None,
        viewer: bool = False,
        render_path: Path | None = None,
        render_width: int = 640,
        render_height: int = 480,
    ) -> Dict[str, Any]:
        """Run the configured model and return a state summary."""
        model_path = self.model_path
        if not model_path.exists():
            raise MuJoCoModelError(
                f"Configured model XML does not exist: {model_path}. "
                "Run `git submodule update --init --recursive` or update "
                "`robot.model_xml_path` in the config."
            )

        mujoco = import_mujoco()
        duration = self._duration(duration_s)

        try:
            model = mujoco.MjModel.from_xml_path(str(model_path))
        except Exception as exc:  # MuJoCo raises several exception types here.
            raise MuJoCoModelError(f"Failed to load MuJoCo model from {model_path}: {exc}") from exc

        timestep = float(self.config["sim"].get("timestep", model.opt.timestep))
        if timestep <= 0:
            raise MuJoCoModelError(f"Invalid sim.timestep: {timestep}")
        model.opt.timestep = timestep

        data = mujoco.MjData(model)
        keyframe_name = self.config.get("robot", {}).get("initial_keyframe")
        reset_keyframe = self._reset_to_keyframe(mujoco, model, data, keyframe_name)

        interface = RobotInterface(mujoco, model, data)
        steps = max(1, math.ceil(duration / timestep))

        if viewer:
            self._run_with_viewer(mujoco, model, data, steps, timestep)
        else:
            for _ in range(steps):
                mujoco.mj_step(model, data)

        state = interface.get_state()
        if render_path:
            try:
                self._save_render(mujoco, model, data, render_path, render_width, render_height)
                state["render_path"] = str(render_path)
            except Exception as exc:
                state["render_path"] = None
                state["render_error"] = str(exc)

        state.update(
            {
                "status": "completed",
                "model_xml_path": str(model_path),
                "requested_duration_s": float(duration),
                "steps": int(steps),
                "timestep_s": float(timestep),
                "initial_keyframe": reset_keyframe,
            }
        )
        return state

    def _duration(self, duration_s: float | None) -> float:
        duration = duration_s
        if duration is None:
            duration = self.config.get("sim", {}).get("default_duration_s", 3.0)

        duration = float(duration)
        if duration <= 0:
            raise MuJoCoRunnerError(f"Duration must be positive, got {duration}")
        return duration

    def _reset_to_keyframe(self, mujoco: Any, model: Any, data: Any, keyframe_name: str | None) -> str | None:
        if not keyframe_name:
            return None

        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe_name)
        if key_id < 0:
            return None

        mujoco.mj_resetDataKeyframe(model, data, key_id)
        return keyframe_name

    def _run_with_viewer(self, mujoco: Any, model: Any, data: Any, steps: int, timestep: float) -> None:
        try:
            import mujoco.viewer
        except Exception as exc:
            raise MuJoCoRunnerError(
                f"MuJoCo viewer is unavailable in this environment: {exc}"
            ) from exc

        with mujoco.viewer.launch_passive(model, data) as viewer:
            for _ in range(steps):
                if not viewer.is_running():
                    break
                step_start = time.time()
                mujoco.mj_step(model, data)
                viewer.sync()
                elapsed = time.time() - step_start
                if elapsed < timestep:
                    time.sleep(timestep - elapsed)

    def _save_render(
        self,
        mujoco: Any,
        model: Any,
        data: Any,
        path: Path,
        width: int,
        height: int,
    ) -> None:
        """Save a dependency-free final RGB frame as PNG or binary PPM."""
        if width <= 0 or height <= 0:
            raise MuJoCoRunnerError(f"Render dimensions must be positive, got {width}x{height}")

        path.parent.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(model, width=width, height=height)
        try:
            renderer.update_scene(data)
            pixels = renderer.render()
        finally:
            close = getattr(renderer, "close", None)
            if close:
                close()

        if path.suffix.lower() == ".png":
            _write_png(path, pixels)
        else:
            with path.open("wb") as file:
                file.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
                file.write(pixels[:, :, :3].tobytes())


def _write_png(path: Path, pixels: Any) -> None:
    """Write RGB pixels as a simple no-dependency PNG."""
    height, width = pixels.shape[:2]
    rgb = pixels[:, :, :3]
    raw_rows = b"".join(b"\x00" + rgb[row].tobytes() for row in range(height))

    def chunk(kind: bytes, data: bytes) -> bytes:
        payload = kind + data
        return (
            struct.pack(">I", len(data))
            + payload
            + struct.pack(">I", zlib.crc32(payload) & 0xFFFFFFFF)
        )

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk("IHDR".encode("ascii"), struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk("IDAT".encode("ascii"), zlib.compress(raw_rows))
    png += chunk("IEND".encode("ascii"), b"")
    path.write_bytes(png)
