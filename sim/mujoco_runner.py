"""Shared MuJoCo helpers for the production simulator runtime."""

from __future__ import annotations

from pathlib import Path
from typing import Any
import struct
import zlib

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class MuJoCoRuntimeError(RuntimeError):
    """Raised when MuJoCo bring-up cannot continue."""


class MuJoCoImportError(MuJoCoRuntimeError):
    """Raised when the MuJoCo Python package is not installed."""


class MuJoCoModelError(MuJoCoRuntimeError):
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
