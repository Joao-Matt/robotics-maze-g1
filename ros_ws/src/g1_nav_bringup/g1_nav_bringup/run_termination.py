"""Shared navigation run termination policy."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path


TERMINAL_RUN_STATUSES = {
    "GOAL_REACHED",
    "COLLISION_ABORT",
    "STUCK",
    "TIMEOUT",
}


def is_terminal_run_status(status: object) -> bool:
    return str(status or "") in TERMINAL_RUN_STATUSES


def is_run_success_status(status: object) -> bool:
    return str(status or "") == "GOAL_REACHED"


def termination_source(status: object) -> str:
    return "navigation_goal" if is_run_success_status(status) else "navigation_or_safety_status"


def parse_duration_s(value: object) -> float | None:
    if isinstance(value, (int, float)):
        duration = float(value)
        return duration if math.isfinite(duration) and duration > 0.0 else None
    if value is None:
        return None
    text = str(value).strip().lower()
    match = re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)\s*(s|sec|secs|second|seconds)?", text)
    if not match:
        return None
    duration = float(match.group(1))
    return duration if math.isfinite(duration) and duration > 0.0 else None


def run_duration_s(output_dir: Path, configured_duration: object, default_duration_s: float = 600.0) -> float:
    manifest = output_dir / "run_manifest.json"
    if manifest.is_file():
        try:
            values = json.loads(manifest.read_text(encoding="utf-8"))
            duration = parse_duration_s((values.get("parameters") or {}).get("duration"))
            if duration is not None:
                return duration
        except Exception:
            pass
    return parse_duration_s(configured_duration) or float(default_duration_s)
