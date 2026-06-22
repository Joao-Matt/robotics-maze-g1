"""Configuration helpers for the direct MuJoCo velocity-controller trainer."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


DEFAULT_RL_CONFIG = Path(__file__).resolve().parents[1] / "configs" / "rl_velocity_controller.yaml"


class RlVelocityConfigError(ValueError):
    """Raised when the RL velocity-controller config is malformed."""


def load_rl_config(path: str | Path = DEFAULT_RL_CONFIG) -> dict[str, Any]:
    """Load and validate the RL velocity-controller YAML config."""
    config_path = Path(path)
    if not config_path.exists():
        raise RlVelocityConfigError(f"RL config file does not exist: {config_path}")
    loaded = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise RlVelocityConfigError(f"RL config must be a YAML mapping: {config_path}")
    values = loaded.get("rl_velocity_controller")
    if not isinstance(values, dict):
        raise RlVelocityConfigError("RL config is missing top-level rl_velocity_controller mapping.")
    if not values.get("curriculum", {}).get("stages"):
        raise RlVelocityConfigError("RL config must define at least one curriculum stage.")
    return values


def deep_copy_config(config: dict[str, Any]) -> dict[str, Any]:
    """Return a mutable deep copy of a loaded config mapping."""
    return deepcopy(config)


def require_mapping(values: dict[str, Any], key: str) -> dict[str, Any]:
    item = values.get(key, {})
    if not isinstance(item, dict):
        raise RlVelocityConfigError(f"{key} must be a mapping.")
    return item

