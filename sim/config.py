"""Configuration loading helpers for the project."""

from pathlib import Path
from typing import Any, Dict

import yaml

REQUIRED_TOP_LEVEL_SECTIONS = ("project", "sim", "maze", "robot", "logging")


class ConfigError(ValueError):
    """Raised when a configuration file is missing or malformed."""


def load_config(path: str | Path) -> Dict[str, Any]:
    """Load a YAML config file and validate the required top-level sections."""
    config_path = Path(path)
    if not config_path.exists():
        raise ConfigError(f"Config file does not exist: {config_path}")

    with config_path.open("r", encoding="utf-8") as config_file:
        loaded = yaml.safe_load(config_file)

    if not isinstance(loaded, dict):
        raise ConfigError(f"Config must be a YAML mapping: {config_path}")

    missing = [section for section in REQUIRED_TOP_LEVEL_SECTIONS if section not in loaded]
    if missing:
        missing_text = ", ".join(missing)
        raise ConfigError(f"Config is missing required section(s): {missing_text}")

    return loaded
