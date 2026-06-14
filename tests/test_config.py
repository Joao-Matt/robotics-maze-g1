from pathlib import Path

import pytest

from sim.config import REQUIRED_TOP_LEVEL_SECTIONS, ConfigError, load_config


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_default_config_exists_and_loads():
    config = load_config(PROJECT_ROOT / "configs" / "default.yaml")

    assert config["project"]["name"] == "robotics-maze-g1"


def test_default_config_has_required_sections():
    config = load_config(PROJECT_ROOT / "configs" / "default.yaml")

    for section in REQUIRED_TOP_LEVEL_SECTIONS:
        assert section in config


def test_default_config_has_g1_model_path():
    config = load_config(PROJECT_ROOT / "configs" / "default.yaml")

    assert config["robot"]["model_xml_path"] == "assets/mujoco_menagerie/unitree_g1/scene.xml"
    assert config["robot"]["initial_keyframe"] == "stand"


def test_missing_config_raises_clear_error():
    with pytest.raises(ConfigError, match="does not exist"):
        load_config(PROJECT_ROOT / "configs" / "missing.yaml")
