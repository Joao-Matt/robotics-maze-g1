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

    assert config["robot"]["model_xml_path"] == "third_party/g1-manipulation-challenge/scene.xml"
    assert config["robot"]["base_model_xml_path"] == "third_party/g1-manipulation-challenge/g1.xml"
    assert config["robot"]["legacy_menagerie_model_xml_path"] == "assets/mujoco_menagerie/unitree_g1/scene.xml"
    assert config["robot"]["initial_keyframe"] is None


def test_default_oracle_speed_is_visible_for_lucky_walker():
    config = load_config(PROJECT_ROOT / "configs" / "default.yaml")

    assert config["oracle"]["forward_speed_mps"] >= 0.8
    assert config["oracle"]["arc_turn_speed_mps"] == 0.8
    assert config["oracle"]["arc_turn_forward_speed_mps"] == 0.4
    assert config["oracle"]["arc_turn_yaw_rate_radps"] == 0.8
    assert config["oracle"]["turn_start_distance_m"] == 0.8
    assert config["oracle"]["max_recovery_attempts"] == 2
    assert config["oracle"]["turn_penalty_cost"] == 2.0
    assert config["oracle"]["waypoint_tolerance_m"] == 0.75
    assert config["oracle"]["approach_tolerance_m"] == 0.35


def test_default_corridor_width_is_lucky_walker_friendly():
    config = load_config(PROJECT_ROOT / "configs" / "default.yaml")

    assert config["maze"]["cell_size_m"] == 1.6


def test_wide_maze_config_is_roomier_than_default():
    default_config = load_config(PROJECT_ROOT / "configs" / "default.yaml")
    wide_config = load_config(PROJECT_ROOT / "configs" / "lucky_wide_maze.yaml")

    assert wide_config["maze"]["cell_size_m"] > default_config["maze"]["cell_size_m"]
    assert wide_config["maze"]["cell_size_m"] == 2.0
    assert wide_config["robot"]["base_model_xml_path"] == default_config["robot"]["base_model_xml_path"]


def test_missing_config_raises_clear_error():
    with pytest.raises(ConfigError, match="does not exist"):
        load_config(PROJECT_ROOT / "configs" / "missing.yaml")
