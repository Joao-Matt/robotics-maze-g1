from pathlib import Path
import copy
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from scripts.run_d435i_visual_check import artifact_paths, normalize_depth, write_dashboard
from sim.config import load_config
from sim.d435i_sensor import D435iSpec, install_d435i
from sim.world_builder import build_maze_world


PROJECT_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def default_config():
    return load_config(PROJECT_ROOT / "configs" / "default.yaml")


def test_install_d435i_adds_fixed_visual_rgb_depth_and_imu(default_config, tmp_path):
    result = build_maze_world(default_config, seed=123, output_dir=tmp_path)
    root = ET.parse(result.model_xml_path).getroot()
    body = root.find(".//body[@name='d435i_link']")

    assert body is not None
    assert body.get("pos") == "0.08 0 0.34"
    assert body.find("joint") is None
    assert body.find("camera[@name='d435i_rgb']") is not None
    assert body.find("camera[@name='d435i_depth']") is not None
    assert body.find("site[@name='d435i_rgb_optical_frame']") is not None
    assert body.find("site[@name='d435i_depth_optical_frame']") is not None
    assert body.find("site[@name='d435i_imu_frame']") is not None
    assert root.find(".//gyro[@name='d435i_angular_velocity']") is not None
    assert root.find(".//accelerometer[@name='d435i_linear_acceleration']") is not None
    for geom in body.findall("geom"):
        assert geom.get("contype") == "0"
        assert geom.get("conaffinity") == "0"


def test_d435i_does_not_change_model_joint_dimensions(default_config, tmp_path):
    mujoco = pytest.importorskip("mujoco")
    enabled = build_maze_world(default_config, seed=123, output_dir=tmp_path / "enabled")
    disabled_config = copy.deepcopy(default_config)
    disabled_config["d435i"]["enabled"] = False
    disabled = build_maze_world(disabled_config, seed=123, output_dir=tmp_path / "disabled")

    enabled_model = mujoco.MjModel.from_xml_path(enabled.model_xml_path)
    disabled_model = mujoco.MjModel.from_xml_path(disabled.model_xml_path)

    assert (enabled_model.nq, enabled_model.nv, enabled_model.nu) == (
        disabled_model.nq,
        disabled_model.nv,
        disabled_model.nu,
    )
    assert enabled_model.ncam == disabled_model.ncam + 2


@pytest.mark.parametrize(
    ("key", "value", "message"),
    [
        ("width", 0, "width and height"),
        ("rgb_fovy_deg", 180, "FOV"),
        ("depth_visual_max_m", 0.1, "depth visualization range"),
    ],
)
def test_invalid_d435i_config_fails_clearly(default_config, key, value, message):
    config = copy.deepcopy(default_config)
    config["d435i"][key] = value

    with pytest.raises(ValueError, match=message):
        D435iSpec.from_config(config)


def test_missing_parent_body_fails_clearly(default_config):
    config = copy.deepcopy(default_config)
    config["d435i"]["parent_body"] = "missing_torso"
    tree = ET.parse(PROJECT_ROOT / "third_party" / "g1-manipulation-challenge" / "g1.xml")

    with pytest.raises(ValueError, match="parent body does not exist"):
        install_d435i(tree, config)


def test_depth_normalization_and_dashboard_artifacts(tmp_path):
    depth = np.array([[0.2, 1.0], [2.0, 4.0]], dtype=np.float32)
    pixels, stats = normalize_depth(depth, 0.15, 8.0)
    paths = artifact_paths(tmp_path, 123)
    summary = {"status": "completed", "depth": stats}
    for key in ("mount_image", "rgb_image", "depth_image"):
        paths[key].write_bytes(b"\x89PNG\r\n\x1a\n")
    write_dashboard(paths["dashboard"], summary, paths)
    paths["summary"].write_text(json.dumps(summary), encoding="utf-8")

    assert pixels.shape == (2, 2, 3)
    assert stats["valid_pixel_count"] == 4
    rendered = paths["dashboard"].read_text(encoding="utf-8")
    for key in ("mount_image", "rgb_image", "depth_image"):
        assert paths[key].name in rendered
