from pathlib import Path
import math
import xml.etree.ElementTree as ET

import pytest

from sim.proxy_robot import PROXY_BODY_NAME, add_proxy_to_world_xml, yaw_to_quat_wxyz


def test_add_proxy_to_world_xml_adds_mocap_body_and_path_markers(tmp_path):
    source = tmp_path / "world.xml"
    output = tmp_path / "sim_follow_world.xml"
    source.write_text("<mujoco><worldbody /></mujoco>\n", encoding="utf-8")

    add_proxy_to_world_xml(
        source,
        output,
        start_xyz=(1.0, 2.0, 0.0),
        path_waypoints=[(1.0, 2.0, 0.0), (2.0, 2.0, 0.0), (3.0, 2.0, 0.0)],
        marker_stride=1,
    )

    root = ET.parse(output).getroot()
    body = root.find(f".//body[@name='{PROXY_BODY_NAME}']")
    markers = root.findall(".//geom")

    assert body is not None
    assert body.get("mocap") == "true"
    assert body.get("pos") == "1 2 0.28"
    assert any((geom.get("name") or "").startswith("oracle_path_marker_") for geom in markers)


def test_yaw_to_quat_wxyz():
    quat = yaw_to_quat_wxyz(math.pi / 2.0)

    assert quat[0] == pytest.approx(math.sqrt(0.5))
    assert quat[1] == 0.0
    assert quat[2] == 0.0
    assert quat[3] == pytest.approx(math.sqrt(0.5))
