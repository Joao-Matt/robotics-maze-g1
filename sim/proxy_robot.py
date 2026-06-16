"""MuJoCo proxy robot helpers for visual waypoint-follow inspection."""

from __future__ import annotations

from pathlib import Path
import math
import xml.etree.ElementTree as ET


PROXY_BODY_NAME = "proxy_waypoint_body"
PROXY_GEOM_NAME = "proxy_waypoint_body_geom"
PROXY_HEADING_GEOM_NAME = "proxy_waypoint_heading_geom"


def add_proxy_to_world_xml(
    source_xml: Path,
    output_xml: Path,
    *,
    start_xyz: tuple[float, float, float],
    path_waypoints: list[tuple[float, float, float]],
    marker_stride: int = 4,
) -> None:
    """Add a visible mocap-controlled proxy body and path markers to a world XML."""
    tree = ET.parse(source_xml)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")

    _append_path_markers(worldbody, path_waypoints, marker_stride=marker_stride)
    _append_proxy_body(worldbody, start_xyz)

    output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)


def set_proxy_pose(mujoco: object, model: object, data: object, pose: object, z_m: float = 0.28) -> None:
    """Set the proxy mocap body pose from a Pose2D-like object."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, PROXY_BODY_NAME)
    if body_id < 0:
        raise ValueError(f"Proxy body is missing from MuJoCo model: {PROXY_BODY_NAME}")

    mocap_id = int(model.body_mocapid[body_id])
    if mocap_id < 0:
        raise ValueError(f"Proxy body is not configured as a mocap body: {PROXY_BODY_NAME}")

    data.mocap_pos[mocap_id] = [float(pose.x), float(pose.y), float(z_m)]
    data.mocap_quat[mocap_id] = yaw_to_quat_wxyz(float(pose.yaw))


def yaw_to_quat_wxyz(yaw: float) -> list[float]:
    """Convert planar yaw to a MuJoCo wxyz quaternion."""
    half = yaw / 2.0
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def _append_proxy_body(worldbody: ET.Element, start_xyz: tuple[float, float, float]) -> None:
    x, y, _ = start_xyz
    body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": PROXY_BODY_NAME,
            "mocap": "true",
            "pos": f"{x:.6g} {y:.6g} 0.28",
        },
    )
    ET.SubElement(
        body,
        "geom",
        {
            "name": PROXY_GEOM_NAME,
            "type": "cylinder",
            "size": "0.24 0.18",
            "rgba": "1.0 0.45 0.05 1",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    ET.SubElement(
        body,
        "geom",
        {
            "name": PROXY_HEADING_GEOM_NAME,
            "type": "capsule",
            "fromto": "0 0 0.12 0.42 0 0.12",
            "size": "0.07",
            "rgba": "1.0 0.85 0.05 1",
            "contype": "0",
            "conaffinity": "0",
        },
    )


def _append_path_markers(
    worldbody: ET.Element,
    path_waypoints: list[tuple[float, float, float]],
    *,
    marker_stride: int,
) -> None:
    stride = max(1, int(marker_stride))
    for index, waypoint in enumerate(path_waypoints):
        if index % stride != 0 and index not in (0, len(path_waypoints) - 1):
            continue
        x, y, _ = waypoint
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": f"oracle_path_marker_{index}",
                "type": "sphere",
                "pos": f"{x:.6g} {y:.6g} 0.08",
                "size": "0.08",
                "rgba": "0.10 0.35 1.0 0.85",
                "contype": "0",
                "conaffinity": "0",
            },
        )
