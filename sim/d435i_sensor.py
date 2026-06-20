"""MuJoCo XML support for the simulated D435i-style sensor assembly."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import math
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class D435iSpec:
    parent_body: str
    link_name: str
    rgb_camera_name: str
    depth_camera_name: str
    rgb_optical_frame: str
    depth_optical_frame: str
    imu_frame: str
    mount_pos_m: tuple[float, float, float]
    pitch_deg: float
    width: int
    height: int
    rgb_fovy_deg: float
    depth_fovy_deg: float
    depth_visual_min_m: float
    depth_visual_max_m: float

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "D435iSpec | None":
        raw = config.get("d435i")
        if raw is None or raw.get("enabled", True) is False:
            return None
        if not isinstance(raw, dict):
            raise ValueError("d435i configuration must be a mapping")

        position = raw.get("mount_pos_m", [0.08, 0.0, 0.34])
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            raise ValueError("d435i.mount_pos_m must contain exactly three numbers")

        spec = cls(
            parent_body=str(raw.get("parent_body", "torso_link_rev_1_0")),
            link_name=str(raw.get("link_name", "d435i_link")),
            rgb_camera_name=str(raw.get("rgb_camera_name", "d435i_rgb")),
            depth_camera_name=str(raw.get("depth_camera_name", "d435i_depth")),
            rgb_optical_frame=str(raw.get("rgb_optical_frame", "d435i_rgb_optical_frame")),
            depth_optical_frame=str(raw.get("depth_optical_frame", "d435i_depth_optical_frame")),
            imu_frame=str(raw.get("imu_frame", "d435i_imu_frame")),
            mount_pos_m=tuple(float(value) for value in position),
            pitch_deg=float(raw.get("pitch_deg", 10.0)),
            width=int(raw.get("width", 640)),
            height=int(raw.get("height", 480)),
            rgb_fovy_deg=float(raw.get("rgb_fovy_deg", 42.0)),
            depth_fovy_deg=float(raw.get("depth_fovy_deg", 58.0)),
            depth_visual_min_m=float(raw.get("depth_visual_min_m", 0.15)),
            depth_visual_max_m=float(raw.get("depth_visual_max_m", 8.0)),
        )
        spec.validate()
        return spec

    def validate(self) -> None:
        named = {
            self.parent_body,
            self.link_name,
            self.rgb_camera_name,
            self.depth_camera_name,
            self.rgb_optical_frame,
            self.depth_optical_frame,
            self.imu_frame,
        }
        if "" in named:
            raise ValueError("d435i frame and body names must not be empty")
        if len(named) != 7:
            raise ValueError("d435i frame, camera, link, and parent names must be unique")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("d435i width and height must be positive")
        if not 0.0 < self.pitch_deg < 90.0:
            raise ValueError("d435i.pitch_deg must be between 0 and 90 degrees")
        if not 0.0 < self.rgb_fovy_deg < 180.0 or not 0.0 < self.depth_fovy_deg < 180.0:
            raise ValueError("d435i camera FOV values must be between 0 and 180 degrees")
        if self.depth_visual_min_m < 0.0 or self.depth_visual_max_m <= self.depth_visual_min_m:
            raise ValueError("d435i depth visualization range must have 0 <= min < max")

    def camera_metadata(self, name: str) -> dict[str, float | int | str]:
        fovy = self.rgb_fovy_deg if name == self.rgb_camera_name else self.depth_fovy_deg
        fy = 0.5 * self.height / math.tan(math.radians(fovy) / 2.0)
        fovx = math.degrees(2.0 * math.atan(0.5 * self.width / fy))
        return {
            "name": name,
            "width": self.width,
            "height": self.height,
            "fovx_deg": fovx,
            "fovy_deg": fovy,
            "fx_px": fy,
            "fy_px": fy,
            "cx_px": (self.width - 1) / 2.0,
            "cy_px": (self.height - 1) / 2.0,
        }


def install_d435i(tree: ET.ElementTree, config: dict[str, Any]) -> D435iSpec | None:
    """Attach a fixed, visible RGB-D/IMU assembly to the configured G1 body."""
    spec = D435iSpec.from_config(config)
    if spec is None:
        return None

    root = tree.getroot()
    parent = root.find(f".//body[@name='{spec.parent_body}']")
    if parent is None:
        raise ValueError(f"D435i parent body does not exist: {spec.parent_body}")

    for tag, name in (
        ("body", spec.link_name),
        ("camera", spec.rgb_camera_name),
        ("camera", spec.depth_camera_name),
        ("site", spec.rgb_optical_frame),
        ("site", spec.depth_optical_frame),
        ("site", spec.imu_frame),
    ):
        if root.find(f".//{tag}[@name='{name}']") is not None:
            raise ValueError(f"D435i {tag} name already exists: {name}")

    pitch = math.radians(spec.pitch_deg)
    xyaxes = f"0 -1 0 {math.sin(pitch):.8g} 0 {math.cos(pitch):.8g}"
    position = " ".join(f"{value:.8g}" for value in spec.mount_pos_m)
    body = ET.SubElement(parent, "body", {"name": spec.link_name, "pos": position})

    common_geom = {"contype": "0", "conaffinity": "0", "group": "1"}
    ET.SubElement(body, "geom", {
        "name": "d435i_housing", "type": "box", "pos": "0 0 0", "size": "0.025 0.09 0.025",
        "rgba": "0.12 0.14 0.16 1", **common_geom,
    })
    ET.SubElement(body, "geom", {
        "name": "d435i_rgb_lens", "type": "cylinder", "pos": "0.027 -0.027 0",
        "quat": "0.70710678 0 0.70710678 0", "size": "0.012 0.004", "rgba": "0.10 0.35 0.75 1",
        **common_geom,
    })
    ET.SubElement(body, "geom", {
        "name": "d435i_depth_lens", "type": "cylinder", "pos": "0.027 0.027 0",
        "quat": "0.70710678 0 0.70710678 0", "size": "0.012 0.004", "rgba": "0.08 0.08 0.10 1",
        **common_geom,
    })

    camera_pos = "0.032 0 0"
    ET.SubElement(body, "camera", {
        "name": spec.rgb_camera_name, "pos": camera_pos, "xyaxes": xyaxes, "fovy": f"{spec.rgb_fovy_deg:.8g}",
    })
    ET.SubElement(body, "camera", {
        "name": spec.depth_camera_name, "pos": camera_pos, "xyaxes": xyaxes, "fovy": f"{spec.depth_fovy_deg:.8g}",
    })
    site_attrs = {"type": "sphere", "size": "0.003", "rgba": "0 0 0 0"}
    ET.SubElement(body, "site", {"name": spec.rgb_optical_frame, "pos": camera_pos, **site_attrs})
    ET.SubElement(body, "site", {"name": spec.depth_optical_frame, "pos": camera_pos, **site_attrs})
    ET.SubElement(body, "site", {"name": spec.imu_frame, "pos": "0 0 0", **site_attrs})

    sensors = root.find("sensor")
    if sensors is None:
        sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", {"name": "d435i_angular_velocity", "site": spec.imu_frame})
    ET.SubElement(sensors, "accelerometer", {"name": "d435i_linear_acceleration", "site": spec.imu_frame})
    return spec
