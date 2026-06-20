"""Publish the final SLAM map-to-odom transform for replay evaluation."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class SavedMapTransform(Node):
    def __init__(self) -> None:
        super().__init__("saved_map_transform")
        self.declare_parameter("transform_path", "")
        path = Path(str(self.get_parameter("transform_path").value))
        values = json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {
            "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.child_frame_id = "odom"
        for field in ("x", "y", "z"):
            setattr(message.transform.translation, field, float(values["translation"][field]))
            setattr(message.transform.rotation, field, float(values["rotation"][field]))
        message.transform.rotation.w = float(values["rotation"]["w"])
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(message)


def main() -> int:
    rclpy.init()
    node = SavedMapTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
