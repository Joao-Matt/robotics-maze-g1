"""Convert the voxel-filtered D435i PointCloud2 into a SLAM LaserScan."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2


def points_to_ranges(points: np.ndarray, angle_min: float, angle_max: float, bins: int,
                     minimum_height_m: float, maximum_height_m: float,
                     range_min: float, range_max: float) -> np.ndarray:
    """Project filtered XYZ points into angular bins, retaining nearest hits."""
    ranges = np.full(int(bins), np.inf, dtype=np.float32)
    if not points.size:
        return ranges
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    distance = np.hypot(points[:, 0], points[:, 1])
    angles = np.arctan2(points[:, 1], points[:, 0])
    valid = (np.isfinite(points).all(axis=1) & (points[:, 2] >= minimum_height_m) &
             (points[:, 2] <= maximum_height_m) & (distance >= range_min) &
             (distance <= range_max) & (angles >= angle_min) & (angles <= angle_max))
    if not np.any(valid):
        return ranges
    indices = np.floor((angles[valid] - angle_min) * (bins - 1) / (angle_max - angle_min)).astype(int)
    np.minimum.at(ranges, np.clip(indices, 0, bins - 1), distance[valid])
    return ranges


class VoxelCloudToScan(Node):
    def __init__(self) -> None:
        super().__init__("voxel_cloud_to_scan")
        defaults = (("minimum_height_m", -0.9), ("maximum_height_m", 0.15),
                    ("range_min_m", 0.45), ("range_max_m", 8.0),
                    ("angle_min_rad", -0.75), ("angle_max_rad", 0.75), ("bins", 320))
        for name, value in defaults:
            self.declare_parameter(name, value)
        self.publisher = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/camera/depth/points_filtered", self._cloud, qos_profile_sensor_data)

    def _cloud(self, message: PointCloud2) -> None:
        if message.point_step < 12 or not message.data:
            return
        # DepthPointCloud publishes tightly packed little-endian float32 XYZ.
        raw = np.frombuffer(bytes(message.data), dtype=np.float32)
        stride = message.point_step // 4
        points = raw.reshape(-1, stride)[:, :3]
        amin = float(self.get_parameter("angle_min_rad").value)
        amax = float(self.get_parameter("angle_max_rad").value)
        bins = int(self.get_parameter("bins").value)
        rmin = float(self.get_parameter("range_min_m").value)
        rmax = float(self.get_parameter("range_max_m").value)
        ranges = points_to_ranges(points, amin, amax, bins,
                                  float(self.get_parameter("minimum_height_m").value),
                                  float(self.get_parameter("maximum_height_m").value), rmin, rmax)
        scan = LaserScan()
        scan.header = message.header
        scan.angle_min, scan.angle_max = amin, amax
        scan.angle_increment = (amax - amin) / max(1, bins - 1)
        scan.scan_time = 1.0 / 15.0
        scan.range_min, scan.range_max = rmin, rmax
        scan.ranges = ranges.tolist()
        self.publisher.publish(scan)


def main() -> None:
    rclpy.init()
    node = VoxelCloudToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
