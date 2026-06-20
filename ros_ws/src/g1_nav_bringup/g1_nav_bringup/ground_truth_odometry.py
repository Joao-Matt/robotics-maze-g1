"""Expose MuJoCo ground truth as a planar, start-relative Nav2 odometry frame."""

from __future__ import annotations

import copy
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GroundTruthOdometry(Node):
    """Publish ``odom -> base_link`` from the exact simulated pelvis pose.

    The first ground-truth sample defines (0, 0, 0), keeping SLAM's map frame
    compatible with the existing start-relative map evaluation artifacts.
    """

    def __init__(self) -> None:
        super().__init__("ground_truth_odometry")
        self.initial = None
        self.publisher = self.create_publisher(Odometry, "/odom", 50)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/ground_truth/odom", self._ground_truth, 50)

    def _ground_truth(self, message: Odometry) -> None:
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        if self.initial is None:
            self.initial = (x, y, yaw)
        x0, y0, yaw0 = self.initial
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = x - x0, y - y0
        local_x, local_y = c * dx + s * dy, -s * dx + c * dy
        local_yaw = math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))

        odom = copy.deepcopy(message)
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = local_x
        odom.pose.pose.position.y = local_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(local_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(local_yaw / 2.0)
        vx, vy = float(message.twist.twist.linear.x), float(message.twist.twist.linear.y)
        odom.twist.twist.linear.x = c * vx + s * vy
        odom.twist.twist.linear.y = -s * vx + c * vy
        self.publisher.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = local_x
        transform.transform.translation.y = local_y
        transform.transform.rotation = odom.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = GroundTruthOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
