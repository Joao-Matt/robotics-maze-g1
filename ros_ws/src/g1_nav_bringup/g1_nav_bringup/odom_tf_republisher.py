"""Publish current-time TF from the latest sensor-derived RGB-D odometry pose."""
import rclpy
import copy, math
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfRepublisher(Node):
    def __init__(self):
        super().__init__("rgbd_odom_tf")
        self.latest=None; self.broadcaster=TransformBroadcaster(self); self.odom_pub=self.create_publisher(Odometry,"/odom",50)
        self.create_subscription(Odometry,"/odometry/rgbd_raw",self._odom,50)
        self.create_timer(.02,self._publish,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def _odom(self,msg):
        values=(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        if not all(math.isfinite(v) for v in values[:3]):return
        q=msg.pose.pose.orientation; norm=math.sqrt(sum(v*v for v in (q.x,q.y,q.z,q.w)))
        clean=copy.deepcopy(msg)
        if not math.isfinite(norm) or norm<.5:
            clean.pose.pose.orientation.x=clean.pose.pose.orientation.y=clean.pose.pose.orientation.z=0.0; clean.pose.pose.orientation.w=1.0
        else:
            clean.pose.pose.orientation.x/=norm; clean.pose.pose.orientation.y/=norm; clean.pose.pose.orientation.z/=norm; clean.pose.pose.orientation.w/=norm
        self.latest=clean
    def _publish(self):
        if self.latest is None:return
        stamp=self.get_clock().now().to_msg(); odom=copy.deepcopy(self.latest); odom.header.stamp=stamp; odom.header.frame_id="odom"; odom.child_frame_id="base_link"; self.odom_pub.publish(odom)
        msg=TransformStamped(); msg.header.stamp=stamp; msg.header.frame_id="odom"; msg.child_frame_id="base_link"
        msg.transform.translation.x=self.latest.pose.pose.position.x; msg.transform.translation.y=self.latest.pose.pose.position.y; msg.transform.translation.z=self.latest.pose.pose.position.z
        msg.transform.rotation=self.latest.pose.pose.orientation; self.broadcaster.sendTransform(msg)


def main():
    rclpy.init(); node=OdomTfRepublisher()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
