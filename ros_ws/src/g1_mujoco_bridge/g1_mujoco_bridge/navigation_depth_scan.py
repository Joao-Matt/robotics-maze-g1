"""Project pitched D435 depth into a floor-filtered planar navigation scan."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan


class NavigationDepthScan(Node):
    def __init__(self):
        super().__init__("navigation_depth_scan")
        self.info=None
        self.declare_parameter("pitch_deg",10.0); self.declare_parameter("minimum_height_m",-0.9); self.declare_parameter("maximum_height_m",0.15)
        self.create_subscription(CameraInfo,"/camera/depth/camera_info",lambda m:setattr(self,"info",m),10)
        self.create_subscription(Image,"/camera/depth/image_rect_raw",self._depth,qos_profile_sensor_data)
        self.pub=self.create_publisher(LaserScan,"/scan",qos_profile_sensor_data)
    def _depth(self,msg):
        if self.info is None or msg.encoding!="32FC1":return
        depth=np.frombuffer(bytes(msg.data),dtype=np.float32).reshape(msg.height,msg.width)
        fx,fy,cx,cy=self.info.k[0],self.info.k[4],self.info.k[2],self.info.k[5]
        rows=np.arange(msg.height,dtype=np.float32)[:,None]; cols=np.arange(msg.width,dtype=np.float32)[None,:]
        xo=(cols-cx)*depth/fx; yo=(rows-cy)*depth/fy; zo=depth
        pitch=math.radians(float(self.get_parameter("pitch_deg").value)); c,s=math.cos(pitch),math.sin(pitch)
        # optical (right,down,forward) -> link (forward,left,up), then downward pitch
        xl=c*zo-s*yo; yl=-xo; zl=-s*zo-c*yo
        horizontal=np.hypot(xl,yl); low=float(self.get_parameter("minimum_height_m").value); high=float(self.get_parameter("maximum_height_m").value)
        valid=np.isfinite(depth)&(depth>0)&(horizontal>=.45)&(horizontal<=8.0)&(zl>=low)&(zl<=high)&(xl>0)
        ranges=np.full(msg.width,np.inf,dtype=np.float32)
        for col in range(msg.width):
            values=horizontal[:,col][valid[:,col]]
            if values.size:ranges[col]=values.min()
        hfov=2*math.atan(msg.width/(2*fx)); scan=LaserScan(); scan.header=msg.header; scan.header.frame_id="d435i_link"
        scan.angle_min=-hfov/2; scan.angle_max=hfov/2; scan.angle_increment=hfov/max(1,msg.width-1); scan.time_increment=0.0; scan.scan_time=.1; scan.range_min=.45; scan.range_max=8.0; scan.ranges=ranges[::-1].tolist(); self.pub.publish(scan)


def main():
    rclpy.init(); node=NavigationDepthScan()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
