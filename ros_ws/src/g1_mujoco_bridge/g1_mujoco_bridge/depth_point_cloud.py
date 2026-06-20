"""Publish a voxel-filtered 3D PointCloud2 directly from D435i depth."""

from __future__ import annotations

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


def depth_to_points(depth: np.ndarray, intrinsics, pitch_deg: float = 10.0,
                    voxel_size_m: float = .05, pixel_step: int = 2,
                    minimum_depth_m: float = .3, maximum_depth_m: float = 8.0) -> np.ndarray:
    """Back-project depth into d435i_link and retain one point per 3D voxel."""
    fx,fy,cx,cy=(float(intrinsics[i]) for i in (0,4,2,5)); step=max(1,int(pixel_step))
    sampled=np.asarray(depth,dtype=np.float32)[::step,::step]
    rows=np.arange(0,depth.shape[0],step,dtype=np.float32)[:,None]
    cols=np.arange(0,depth.shape[1],step,dtype=np.float32)[None,:]
    right=(cols-cx)*sampled/fx; down=(rows-cy)*sampled/fy; forward=sampled
    pitch=math.radians(float(pitch_deg)); c,s=math.cos(pitch),math.sin(pitch)
    # Optical (right, down, forward) to the physical D435i link, including mount pitch.
    x=c*forward-s*down; y=-right; z=-s*forward-c*down
    valid=np.isfinite(sampled)&(sampled>=minimum_depth_m)&(sampled<=maximum_depth_m)&(x>0)
    points=np.column_stack((x[valid],y[valid],z[valid])).astype(np.float32,copy=False)
    if not points.size:return points.reshape(0,3)
    if voxel_size_m>0:
        keys=np.floor(points/float(voxel_size_m)).astype(np.int32)
        _,indices=np.unique(keys,axis=0,return_index=True); points=points[np.sort(indices)]
    return np.ascontiguousarray(points,dtype=np.float32)


class DepthPointCloud(Node):
    def __init__(self):
        super().__init__("d435i_depth_point_cloud")
        for name,value in (("pitch_deg",10.0),("voxel_size_m",.05),("pixel_step",2),("minimum_depth_m",.3),("maximum_depth_m",8.0),("maximum_points",20000)):
            self.declare_parameter(name,value)
        self.info=None; self.create_subscription(CameraInfo,"/camera/depth/camera_info",lambda m:setattr(self,"info",m),10)
        self.create_subscription(Image,"/camera/depth/image_rect_raw",self._depth,qos_profile_sensor_data)
        self.publisher=self.create_publisher(PointCloud2,"/camera/depth/points_filtered",qos_profile_sensor_data)

    def _depth(self,message):
        if self.info is None or message.encoding!="32FC1":return
        depth=np.frombuffer(bytes(message.data),dtype=np.float32).reshape(message.height,message.width)
        points=depth_to_points(depth,self.info.k,float(self.get_parameter("pitch_deg").value),float(self.get_parameter("voxel_size_m").value),int(self.get_parameter("pixel_step").value),float(self.get_parameter("minimum_depth_m").value),float(self.get_parameter("maximum_depth_m").value))
        maximum=int(self.get_parameter("maximum_points").value)
        if points.shape[0]>maximum:points=points[np.linspace(0,points.shape[0]-1,maximum,dtype=int)]
        cloud=PointCloud2(); cloud.header=message.header; cloud.header.frame_id="d435i_link"; cloud.height=1; cloud.width=points.shape[0]
        cloud.fields=[PointField(name=name,offset=offset,datatype=PointField.FLOAT32,count=1) for name,offset in (("x",0),("y",4),("z",8))]
        cloud.is_bigendian=False; cloud.point_step=12; cloud.row_step=12*cloud.width; cloud.is_dense=True; cloud.data=points.tobytes(); self.publisher.publish(cloud)


def main():
    rclpy.init(); node=DepthPointCloud()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
