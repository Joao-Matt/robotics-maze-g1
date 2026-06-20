"""Gated full-3D point-cloud ICP odometry with IMU/control prediction."""
from __future__ import annotations
import json,math
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo,Image,Imu
from std_msgs.msg import String
from g1_mujoco_bridge.depth_point_cloud import depth_to_points


def _nearest(source,target,cell_size=.5):
    buckets={}
    for index,key in enumerate(np.floor(target/cell_size).astype(np.int32)):buckets.setdefault(tuple(key),[]).append(index)
    indices=np.zeros(len(source),dtype=int); errors=np.full(len(source),np.inf,dtype=float)
    for i,(point,key) in enumerate(zip(source,np.floor(source/cell_size).astype(np.int32))):
        candidates=[]
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                for dz in (-1,0,1):candidates.extend(buckets.get((int(key[0]+dx),int(key[1]+dy),int(key[2]+dz)),()))
        if candidates:
            values=np.asarray(candidates,dtype=int); distances=np.linalg.norm(target[values]-point,axis=1); best=int(distances.argmin()); indices[i]=values[best]; errors[i]=distances[best]
    return indices,errors


def _fit3(source,target):
    a,b=source.mean(0),target.mean(0); u,_,vt=np.linalg.svd((source-a).T@(target-b)); rotation=vt.T@u.T
    if np.linalg.det(rotation)<0:vt[-1]*=-1; rotation=vt.T@u.T
    return rotation,b-rotation@a


def match_clouds(previous,current,dx,dyaw,iterations=4,max_correspondence=.5):
    c,s=math.cos(dyaw),math.sin(dyaw); rotation=np.asarray(((c,-s,0),(s,c,0),(0,0,1.0))); translation=np.asarray((dx,0,0.0)); ratio=0.; rmse=float("inf")
    for _ in range(iterations):
        transformed=current@rotation.T+translation; nearest,errors=_nearest(transformed,previous,max_correspondence); valid=errors<max_correspondence; ratio=float(valid.mean())
        if valid.sum()<20:break
        rotation,translation=_fit3(current[valid],previous[nearest[valid]]); rmse=float(np.sqrt(np.mean(errors[valid]**2)))
    yaw=math.atan2(rotation[1,0],rotation[0,0]); roll=math.atan2(rotation[2,1],rotation[2,2]); pitch=math.asin(max(-1.,min(1.,-rotation[2,0])))
    return float(translation[0]),float(translation[1]),yaw,roll,pitch,rmse,ratio


class PointCloudImuOdometry(Node):
    def __init__(self):
        super().__init__("pointcloud_imu_odometry")
        self.declare_parameter("linear_prediction_scale",.65); self.declare_parameter("maximum_icp_correction_m",.15)
        self.previous=None; self.previous_ns=None; self.x=self.y=self.yaw=0.; self.command=0.; self.imu_yaw=None; self.sequence=0; self.accepted_count=0; self.rejected_count=0; self.info=None
        self.publisher=self.create_publisher(Odometry,"/odometry/depth_icp_raw",30); self.status=self.create_publisher(String,"/odometry/d435i_status",10)
        self.create_subscription(CameraInfo,"/camera/depth/camera_info",lambda m:setattr(self,"info",m),10)
        self.create_subscription(Image,"/camera/depth/image_rect_raw",self._depth,qos_profile_sensor_data)
        self.create_subscription(TwistStamped,"/applied_cmd_vel",lambda m:setattr(self,"command",float(m.twist.linear.x)),20)
        self.create_subscription(Imu,"/imu/data",self._imu,qos_profile_sensor_data)
    def _imu(self,m):
        if math.isfinite(float(m.angular_velocity.z)):self.imu_yaw=float(m.angular_velocity.z)
    def _depth(self,m):
        if self.info is None or m.encoding!="32FC1":return
        depth=np.frombuffer(bytes(m.data),dtype=np.float32).reshape(m.height,m.width)
        points=depth_to_points(depth,self.info.k,pitch_deg=10.,voxel_size_m=.05,pixel_step=2).astype(np.float64)
        if len(points)>500:points=points[np.linspace(0,len(points)-1,500,dtype=int)]
        stamp=rclpy.time.Time.from_msg(m.header.stamp).nanoseconds; dt=0 if self.previous_ns is None else max(0.,min(2.,(stamp-self.previous_ns)*1e-9)); prior_dx=self.command*dt*float(self.get_parameter("linear_prediction_scale").value); prior_yaw=(self.imu_yaw or 0.)*dt
        accepted=False; rmse=0.; ratio=1.; dx=dy=dyaw=0.
        if self.previous is not None and dt>0:
            dx,dy,dyaw,roll,pitch,rmse,ratio=match_clouds(self.previous,points,prior_dx,prior_yaw)
            accepted=math.isfinite(rmse) and ratio>=.15 and math.hypot(dx-prior_dx,dy)<=float(self.get_parameter("maximum_icp_correction_m").value) and abs(dyaw-prior_yaw)<=.3 and max(abs(roll),abs(pitch))<=.35
            if accepted:self.accepted_count+=1
            else:self.rejected_count+=1; dx,dy,dyaw=prior_dx,0.,prior_yaw
            c,s=math.cos(self.yaw),math.sin(self.yaw); self.x+=c*dx-s*dy; self.y+=s*dx+c*dy; self.yaw=math.atan2(math.sin(self.yaw+dyaw),math.cos(self.yaw+dyaw))
        self.previous,self.previous_ns=points,stamp; self.sequence+=1; variance=max(.03,min(.3,rmse*rmse if accepted else .15)); qz,qw=math.sin(self.yaw/2),math.cos(self.yaw/2)
        odom=Odometry(); odom.header=m.header; odom.header.frame_id="odom"; odom.child_frame_id="base_link"; odom.pose.pose.position.x=self.x; odom.pose.pose.position.y=self.y; odom.pose.pose.orientation.z=qz; odom.pose.pose.orientation.w=qw
        odom.twist.twist.linear.x=dx/dt if dt else 0.; odom.twist.twist.linear.y=dy/dt if dt else 0.; odom.twist.twist.angular.z=dyaw/dt if dt else 0.
        for covariance in (odom.pose.covariance,odom.twist.covariance):covariance[0]=covariance[7]=variance; covariance[35]=variance
        self.publisher.publish(odom); self.status.publish(String(data=json.dumps({"source":"gated_3d_cloud_icp_imu","sequence":self.sequence,"icp_accepted":accepted,"accepted_count":self.accepted_count,"rejected_count":self.rejected_count,"linear_prediction_scale":float(self.get_parameter("linear_prediction_scale").value),"rmse_m":rmse if math.isfinite(rmse) else None,"inlier_ratio":ratio,"ground_truth":False})))


def main():
    rclpy.init(); node=PointCloudImuOdometry()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
