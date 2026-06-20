"""Planar D435i scan odometry with a command prediction and depth correction.

The command is only an ICP initial guess.  The published pose is corrected from
successive D435i depth scans and never reads MuJoCo ground truth.
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def scan_points(message: LaserScan, maximum_points: int = 180) -> np.ndarray:
    ranges=np.asarray(message.ranges,dtype=np.float64)
    indices=np.arange(ranges.size)
    valid=np.isfinite(ranges)&(ranges>=message.range_min)&(ranges<=message.range_max)
    indices,ranges=indices[valid],ranges[valid]
    if ranges.size>maximum_points:
        chosen=np.linspace(0,ranges.size-1,maximum_points,dtype=int); indices,ranges=indices[chosen],ranges[chosen]
    angles=message.angle_min+indices*message.angle_increment
    return np.column_stack((ranges*np.cos(angles),ranges*np.sin(angles)))


def rigid_fit(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray,np.ndarray]:
    source_mean,target_mean=source.mean(axis=0),target.mean(axis=0)
    u,_,vt=np.linalg.svd((source-source_mean).T@(target-target_mean))
    rotation=vt.T@u.T
    if np.linalg.det(rotation)<0:
        vt[-1]*=-1; rotation=vt.T@u.T
    return rotation,target_mean-rotation@source_mean


def scan_match(previous: np.ndarray, current: np.ndarray, dx: float, dy: float, dyaw: float,
               maximum_correspondence: float = .35, iterations: int = 8) -> tuple[float,float,float,float,float]:
    """Return current-base pose in the previous-base frame plus RMSE/inlier ratio."""
    c,s=math.cos(dyaw),math.sin(dyaw); rotation=np.asarray(((c,-s),(s,c))); translation=np.asarray((dx,dy))
    inliers=np.zeros(current.shape[0],dtype=bool); distances=np.empty((current.shape[0],previous.shape[0]))
    for _ in range(iterations):
        transformed=current@rotation.T+translation
        distances=((transformed[:,None,:]-previous[None,:,:])**2).sum(axis=2)
        nearest=distances.argmin(axis=1); errors=np.sqrt(distances[np.arange(current.shape[0]),nearest])
        inliers=errors<maximum_correspondence
        if inliers.sum()<8:break
        rotation,translation=rigid_fit(current[inliers],previous[nearest[inliers]])
    if inliers.sum()<8:return dx,dy,dyaw,float("inf"),0.0
    transformed=current@rotation.T+translation
    errors=np.sqrt(((transformed[inliers]-previous[distances.argmin(axis=1)[inliers]])**2).sum(axis=1))
    yaw=math.atan2(rotation[1,0],rotation[0,0])
    return float(translation[0]),float(translation[1]),yaw,float(np.sqrt(np.mean(errors**2))),float(inliers.mean())


class D435iScanOdometry(Node):
    def __init__(self):
        super().__init__("d435i_scan_odometry")
        self.previous=None; self.previous_stamp=None; self.x=self.y=self.yaw=0.0
        self.command=(0.0,0.0); self.imu_yaw_rate=None; self.last_scan_wall=time.monotonic(); self.sequence=0
        self.publisher=self.create_publisher(Odometry,"/odom",50)
        self.status=self.create_publisher(String,"/odometry/d435i_status",10)
        self.broadcaster=TransformBroadcaster(self)
        self.create_subscription(LaserScan,"/scan",self._scan,qos_profile_sensor_data)
        self.create_subscription(TwistStamped,"/applied_cmd_vel",self._command,20)
        self.create_subscription(Imu,"/imu/data",self._imu,qos_profile_sensor_data)

    def _command(self,message):self.command=(float(message.twist.linear.x),float(message.twist.angular.z))
    def _imu(self,message):
        value=float(message.angular_velocity.z)
        if math.isfinite(value):self.imu_yaw_rate=value

    def _scan(self,message):
        points=scan_points(message); stamp=rclpy.time.Time.from_msg(message.header.stamp); now_ns=stamp.nanoseconds
        if points.shape[0]<8:return
        dt=0.0 if self.previous_stamp is None else max(0.0,min(2.0,(now_ns-self.previous_stamp)*1e-9))
        predicted_dx=self.command[0]*dt; predicted_yaw=(self.imu_yaw_rate if self.imu_yaw_rate is not None else self.command[1])*dt
        rmse,ratio=0.0,1.0
        if self.previous is not None and dt>0:
            dx,dy,dyaw,rmse,ratio=scan_match(self.previous,points,predicted_dx,0.0,predicted_yaw)
            # A repetitive corridor can create a false distant match.  Sensor
            # correction is bounded around the short command prediction.
            if not math.isfinite(rmse) or ratio<.12 or math.hypot(dx-predicted_dx,dy)>.22 or abs(dyaw-predicted_yaw)>.25:
                dx,dy,dyaw=predicted_dx,0.0,predicted_yaw
            c,s=math.cos(self.yaw),math.sin(self.yaw)
            self.x+=c*dx-s*dy; self.y+=s*dx+c*dy; self.yaw=math.atan2(math.sin(self.yaw+dyaw),math.cos(self.yaw+dyaw))
            vx=dx/dt; vy=dy/dt; wz=dyaw/dt
        else:vx=vy=wz=0.0
        self.previous,self.previous_stamp=points,now_ns; self.sequence+=1
        self._publish(message.header.stamp,vx,vy,wz,rmse,ratio)

    def _publish(self,stamp,vx,vy,wz,rmse,ratio):
        qz,qw=math.sin(self.yaw/2),math.cos(self.yaw/2)
        odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id="odom"; odom.child_frame_id="base_link"
        odom.pose.pose.position.x=self.x; odom.pose.pose.position.y=self.y; odom.pose.pose.orientation.z=qz; odom.pose.pose.orientation.w=qw
        variance=max(.0025,min(.25,rmse*rmse if math.isfinite(rmse) else .25)); odom.pose.covariance[0]=variance; odom.pose.covariance[7]=variance; odom.pose.covariance[35]=variance*2
        odom.twist.twist.linear.x=vx; odom.twist.twist.linear.y=vy; odom.twist.twist.angular.z=wz
        odom.twist.covariance[0]=variance; odom.twist.covariance[7]=variance; odom.twist.covariance[35]=variance*2
        self.publisher.publish(odom)
        transform=TransformStamped(); transform.header=odom.header; transform.child_frame_id="base_link"; transform.transform.translation.x=self.x; transform.transform.translation.y=self.y; transform.transform.rotation=odom.pose.pose.orientation
        self.broadcaster.sendTransform(transform)
        self.status.publish(String(data=(f'{{"source":"d435i_depth_scan_imu","linear_command_prediction":true,"imu_yaw_prediction":true,"ground_truth":false,'
                                        f'"sequence":{self.sequence},"rmse_m":{rmse if math.isfinite(rmse) else 999.0:.5f},"inlier_ratio":{ratio:.5f}}}')))


def main():
    rclpy.init(); node=D435iScanOdometry()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
