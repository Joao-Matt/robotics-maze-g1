"""Planar D435i scan odometry with a command prediction and depth correction.

The command is only an ICP initial guess.  The published pose is corrected from
successive D435i depth scans and never reads MuJoCo ground truth.
"""

from __future__ import annotations

from collections import deque
import math
import time
import json

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


def sensor_delta_from_base(dx: float, dy: float, dyaw: float, offset_x: float, offset_y: float) -> tuple[float, float]:
    c,s=math.cos(dyaw),math.sin(dyaw)
    return dx+(c-1.0)*offset_x-s*offset_y,dy+s*offset_x+(c-1.0)*offset_y


def base_delta_from_sensor(dx: float, dy: float, dyaw: float, offset_x: float, offset_y: float) -> tuple[float, float]:
    c,s=math.cos(dyaw),math.sin(dyaw)
    return dx+(1.0-c)*offset_x+s*offset_y,dy-s*offset_x+(1.0-c)*offset_y


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float | None:
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm<1e-9:return None
    x,y,z,w=x/norm,y/norm,z/norm,w/norm
    return math.atan2(2.0*(w*z+x*y),1.0-2.0*(y*y+z*z))


class D435iScanOdometry(Node):
    def __init__(self):
        super().__init__("d435i_scan_odometry")
        for name, default in (
            ("output_odom_topic", "/odom"),
            ("publish_tf", True),
            ("odom_frame_id", "odom"),
            ("base_frame_id", "base_link"),
            ("sensor_offset_x_m", 0.0),
            ("sensor_offset_y_m", 0.0),
            ("linear_prediction_scale", 1.0),
            ("angular_prediction_scale", 1.0),
            ("imu_yaw_rate_scale", 1.0),
            ("use_imu_orientation_yaw", True),
            ("imu_orientation_yaw_scale", 1.0),
            ("odometry_translation_scale", 1.0),
            ("odometry_yaw_scale", 1.0),
            ("scan_translation_correction_weight", 1.0),
            ("scan_yaw_correction_weight", 1.0),
            ("command_latency_s", 0.0),
            ("imu_latency_s", 0.0),
            ("command_timeout_s", 0.35),
            ("imu_timeout_s", 0.35),
            ("scan_maximum_points", 180),
            ("icp_maximum_correspondence_m", 0.35),
            ("icp_iterations", 8),
            ("icp_min_inlier_ratio", 0.12),
            ("icp_max_prediction_translation_error_m", 0.22),
            ("icp_max_prediction_yaw_error_rad", 0.25),
            ("max_step_translation_m", 0.35),
            ("max_step_yaw_rad", 0.5),
            ("odom_dt_max_s", 2.0),
            ("odom_covariance_min", 0.0025),
            ("odom_covariance_max", 0.25),
        ):
            self.declare_parameter(name, default)
        self.output_odom_topic=str(self.get_parameter("output_odom_topic").value).strip() or "/odom"
        self.publish_tf=bool(self.get_parameter("publish_tf").value)
        self.odom_frame_id=str(self.get_parameter("odom_frame_id").value).strip() or "odom"
        self.base_frame_id=str(self.get_parameter("base_frame_id").value).strip() or "base_link"
        self.previous=None; self.previous_stamp=None; self.x=self.y=self.yaw=0.0
        self.command_history=deque(maxlen=200); self.imu_history=deque(maxlen=400)
        self.last_scan_wall=time.monotonic(); self.sequence=0; self.accepted_count=0; self.rejected_count=0; self.step_limited_count=0
        self.publisher=self.create_publisher(Odometry,self.output_odom_topic,50)
        self.status=self.create_publisher(String,"/odometry/d435i_status",10)
        self.broadcaster=TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(LaserScan,"/scan",self._scan,qos_profile_sensor_data)
        self.create_subscription(TwistStamped,"/applied_cmd_vel",self._command,20)
        self.create_subscription(Imu,"/imu/data",self._imu,qos_profile_sensor_data)

    def _stamp_ns(self,message):
        return rclpy.time.Time.from_msg(message.header.stamp).nanoseconds

    def _command(self,message):
        self.command_history.append((self._stamp_ns(message),float(message.twist.linear.x),float(message.twist.angular.z)))

    def _imu(self,message):
        value=float(message.angular_velocity.z)
        yaw=quaternion_yaw(float(message.orientation.x),float(message.orientation.y),float(message.orientation.z),float(message.orientation.w))
        if math.isfinite(value) or yaw is not None:self.imu_history.append((self._stamp_ns(message),value if math.isfinite(value) else None,yaw))

    @staticmethod
    def _latest_at(history,target_ns,timeout_s):
        if not history:return None,None
        timeout_ns=max(0.0,float(timeout_s))*1e9
        chosen=None
        for item in reversed(history):
            if item[0]<=target_ns:
                chosen=item;break
        if chosen is None:chosen=history[0]
        age_ns=abs(target_ns-chosen[0])
        if age_ns>timeout_ns:return None,age_ns*1e-9
        return chosen,age_ns*1e-9

    def _scan(self,message):
        maximum_points=max(8,int(self.get_parameter("scan_maximum_points").value))
        points=scan_points(message,maximum_points); stamp=rclpy.time.Time.from_msg(message.header.stamp); now_ns=stamp.nanoseconds
        if points.shape[0]<8:return
        dt_limit=max(0.0,float(self.get_parameter("odom_dt_max_s").value))
        dt=0.0 if self.previous_stamp is None else max(0.0,min(dt_limit,(now_ns-self.previous_stamp)*1e-9))
        command_target=now_ns-int(float(self.get_parameter("command_latency_s").value)*1e9)
        imu_target=now_ns-int(float(self.get_parameter("imu_latency_s").value)*1e9)
        command,command_age=self._latest_at(self.command_history,command_target,float(self.get_parameter("command_timeout_s").value))
        imu_timeout=float(self.get_parameter("imu_timeout_s").value)
        imu,imu_age=self._latest_at(self.imu_history,imu_target,imu_timeout)
        previous_imu,_=self._latest_at(self.imu_history,self.previous_stamp-int(float(self.get_parameter("imu_latency_s").value)*1e9),imu_timeout) if self.previous_stamp is not None else (None,None)
        command_vx=0.0 if command is None else float(command[1])
        command_wz=0.0 if command is None else float(command[2])
        predicted_base_dx=command_vx*float(self.get_parameter("linear_prediction_scale").value)*dt
        yaw_source="command"
        yaw_rate=(float(imu[1])*float(self.get_parameter("imu_yaw_rate_scale").value)) if imu is not None and imu[1] is not None else command_wz*float(self.get_parameter("angular_prediction_scale").value)
        predicted_yaw=yaw_rate*dt
        if bool(self.get_parameter("use_imu_orientation_yaw").value) and imu is not None and previous_imu is not None and imu[2] is not None and previous_imu[2] is not None:
            predicted_yaw=math.atan2(math.sin(float(imu[2])-float(previous_imu[2])),math.cos(float(imu[2])-float(previous_imu[2])))*float(self.get_parameter("imu_orientation_yaw_scale").value)
            yaw_source="imu_orientation"
        elif imu is not None and imu[1] is not None:
            yaw_source="imu_rate"
        offset_x=float(self.get_parameter("sensor_offset_x_m").value); offset_y=float(self.get_parameter("sensor_offset_y_m").value)
        predicted_sensor_dx,predicted_sensor_dy=sensor_delta_from_base(predicted_base_dx,0.0,predicted_yaw,offset_x,offset_y)
        rmse,ratio=0.0,1.0; accepted=False; step_limited=False
        if self.previous is not None and dt>0:
            sensor_dx,sensor_dy,dyaw,rmse,ratio=scan_match(
                self.previous,
                points,
                predicted_sensor_dx,
                predicted_sensor_dy,
                predicted_yaw,
                maximum_correspondence=float(self.get_parameter("icp_maximum_correspondence_m").value),
                iterations=max(1,int(self.get_parameter("icp_iterations").value)),
            )
            # A repetitive corridor can create a false distant match.  Sensor
            # correction is bounded around the short command prediction.
            min_ratio=float(self.get_parameter("icp_min_inlier_ratio").value)
            max_translation_error=float(self.get_parameter("icp_max_prediction_translation_error_m").value)
            max_yaw_error=float(self.get_parameter("icp_max_prediction_yaw_error_rad").value)
            accepted=math.isfinite(rmse) and ratio>=min_ratio and math.hypot(sensor_dx-predicted_sensor_dx,sensor_dy-predicted_sensor_dy)<=max_translation_error and abs(dyaw-predicted_yaw)<=max_yaw_error
            if not accepted:
                self.rejected_count+=1; sensor_dx,sensor_dy,dyaw=predicted_sensor_dx,predicted_sensor_dy,predicted_yaw
            else:
                self.accepted_count+=1
                translation_weight=max(0.0,min(1.0,float(self.get_parameter("scan_translation_correction_weight").value)))
                yaw_weight=max(0.0,min(1.0,float(self.get_parameter("scan_yaw_correction_weight").value)))
                sensor_dx=predicted_sensor_dx+translation_weight*(sensor_dx-predicted_sensor_dx)
                sensor_dy=predicted_sensor_dy+translation_weight*(sensor_dy-predicted_sensor_dy)
                yaw_correction=math.atan2(math.sin(dyaw-predicted_yaw),math.cos(dyaw-predicted_yaw))
                dyaw=predicted_yaw+yaw_weight*yaw_correction
            dx,dy=base_delta_from_sensor(sensor_dx,sensor_dy,dyaw,offset_x,offset_y)
            motion_scale=max(0.0,float(self.get_parameter("odometry_translation_scale").value))
            yaw_scale=max(0.0,float(self.get_parameter("odometry_yaw_scale").value))
            dx*=motion_scale; dy*=motion_scale; dyaw*=yaw_scale
            max_step_translation=float(self.get_parameter("max_step_translation_m").value)
            max_step_yaw=float(self.get_parameter("max_step_yaw_rad").value)
            step_translation=math.hypot(dx,dy)
            if (max_step_translation>0.0 and step_translation>max_step_translation) or (max_step_yaw>0.0 and abs(dyaw)>max_step_yaw):
                self.step_limited_count+=1; step_limited=True
                if max_step_translation>0.0 and step_translation>max_step_translation:
                    scale=max_step_translation/max(1e-9,step_translation); dx*=scale; dy*=scale
                if max_step_yaw>0.0 and abs(dyaw)>max_step_yaw:
                    dyaw=math.copysign(max_step_yaw,dyaw)
            c,s=math.cos(self.yaw),math.sin(self.yaw)
            self.x+=c*dx-s*dy; self.y+=s*dx+c*dy; self.yaw=math.atan2(math.sin(self.yaw+dyaw),math.cos(self.yaw+dyaw))
            vx=dx/dt; vy=dy/dt; wz=dyaw/dt
        else:vx=vy=wz=0.0
        self.previous,self.previous_stamp=points,now_ns; self.sequence+=1
        self._publish(message.header.stamp,vx,vy,wz,rmse,ratio,accepted,step_limited,command_age,imu_age,predicted_base_dx,predicted_yaw,yaw_source)

    def _publish(self,stamp,vx,vy,wz,rmse,ratio,accepted,step_limited,command_age,imu_age,predicted_dx,predicted_yaw,yaw_source):
        qz,qw=math.sin(self.yaw/2),math.cos(self.yaw/2)
        odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id=self.odom_frame_id; odom.child_frame_id=self.base_frame_id
        odom.pose.pose.position.x=self.x; odom.pose.pose.position.y=self.y; odom.pose.pose.orientation.z=qz; odom.pose.pose.orientation.w=qw
        covariance_min=float(self.get_parameter("odom_covariance_min").value)
        covariance_max=float(self.get_parameter("odom_covariance_max").value)
        variance=max(covariance_min,min(covariance_max,rmse*rmse if math.isfinite(rmse) else covariance_max)); odom.pose.covariance[0]=variance; odom.pose.covariance[7]=variance; odom.pose.covariance[35]=variance*2
        odom.twist.twist.linear.x=vx; odom.twist.twist.linear.y=vy; odom.twist.twist.angular.z=wz
        odom.twist.covariance[0]=variance; odom.twist.covariance[7]=variance; odom.twist.covariance[35]=variance*2
        self.publisher.publish(odom)
        if self.broadcaster is not None:
            transform=TransformStamped(); transform.header=odom.header; transform.child_frame_id=self.base_frame_id; transform.transform.translation.x=self.x; transform.transform.translation.y=self.y; transform.transform.rotation=odom.pose.pose.orientation
            self.broadcaster.sendTransform(transform)
        self.status.publish(String(data=json.dumps({
            "source": "d435i_depth_scan_imu",
            "published_topic": self.output_odom_topic,
            "publishes_tf": self.publish_tf,
            "odom_frame_id": self.odom_frame_id,
            "base_frame_id": self.base_frame_id,
            "linear_command_prediction": True,
            "imu_yaw_prediction": True,
            "predicted_yaw_source": yaw_source,
            "ground_truth": False,
            "sequence": self.sequence,
            "icp_accepted": accepted,
            "accepted_count": self.accepted_count,
            "rejected_count": self.rejected_count,
            "step_limited": step_limited,
            "step_limited_count": self.step_limited_count,
            "command_age_s": command_age,
            "imu_age_s": imu_age,
            "predicted_dx_m": predicted_dx,
            "predicted_yaw_rad": predicted_yaw,
            "rmse_m": rmse if math.isfinite(rmse) else 999.0,
            "inlier_ratio": ratio,
            "sensor_offset_x_m": float(self.get_parameter("sensor_offset_x_m").value),
            "sensor_offset_y_m": float(self.get_parameter("sensor_offset_y_m").value),
            "linear_prediction_scale": float(self.get_parameter("linear_prediction_scale").value),
            "angular_prediction_scale": float(self.get_parameter("angular_prediction_scale").value),
            "imu_yaw_rate_scale": float(self.get_parameter("imu_yaw_rate_scale").value),
            "use_imu_orientation_yaw": bool(self.get_parameter("use_imu_orientation_yaw").value),
            "imu_orientation_yaw_scale": float(self.get_parameter("imu_orientation_yaw_scale").value),
            "odometry_translation_scale": float(self.get_parameter("odometry_translation_scale").value),
            "odometry_yaw_scale": float(self.get_parameter("odometry_yaw_scale").value),
            "scan_translation_correction_weight": float(self.get_parameter("scan_translation_correction_weight").value),
            "scan_yaw_correction_weight": float(self.get_parameter("scan_yaw_correction_weight").value),
            "command_latency_s": float(self.get_parameter("command_latency_s").value),
            "imu_latency_s": float(self.get_parameter("imu_latency_s").value),
            "command_timeout_s": float(self.get_parameter("command_timeout_s").value),
            "imu_timeout_s": float(self.get_parameter("imu_timeout_s").value),
            "max_step_translation_m": float(self.get_parameter("max_step_translation_m").value),
            "max_step_yaw_rad": float(self.get_parameter("max_step_yaw_rad").value),
            "scan_maximum_points": int(self.get_parameter("scan_maximum_points").value),
            "icp_maximum_correspondence_m": float(self.get_parameter("icp_maximum_correspondence_m").value),
            "icp_iterations": int(self.get_parameter("icp_iterations").value),
            "icp_min_inlier_ratio": float(self.get_parameter("icp_min_inlier_ratio").value),
            "icp_max_prediction_translation_error_m": float(self.get_parameter("icp_max_prediction_translation_error_m").value),
            "icp_max_prediction_yaw_error_rad": float(self.get_parameter("icp_max_prediction_yaw_error_rad").value),
        }, sort_keys=True)))


def main():
    rclpy.init(); node=D435iScanOdometry()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
