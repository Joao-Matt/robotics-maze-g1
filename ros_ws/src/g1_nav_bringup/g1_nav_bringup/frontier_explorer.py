"""Sensor-only frontier exploration and RGB-D goal-marker detection."""

from __future__ import annotations

import json, math, time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from g1_nav_bringup.navigation_evaluation import frontier_clusters, frontier_goal, largest_connected_component, path_is_known_free
from sim.mujoco_runner import _write_png


TERMINAL={"TIMEOUT","STUCK","FALL_DETECTED","COLLISION_ABORT","COMMAND_TIMEOUT","NAV2_ABORTED"}


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        for name,default in (("output_dir","/workspace/runs"),("live_visual_dir",""),("duration_s",600.0),("frontier_setback_m",0.5),("sensor_timeout_s",3.0)):
            self.declare_parameter(name,default)
        self.declare_parameter("raw_odometry_timeout_s",6.0)
        self.out=Path(str(self.get_parameter("output_dir").value)); self.out.mkdir(parents=True,exist_ok=True)
        live=str(self.get_parameter("live_visual_dir").value); self.live=Path(live) if live else None
        self.map=self.costmap=self.odom=self.rgb=self.depth=self.rgb_info=self.depth_info=None
        self.first_map_time=None; self.known_history=[]; self.frontier_events=[]; self.marker_events=[]; self.blacklist=[]
        self.current_goal=None; self.goal_kind=None; self.busy=False; self.armed=False; self.marker_streak=0; self.done=False; self.no_frontier_since=None
        self.bootstrap_start=None; self.bootstrap_complete=True
        self.last_odom_wall=0.0; self.last_raw_odom_wall=0.0; self.last_scan_wall=0.0; self.motion={}
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.compute=ActionClient(self,ComputePathToPose,"compute_path_to_pose")
        self.navigate=ActionClient(self,NavigateToPose,"navigate_to_pose")
        self.lifecycle={}; self.lifecycle_pending=set(); self.lifecycle_clients={name:self.create_client(GetState,f"/{name}/get_state") for name in ("planner_server","controller_server","bt_navigator","behavior_server")}
        self.arm_pub=self.create_publisher(Bool,"/navigation/armed",10)
        self.status_pub=self.create_publisher(String,"/exploration/status",10)
        self.frontier_goal_pub=self.create_publisher(PoseStamped,"/exploration/frontier_goal",10)
        self.marker_goal_pub=self.create_publisher(PoseStamped,"/exploration/marker_goal",10)
        self.bootstrap_cmd_pub=self.create_publisher(Twist,"/cmd_vel",10)
        self.create_subscription(OccupancyGrid,"/map",self._map,10)
        self.create_subscription(OccupancyGrid,"/global_costmap/costmap",lambda m:setattr(self,"costmap",m),10)
        self.create_subscription(Odometry,"/odom",self._odom,50)
        self.create_subscription(NavPath,"/plan",lambda m:None,20)
        self.create_subscription(Image,"/camera/color/image_raw",self._rgb,qos_profile_sensor_data)
        self.create_subscription(Image,"/camera/depth/image_rect_raw",self._depth,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,"/camera/color/camera_info",lambda m:setattr(self,"rgb_info",m),10)
        self.create_subscription(CameraInfo,"/camera/depth/camera_info",lambda m:setattr(self,"depth_info",m),10)
        self.create_subscription(LaserScan,"/scan",lambda m:setattr(self,"last_scan_wall",time.monotonic()),qos_profile_sensor_data)
        self.create_subscription(String,"/navigation/status",self._motion,20)
        self.create_subscription(String,"/odometry/d435i_status",lambda m:setattr(self,"last_raw_odom_wall",time.monotonic()),20)
        self.create_timer(1.0,self._tick)
        self.create_timer(.1,self._bootstrap_control)

    def _map(self,msg):
        self.map=msg; known=sum(v>=0 for v in msg.data); now=self.get_clock().now().nanoseconds*1e-9
        if self.first_map_time is None:self.first_map_time=now
        self.known_history.append((now,known))

    def _odom(self,msg):
        covariance=(msg.pose.covariance[0],msg.pose.covariance[7],msg.pose.covariance[35])
        if not all(math.isfinite(v) and 0.0<=v<=10.0 for v in covariance):
            if self.armed:self._finish("ODOMETRY_LOST")
            return
        self.odom=msg; self.last_odom_wall=time.monotonic()
    def _rgb(self,msg):self.rgb=msg; self._detect_marker()
    def _depth(self,msg):self.depth=msg
    def _motion(self,msg):
        try:self.motion=json.loads(msg.data)
        except Exception:self.motion={"status":msg.data}
        if self.motion.get("status") in TERMINAL:self._finish(str(self.motion["status"]))

    def _tick(self):
        if self.done:return
        self._check_lifecycle()
        now=time.monotonic()
        timeout=float(self.get_parameter("sensor_timeout_s").value)
        if self.odom is not None and now-self.last_odom_wall>timeout:return self._finish("ODOMETRY_LOST")
        if self.armed and self.last_raw_odom_wall and now-self.last_raw_odom_wall>float(self.get_parameter("raw_odometry_timeout_s").value):return self._finish("ODOMETRY_LOST")
        if self.last_scan_wall and now-self.last_scan_wall>timeout:return self._finish("SCAN_LOST")
        self._publish_status("NAVIGATING" if self.busy else "SELECTING_FRONTIER")
        if self.map is not None and self.odom is not None and not self.bootstrap_complete:
            if self.bootstrap_start is None:
                self.bootstrap_start=self.get_clock().now().nanoseconds*1e-9; self.armed=True; self.arm_pub.publish(Bool(data=True))
            return
        sensors_ready=self.odom is not None and self.last_scan_wall>0
        if not self.busy and self.map is not None and sensors_ready and all(self.lifecycle.get(name)=="active" for name in self.lifecycle_clients):self._select_frontier()

    def _bootstrap_control(self):
        if self.done or self.bootstrap_start is None or self.bootstrap_complete:return
        elapsed=self.get_clock().now().nanoseconds*1e-9-self.bootstrap_start
        command=Twist()
        if elapsed<8.0:command.angular.z=.5
        else:
            self.bootstrap_complete=True; self.armed=False; self.arm_pub.publish(Bool(data=False))
        self.bootstrap_cmd_pub.publish(command)

    def _check_lifecycle(self):
        for name,client in self.lifecycle_clients.items():
            if self.lifecycle.get(name)!="active" and name not in self.lifecycle_pending and client.service_is_ready():
                self.lifecycle_pending.add(name)
                future=client.call_async(GetState.Request()); future.add_done_callback(lambda f,node=name:self._lifecycle_response(node,f))
    def _lifecycle_response(self,name,future):
        self.lifecycle_pending.discard(name)
        try:self.lifecycle[name]=future.result().current_state.label
        except Exception:self.lifecycle[name]="unavailable"

    def _select_frontier(self):
        clusters=frontier_clusters(self.map.data,self.map.info.width,self.map.info.height)
        candidates=[]; resolution=self.map.info.resolution
        try:
            current_tf=self.tf.lookup_transform("map","base_link",rclpy.time.Time()); current=(current_tf.transform.translation.x,current_tf.transform.translation.y)
        except Exception:current=(0.0,0.0)
        for cluster in clusters:
            cell=frontier_goal(cluster,self.map.data,self.map.info.width,self.map.info.height,max(1,round(float(self.get_parameter("frontier_setback_m").value)/resolution)))
            if cell is None:continue
            x=self.map.info.origin.position.x+(cell[0]+.5)*resolution; y=self.map.info.origin.position.y+(cell[1]+.5)*resolution
            self.blacklist=[entry for entry in self.blacklist if time.monotonic()-entry[2]<30.0]
            if any(math.hypot(x-bx,y-by)<.75 for bx,by,_ in self.blacklist):continue
            path_cost_estimate=math.hypot(x-current[0],y-current[1]); score=float(len(cluster))-path_cost_estimate
            candidates.append((score,len(cluster),x,y))
        if not candidates:
            self.no_frontier_since=self.no_frontier_since or time.monotonic()
            if time.monotonic()-self.no_frontier_since>=20.0:self._finish("NO_REACHABLE_FRONTIER")
            return
        self.no_frontier_since=None
        candidates.sort(reverse=True); self._candidate_queue=candidates; self._preflight_next()

    def _preflight_next(self):
        if not self._candidate_queue:self.busy=False; return
        score,size,x,y=self._candidate_queue.pop(0); self.busy=True; self.current_goal=(x,y); self.goal_kind="frontier"; self.current_score=score
        if not self.compute.wait_for_server(timeout_sec=.2):self.frontier_events.append({"goal":self.current_goal,"event":"rejected","reason":"planner_unavailable"}); self.busy=False; return
        goal=ComputePathToPose.Goal(); goal.goal=self._pose(x,y); goal.use_start=False
        self.compute.send_goal_async(goal).add_done_callback(lambda f:self._compute_accepted(f,size))

    def _compute_accepted(self,future,size):
        handle=future.result()
        if not handle.accepted:self.frontier_events.append({"goal":self.current_goal,"event":"rejected","reason":"preflight_rejected"}); self.busy=False; return
        handle.get_result_async().add_done_callback(lambda f:self._path_ready(f,size))

    def _path_ready(self,future,size):
        path=future.result().result.path; points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
        source=self.costmap or self.map; safe=False
        if source and len(points)>1:
            info=source.info; safe=path_is_known_free(source.data,info.width,info.height,(info.origin.position.x,info.origin.position.y),info.resolution,points,max_cost=252 if self.costmap else 0)
        if not safe:self.frontier_events.append({"goal":self.current_goal,"event":"rejected","reason":"unsafe_or_unknown_path"}); self.blacklist.append((*self.current_goal,time.monotonic())); self.busy=False; return self._preflight_next()
        self.armed=True; self.arm_pub.publish(Bool(data=True)); self.frontier_goal_pub.publish(self._pose(*self.current_goal))
        self.frontier_events.append({"time_s":self.get_clock().now().nanoseconds*1e-9,"goal":self.current_goal,"cluster_size":size,"score":self.current_score,"event":"accepted"})
        self._send_navigation(*self.current_goal)

    def _send_navigation(self,x,y):
        if not self.navigate.wait_for_server(timeout_sec=.2):self.busy=False; return
        goal=NavigateToPose.Goal(); goal.pose=self._pose(x,y)
        self.navigate.send_goal_async(goal).add_done_callback(self._nav_accepted)
    def _nav_accepted(self,future):
        handle=future.result(); self._nav_handle=handle
        if not handle.accepted:self.blacklist.append((*self.current_goal,time.monotonic())); self.busy=False; return
        handle.get_result_async().add_done_callback(self._nav_done)
    def _nav_done(self,future):
        status=future.result().status
        if self.goal_kind=="marker":return self._finish("GOAL_REACHED" if status==4 else "NAV2_ABORTED")
        self.frontier_events.append({"time_s":self.get_clock().now().nanoseconds*1e-9,"goal":self.current_goal,"event":"completed" if status==4 else "navigation_failed","action_status":status})
        if status==4:self.blacklist.append((*self.current_goal,time.monotonic()))
        if status!=4:self.blacklist.append((*self.current_goal,time.monotonic()))
        self.arm_pub.publish(Bool(data=False)); self.armed=False; self.busy=False

    def _detect_marker(self):
        if not all(v is not None for v in (self.rgb,self.depth,self.rgb_info,self.depth_info)):return
        rgb=np.frombuffer(bytes(self.rgb.data),dtype=np.uint8).reshape(self.rgb.height,self.rgb.width,3)
        mask=largest_connected_component((rgb[:,:,0]>140)&(rgb[:,:,0]>1.6*rgb[:,:,1])&(rgb[:,:,0]>1.6*rgb[:,:,2]))
        count=int(mask.sum())
        if count<max(80,int(mask.size*.001)):
            self.marker_streak=0; return
        ys,xs=np.nonzero(mask); u=float(np.median(xs)); v=float(np.median(ys))
        kr,kd=self.rgb_info.k,self.depth_info.k; xn=(u-kr[2])/kr[0]; yn=(v-kr[5])/kr[4]
        ud=int(round(kd[0]*xn+kd[2])); vd=int(round(kd[4]*yn+kd[5]))
        depth=np.frombuffer(bytes(self.depth.data),dtype=np.float32).reshape(self.depth.height,self.depth.width)
        patch=depth[max(0,vd-8):vd+9,max(0,ud-8):ud+9]; valid=patch[np.isfinite(patch)&(patch>0)]
        if valid.size<patch.size*.3:self.marker_streak=0; return
        distance=float(np.median(valid)); self.marker_streak+=1
        if self.live:
            overlay=rgb.copy(); x0,x1,y0,y1=int(xs.min()),int(xs.max()),int(ys.min()),int(ys.max())
            overlay[y0:min(y0+3,overlay.shape[0]),x0:x1+1]=[255,255,0]; overlay[max(0,y1-2):y1+1,x0:x1+1]=[255,255,0]
            overlay[y0:y1+1,x0:min(x0+3,overlay.shape[1])]=[255,255,0]; overlay[y0:y1+1,max(0,x1-2):x1+1]=[255,255,0]
            self.live.mkdir(parents=True,exist_ok=True); _write_png(self.live/"marker_detection.png",overlay)
        if self.marker_streak<3 or self.goal_kind=="marker":return
        try:t=self.tf.lookup_transform("map",self.depth.header.frame_id,rclpy.time.Time())
        except Exception:return
        point=self._transform_point((xn*distance,yn*distance,distance),t.transform)
        self.marker_events.append({"time_s":self.get_clock().now().nanoseconds*1e-9,"map_point":point,"depth_m":distance,"pixels":count})
        self.goal_kind="marker"; self.current_goal=(point[0],point[1]); self.marker_goal_pub.publish(self._pose(point[0],point[1]))
        if hasattr(self,"_nav_handle"):self._nav_handle.cancel_goal_async()
        self._send_navigation(point[0],point[1])

    @staticmethod
    def _transform_point(point,t):
        q=t.rotation; x,y,z=point
        # Quaternion rotation through q*v*q^-1.
        uv=np.cross([q.x,q.y,q.z],[x,y,z]); uuv=np.cross([q.x,q.y,q.z],uv)
        rotated=np.asarray([x,y,z])+2*(q.w*uv+uuv)
        return float(rotated[0]+t.translation.x),float(rotated[1]+t.translation.y),float(rotated[2]+t.translation.z)
    def _pose(self,x,y):
        p=PoseStamped(); p.header.frame_id="map"; p.header.stamp=self.get_clock().now().to_msg(); p.pose.position.x=x; p.pose.position.y=y; p.pose.orientation.w=1.0; return p
    def _publish_status(self,status):
        values={"status":status,"loaded_map":False,"first_map_time_s":self.first_map_time,"known_cells":self.known_history[-1][1] if self.known_history else 0,"known_cell_growth":self.known_history,"frontier_events":self.frontier_events,"frontier_selections":sum(e.get("event")=="accepted" for e in self.frontier_events),"frontier_completions":sum(e.get("event")=="completed" for e in self.frontier_events),"rejected_candidates":sum(e.get("event")=="rejected" for e in self.frontier_events),"marker_detections":self.marker_events,"goal_kind":self.goal_kind,"current_goal":self.current_goal,"navigation_armed":self.armed}
        self.status_pub.publish(String(data=json.dumps(values,sort_keys=True)))
        if self.live:
            self.live.mkdir(parents=True,exist_ok=True); (self.live/"exploration_status.json").write_text(json.dumps(values,indent=2),encoding="utf-8")
    def _finish(self,status):
        if self.done:return
        self.done=True; self.arm_pub.publish(Bool(data=False)); self._publish_status(status)
        (self.out/"exploration_events.json").write_text(json.dumps({"frontiers":self.frontier_events,"markers":self.marker_events,"known_history":self.known_history},indent=2),encoding="utf-8")


def main():
    rclpy.init(); node=FrontierExplorer()
    try:
        while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:node._finish("INTERRUPTED")
    finally:node.destroy_node(); rclpy.shutdown()
    return 0
