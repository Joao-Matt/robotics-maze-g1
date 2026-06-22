"""Phase 6 goal sender, safety gate, evaluator, and visual artifact writer."""

from __future__ import annotations

import csv
from html import escape
import json
import math
from pathlib import Path
import shutil

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from maze.generator import generate_maze_from_config
from sim.config import load_config
from sim.mujoco_runner import _write_png
from sim.world_builder import cell_to_world_xy
from g1_nav_bringup.navigation_evaluation import farthest_reachable_free, path_is_known_free, stop_status


TERMINAL_MOTION = {"TIMEOUT", "STUCK", "FALL_DETECTED", "COLLISION_ABORT", "NAV2_ABORTED", "COMMAND_TIMEOUT"}


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class NavigationProbe(Node):
    def __init__(self) -> None:
        super().__init__("phase6_navigation_probe")
        for name, default in (("seed",123),("duration_s",1200.0),("output_dir","/workspace/runs/visual"),("live_visual_dir",""),("goal_source","map_inferred_exit"),("map_to_odom_path",""),("config_path","/workspace/configs/default.yaml")):
            self.declare_parameter(name, default)
        self.seed = int(self.get_parameter("seed").value)
        self.duration = float(self.get_parameter("duration_s").value)
        self.out = Path(str(self.get_parameter("output_dir").value)); self.out.mkdir(parents=True, exist_ok=True)
        live = str(self.get_parameter("live_visual_dir").value); self.live = Path(live) if live else None
        self.goal_source = str(self.get_parameter("goal_source").value)
        self.config = load_config(str(self.get_parameter("config_path").value))
        tf_path = Path(str(self.get_parameter("map_to_odom_path").value))
        self.map_to_odom = json.loads(tf_path.read_text(encoding="utf-8")) if tf_path.is_file() else None
        self.map = self.costmap = self.path = self.nav_odom = None
        self.raw_commands=[]; self.applied_commands=[]; self.achieved_commands=[]; self.actual=[]; self.paths=[]
        self.motion={}; self.lifecycle={}; self.goal_xy=None; self.goal_sent=False; self.goal_accepted=False
        self.nav_result=None; self.done=False; self.technical_success=False; self.armed=False; self.path_safe=False
        self.action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.lifecycle_clients={name:self.create_client(GetState,f"/{name}/get_state") for name in ("map_server","planner_server","controller_server","bt_navigator","behavior_server")}
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,"/map",self._map,qos)
        self.create_subscription(OccupancyGrid,"/global_costmap/costmap",lambda message:setattr(self,"costmap",message),10)
        self.create_subscription(NavPath,"/plan",self._path,20)
        self.create_subscription(Twist,"/cmd_vel",lambda m:self.raw_commands.append((self._now(),float(m.linear.x),float(m.angular.z))),50)
        self.create_subscription(TwistStamped,"/applied_cmd_vel",lambda m:self.applied_commands.append((stamp_seconds(m.header.stamp),float(m.twist.linear.x),float(m.twist.angular.z))),50)
        self.create_subscription(TwistStamped,"/ground_truth/achieved_velocity",lambda m:self.achieved_commands.append((stamp_seconds(m.header.stamp),float(m.twist.linear.x),float(m.twist.angular.z))),50)
        self.create_subscription(Odometry,"/odom",self._nav_odom,50)
        self.create_subscription(Odometry,"/ground_truth/odom",self._ground_truth,50)
        self.create_subscription(String,"/navigation/status",self._motion,20)
        self.arm_pub=self.create_publisher(Bool,"/navigation/armed",10)
        steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0,self._try_goal,clock=steady)
        self.create_timer(0.5,self._live_plot,clock=steady)
        self.watchdog=self.create_timer(max(30.0,self.duration*2.0),lambda:self._finish("wall_clock_watchdog"),clock=steady)

    def _now(self): return self.get_clock().now().nanoseconds*1e-9
    def _map(self,message):
        self.map=message
        if self.live:
            self.live.mkdir(parents=True,exist_ok=True)
            path=self.live/"navigation_map.png"; tmp=path.with_name(".tmp_"+path.name); _write_png(tmp,self._map_pixels()); tmp.replace(path)
    def _nav_odom(self,message): self.nav_odom=message
    def _ground_truth(self,message): self.actual.append((stamp_seconds(message.header.stamp),float(message.pose.pose.position.x),float(message.pose.pose.position.y)))
    def _path(self,message):
        self.path=message; points=[(p.pose.position.x,p.pose.position.y) for p in message.poses]; self.paths.append(points)
        source=self.costmap if self.costmap is not None else self.map
        if source is not None:
            info=source.info
            max_cost=252 if self.costmap is not None else 0
            self.path_safe=path_is_known_free(source.data,info.width,info.height,(info.origin.position.x,info.origin.position.y),info.resolution,points,max_cost=max_cost)
            if self.path_safe and not self.armed:
                self.armed=True; self.arm_pub.publish(Bool(data=True))
    def _motion(self,message):
        try: self.motion=json.loads(message.data)
        except json.JSONDecodeError: self.motion={"status":message.data}
        if self.motion.get("status") in TERMINAL_MOTION: self._finish(str(self.motion.get("stop_reason","motion_terminal")))

    def _check_lifecycle(self):
        for name,client in self.lifecycle_clients.items():
            if client.service_is_ready():
                future=client.call_async(GetState.Request()); future.add_done_callback(lambda result,node=name:self._lifecycle_response(node,result))
    def _lifecycle_response(self,name,future):
        try:self.lifecycle[name]=future.result().current_state.label
        except Exception:self.lifecycle[name]="unavailable"

    def _transform_odom_to_map(self,x,y):
        if not self.map_to_odom:return x,y
        t,q=self.map_to_odom["translation"],self.map_to_odom["rotation"]
        yaw=math.atan2(2*(q["w"]*q["z"]+q["x"]*q["y"]),1-2*(q["y"]**2+q["z"]**2))
        return t["x"]+math.cos(yaw)*x-math.sin(yaw)*y,t["y"]+math.sin(yaw)*x+math.cos(yaw)*y

    def _select_goal(self):
        if self.map is None or self.nav_odom is None:return None
        if self.goal_source == "oracle_debug":
            config=dict(self.config); config["maze"]=dict(config["maze"]); config["maze"]["cell_size_m"]=2.0
            config["maze"]["cell_width_m"]=2.0
            config["maze"]["cell_length_m"]=2.0
            maze=generate_maze_from_config(config,self.seed); return self._transform_odom_to_map(*cell_to_world_xy(maze,maze.spec.goal_cell))
        source=self.costmap if self.costmap is not None else self.map
        info=source.info
        start_map=self._transform_odom_to_map(self.nav_odom.pose.pose.position.x,self.nav_odom.pose.pose.position.y)
        start=(int((start_map[0]-info.origin.position.x)/info.resolution),int((start_map[1]-info.origin.position.y)/info.resolution))
        # Use a 1.5 m receding-horizon waypoint. The saved SLAM map is partial;
        # forcing a single plan across all known fragments is neither safe nor honest.
        horizon=max(5,int(round(1.5/info.resolution)))
        cell=farthest_reachable_free(source.data,info.width,info.height,start,max_distance=horizon)
        if cell is None:return None
        return info.origin.position.x+(cell[0]+.5)*info.resolution,info.origin.position.y+(cell[1]+.5)*info.resolution

    def _try_goal(self):
        if self.done:return
        self._check_lifecycle()
        if self.goal_sent or not all(self.lifecycle.get(name)=="active" for name in self.lifecycle_clients) or not self.action.wait_for_server(timeout_sec=.05):return
        goal_xy=self._select_goal()
        if goal_xy is None:return
        goal=NavigateToPose.Goal(); goal.pose.header.frame_id="map"; goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x,goal.pose.pose.position.y=goal_xy; goal.pose.pose.orientation.w=1.0
        self.goal_xy=goal_xy; self.goal_sent=True
        self.action.send_goal_async(goal).add_done_callback(self._goal_response)
    def _goal_response(self,future):
        try:handle=future.result(); self.goal_accepted=bool(handle.accepted)
        except Exception:self.goal_accepted=False; self._finish("goal_response_error"); return
        if not self.goal_accepted:self._finish("goal_rejected"); return
        handle.get_result_async().add_done_callback(self._goal_result)
    def _goal_result(self,future):
        try:status=future.result().status
        except Exception:status=-1
        self.nav_result={GoalStatus.STATUS_SUCCEEDED:"SUCCEEDED",GoalStatus.STATUS_ABORTED:"ABORTED",GoalStatus.STATUS_CANCELED:"CANCELED"}.get(status,f"STATUS_{status}")
        self._finish("nav2_action_complete")

    def _live_plot(self):
        if self.live is None:return
        self.live.mkdir(parents=True,exist_ok=True); self._command_svg(self.live/"command_comparison.svg")

    def _command_svg(self,path):
        series=[("Nav2 raw",self.raw_commands,"#38bdf8"),("Applied",self.applied_commands,"#f59e0b"),("Achieved (eval only)",self.achieved_commands,"#22c55e")]
        values=[row for _,rows,_ in series for row in rows]; start=min((r[0] for r in values),default=0); end=max((r[0] for r in values),default=start+1); span=max(end-start,.1)
        def poly(rows,index,mid,scale):return " ".join(f"{60+810*(r[0]-start)/span:.1f},{mid-r[index]*scale:.1f}" for r in rows)
        lines="".join(f'<polyline points="{poly(rows,1,135,180)}" fill="none" stroke="{color}" stroke-width="3"/><polyline points="{poly(rows,2,315,130)}" fill="none" stroke="{color}" stroke-width="3"/>' for _,rows,color in series)
        legend=" ".join(f'<tspan fill="{color}">{escape(name)}  </tspan>' for name,_,color in series)
        status=f"goal={self.goal_source} accepted={self.goal_accepted} armed={self.armed} path_safe={self.path_safe} status={self.motion.get('status','waiting')}"
        path.write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="900" height="420"><rect width="100%" height="100%" fill="#111827"/><g stroke="#475569"><line x1="60" y1="135" x2="870" y2="135"/><line x1="60" y1="315" x2="870" y2="315"/></g><g fill="white" font-family="sans-serif"><text x="20" y="28">{escape(status)}</text><text x="20" y="95">linear.x</text><text x="20" y="275">angular.z</text><text x="20" y="400">{legend}</text></g>{lines}</svg>\n',encoding="utf-8")

    def _trajectory_svg(self,path,nav_only=False):
        points=self.paths[-1] if self.paths else []; actual=[self._transform_odom_to_map(x,y) for _,x,y in self.actual]
        all_points=points+(actual if not nav_only else [])+([self.goal_xy] if self.goal_xy else [])
        if not all_points: all_points=[(0,0),(1,1)]
        xs=[p[0] for p in all_points]; ys=[p[1] for p in all_points]; xmin,xmax=min(xs)-.5,max(xs)+.5; ymin,ymax=min(ys)-.5,max(ys)+.5
        def pixel(p):return 40+640*(p[0]-xmin)/max(xmax-xmin,.1),680-640*(p[1]-ymin)/max(ymax-ymin,.1)
        def line(seq):return " ".join(f"{x:.1f},{y:.1f}" for x,y in map(pixel,seq))
        actual_line="" if nav_only else f'<polyline points="{line(actual)}" fill="none" stroke="#f97316" stroke-width="5"/>'
        path.write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="720" height="720"><rect width="100%" height="100%" fill="#f8fafc"/><polyline points="{line(points)}" fill="none" stroke="#38bdf8" stroke-width="5"/>{actual_line}<g font-family="sans-serif" font-size="18"><text x="20" y="25" fill="#38bdf8">Nav2 path</text><text x="150" y="25" fill="#f97316">Measured G1 trajectory</text></g></svg>\n',encoding="utf-8")

    def _map_pixels(self):
        if self.map is None:return np.full((480,640,3),205,np.uint8)
        a=np.asarray(self.map.data,dtype=np.int16).reshape(self.map.info.height,self.map.info.width); g=np.full(a.shape,205,np.uint8); g[a==0]=254; g[a>=65]=0
        return np.repeat(np.flipud(g)[:,:,None],3,axis=2)

    def _finish(self,reason):
        if self.done:return
        self.done=True; self.arm_pub.publish(Bool(data=False))
        prefix=f"navigate_d435i_seed-{self.seed}"
        paths={name:self.out/f"{prefix}_{suffix}" for name,suffix in {"rviz":"rviz.png","final":"mujoco_final.png","trajectory":"trajectory.svg","nav_path":"nav_path.svg","commands":"cmd_vel.csv","dashboard":"dashboard.html","summary":"summary.json"}.items()}
        with paths["commands"].open("w",newline="",encoding="utf-8") as output:
            writer=csv.writer(output); writer.writerow(["source","sim_time_s","linear_x","angular_z"])
            for name,rows in (("nav2_raw",self.raw_commands),("policy_applied",self.applied_commands),("achieved_ground_truth_eval_only",self.achieved_commands)):writer.writerows((name,*row) for row in rows)
        self._trajectory_svg(paths["trajectory"]); self._trajectory_svg(paths["nav_path"],True)
        command_plot=self.live/"command_comparison.svg" if self.live else self.out/".phase6_command_plot.svg"
        command_plot.parent.mkdir(parents=True,exist_ok=True); self._command_svg(command_plot)
        tmp=paths["rviz"].with_name(".tmp_"+paths["rviz"].name); _write_png(tmp,self._map_pixels()); tmp.replace(paths["rviz"])
        live_final=self.live/"robot_maze.png" if self.live else None
        if live_final and live_final.is_file():shutil.copyfile(live_final,paths["final"])
        else:
            tmp=paths["final"].with_name(".tmp_"+paths["final"].name); _write_png(tmp,np.full((480,640,3),40,np.uint8)); tmp.replace(paths["final"])
        actual_error=None
        if self.actual and self.goal_xy:
            point=self._transform_odom_to_map(self.actual[-1][1],self.actual[-1][2]); actual_error=math.hypot(point[0]-self.goal_xy[0],point[1]-self.goal_xy[1])
        final_status=stop_status(self.nav_result,str(self.motion.get("status","")),actual_error)
        lifecycle_active=all(self.lifecycle.get(name)=="active" for name in self.lifecycle_clients)
        nonzero_applied=any(abs(row[1])>0.001 or abs(row[2])>0.001 for row in self.applied_commands)
        self.technical_success=bool(self.goal_accepted and lifecycle_active and self.paths and self.path_safe and self.armed and self.raw_commands and nonzero_applied)
        summary={"status":"completed" if self.technical_success else "failed","phase":6,"seed":self.seed,"final_status":final_status,"stop_reason":reason,"slam_enabled":True,"slam_mode":"saved_slam_map","nav2_enabled":True,"sensor_source":"simulated_d435i_depth_to_laserscan","goal_source":self.goal_source,"ground_truth_used_for_navigation":self.goal_source=="oracle_debug","ground_truth_usage":"evaluation_and_emergency_safety_only" if self.goal_source!="oracle_debug" else "debug_goal_and_evaluation","cmd_vel_application":"unitree_rl_gym_native_policy","navigation_odometry_source":"applied_command_dead_reckoning","path_known_free":self.path_safe,"navigation_armed":self.armed,"goal_accepted":self.goal_accepted,"nav2_result":self.nav_result,"actual_goal_error_m":actual_error,"lifecycle_states":self.lifecycle,"motion":self.motion,"samples":{"raw_commands":len(self.raw_commands),"applied_commands":len(self.applied_commands),"achieved_velocities":len(self.achieved_commands),"trajectory":len(self.actual)},"rviz_capture_mode":"headless_equivalent","artifacts":{k:str(v) for k,v in paths.items()}}
        paths["summary"].write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
        panels=[("RViz Map / Scan Context",paths["rviz"].name),("Final MuJoCo View",paths["final"].name),("Actual Trajectory and Nav2 Path",paths["trajectory"].name),("Nav2 Path",paths["nav_path"].name)]
        html="".join(f'<section><h2>{title}</h2><img src="{source}"></section>' for title,source in panels)
        command_inline=command_plot.read_text(encoding="utf-8")
        html+=f'<section><h2>Raw, Applied and Achieved Commands</h2>{command_inline}</section>'
        paths["dashboard"].write_text(f'<!doctype html><html><head><meta charset="utf-8"><style>body{{background:#111827;color:white;font-family:sans-serif}}main{{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:1rem}}section{{background:#1f2937;padding:1rem}}img{{width:100%}}</style></head><body><h1>Phase 6 Navigation — {final_status}</h1><p>Nav2 commands were applied through the Unitree RL Gym native policy. Ground truth was evaluation-only.</p><main>{html}</main><pre>{escape(json.dumps(summary,indent=2))}</pre></body></html>',encoding="utf-8")
        print(json.dumps(summary,indent=2),flush=True); self.watchdog.cancel()


def main():
    rclpy.init(); node=NavigationProbe()
    try:
        while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:node._finish("keyboard_interrupt")
    finally:
        success=node.technical_success; node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if success else 1
