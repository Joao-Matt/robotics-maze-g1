"""Cold-start exploration artifact collector and evaluation-only reporter."""

from __future__ import annotations

import csv, json, math, time
from html import escape
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped
from explore_lite_msgs.msg import ExploreStatus
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String

from g1_nav_bringup.live_kpi_metrics import (
    localization_metrics as compute_localization_metrics,
    occupancy_stats,
    projected_free_space_coverage_stats,
)
from g1_nav_bringup.run_termination import is_run_success_status, is_terminal_run_status, run_duration_s, termination_source
from sim.mujoco_runner import _write_png
from sim.run_context import finalize_manifest
from sim.config import load_config
from sim.spawn_orientation import resolve_initial_spawn_yaw
from sim.world_builder import cell_to_world_xy
from maze.generator import generate_maze_from_config
from maze.grid import WALL, physical_cell_width_m, physical_cell_length_m
from nav.path_progress import path_progress_metrics
from nav.planner import PlanningError, plan_oracle_path


ACTIVE={
    "SELECTING_FRONTIER",
    "NAVIGATING",
    "exploration_started",
    "exploration_in_progress",
    "returning_to_origin",
    "FALLBACK_WAITING",
    "FALLBACK_WAITING_M_EXPLORE",
    "FALLBACK_OVERRIDING_M_EXPLORE",
    "FALLBACK_NAVIGATING",
}


def _stamp(stamp):return float(stamp.sec)+float(stamp.nanosec)*1e-9


class ExplorationReporter(Node):
    def __init__(self):
        super().__init__("exploration_reporter")
        for name,default in (("output_dir","/workspace/runs"),("live_visual_dir",""),("seed",123),("duration_s",1200.0),("corridor_width_m",2.0),("config_path","/workspace/configs/default.yaml"),("m_explore_complete_terminal",False),("goal_reached_tolerance_m",0.0),("goal_reached_tolerance_cell_width_fraction",1.0)):
            self.declare_parameter(name,default)
        self.out=Path(str(self.get_parameter("output_dir").value)); self.out.mkdir(parents=True,exist_ok=True)
        self.duration_s=run_duration_s(self.out,self.get_parameter("duration_s").value)
        live=str(self.get_parameter("live_visual_dir").value); self.live=Path(live) if live else None
        self.seed=int(self.get_parameter("seed").value); self.map=None; self.exploration={}; self.motion={}; self.odom_quality={}; self.done=False
        self.started_at=self._now(); self.clock_start_sim=None; self.clock_last_sim=None; self.clock_start_wall=None; self.first_map_time=None; self.known_cell_growth=[]
        config=load_config(str(self.get_parameter("config_path").value)); corridor_width=float(self.get_parameter("corridor_width_m").value); config["maze"]["cell_size_m"]=corridor_width; config["maze"]["cell_width_m"]=corridor_width; config["maze"]["cell_length_m"]=corridor_width
        requested_tolerance=float(self.get_parameter("goal_reached_tolerance_m").value); tolerance_fraction=float(self.get_parameter("goal_reached_tolerance_cell_width_fraction").value)
        self.goal_reached_tolerance_m=requested_tolerance if requested_tolerance>0.0 else corridor_width*tolerance_fraction
        self.goal_reached_tolerance_basis="absolute_m" if requested_tolerance>0.0 else "corridor_width_fraction"
        self.goal_reached_tolerance_cell_width_fraction=tolerance_fraction
        self.maze=generate_maze_from_config(config,self.seed); self.spawn_yaw=resolve_initial_spawn_yaw(config,self.seed); self.start_world=cell_to_world_xy(self.maze,self.maze.spec.start_cell); self.goal_world=cell_to_world_xy(self.maze,self.maze.spec.goal_cell); self.goal_estimated_xy=self._world_to_start_relative(self.goal_world)
        self.raw=[]; self.applied=[]; self.measured=[]; self.actual=[]; self.actual_odom=[]; self.odom=[]; self.odom_eval=[]; self.actual_eval=[]; self.paths=[]
        self.robot_goal_error_m=None; self.robot_goal_reached=False; self.robot_goal_reached_source=None; self.robot_estimated_pose_xy=None; self.robot_estimated_pose_source=None
        self.slam_pose_xy=None; self.slam_pose_goal_error_m=None; self.odom_pose_xy=None; self.odom_goal_error_m=None
        self.slam_pose_samples=0
        self.frontier_goal=self.marker_goal=self.fallback_goal=None; self.trajectory_pub=self.create_publisher(NavPath,"/exploration/trajectory",10); self.truth_trajectory_pub=self.create_publisher(NavPath,"/ground_truth/trajectory",10)
        self.create_subscription(ClockMessage,"/clock",self._clock_message,10)
        self.create_subscription(OccupancyGrid,"/map",self._map,10)
        self.create_subscription(Twist,"/cmd_vel",lambda m:self.raw.append((self._now(),m.linear.x,m.angular.z)),50)
        self.create_subscription(TwistStamped,"/applied_cmd_vel",lambda m:self.applied.append((_stamp(m.header.stamp),m.twist.linear.x,m.twist.angular.z)),50)
        self.create_subscription(Odometry,"/odom",self._odom,50)
        self.create_subscription(PoseWithCovarianceStamped,"/pose",self._slam_pose,50)
        self.create_subscription(TwistStamped,"/ground_truth/achieved_velocity",lambda m:self.measured.append((_stamp(m.header.stamp),m.twist.linear.x,m.twist.angular.z)),50)
        self.create_subscription(Odometry,"/ground_truth/odom",self._actual,50)
        self.create_subscription(NavPath,"/plan",lambda m:self.paths.append([(p.pose.position.x,p.pose.position.y) for p in m.poses]),20)
        self.create_subscription(PoseStamped,"/exploration/frontier_goal",lambda m:setattr(self,"frontier_goal",(m.pose.position.x,m.pose.position.y)),10)
        self.create_subscription(PoseStamped,"/exploration/marker_goal",lambda m:setattr(self,"marker_goal",(m.pose.position.x,m.pose.position.y)),10)
        self.create_subscription(PoseStamped,"/exploration/fallback_goal",lambda m:setattr(self,"fallback_goal",(m.pose.position.x,m.pose.position.y)),10)
        self.create_subscription(String,"/navigation/status",self._motion,20)
        self.create_subscription(String,"/exploration/status",self._exploration,20)
        self.create_subscription(ExploreStatus,"/explore/status",self._m_explore,20)
        self.create_subscription(String,"/odometry/d435i_status",self._odom_status,20)
        self.create_timer(.5,self._live_plot)
        self.create_timer(self.duration_s,lambda:self._finish("TIMEOUT"))
    def _now(self):return self.get_clock().now().nanoseconds*1e-9
    def _clock_message(self,msg):
        now=_stamp(msg.clock)
        if self.clock_start_sim is None:
            self.clock_start_sim=now; self.clock_start_wall=time.monotonic()
        self.clock_last_sim=now
    def _odom(self,msg):
        self.odom.append((_stamp(msg.header.stamp),msg.pose.pose.position.x,msg.pose.pose.position.y,msg.twist.twist.linear.x,msg.twist.twist.angular.z))
        self.odom_eval.append((_stamp(msg.header.stamp),msg.pose.pose.position.x,msg.pose.pose.position.y,self._yaw(msg.pose.pose.orientation)))
        self.odom_pose_xy=(float(msg.pose.pose.position.x),float(msg.pose.pose.position.y)); self.odom_goal_error_m=self._estimated_goal_error(self.odom_pose_xy)
        if self.slam_pose_xy is None:self._check_robot_goal_reached(self.odom_pose_xy,"odom")
        path=NavPath(); path.header=msg.header; path.header.frame_id="odom"
        for _,x,y,_,_ in self.odom[-2000:]:
            pose=PoseStamped(); pose.header=path.header; pose.pose.position.x=x; pose.pose.position.y=y; pose.pose.orientation.w=1.0; path.poses.append(pose)
        self.trajectory_pub.publish(path)
    def _actual(self,msg):
        t=_stamp(msg.header.stamp); x,y=msg.pose.pose.position.x,msg.pose.pose.position.y
        self.actual.append((t,x,y)); self.actual_eval.append((t,x,y,self._yaw(msg.pose.pose.orientation)))
        ox,oy=self._world_to_start_relative((x,y)); self.actual_odom.append((t,ox,oy))
        path=NavPath(); path.header=msg.header; path.header.frame_id="odom"
        for _,px,py in self.actual_odom[-2000:]:
            pose=PoseStamped(); pose.header=path.header; pose.pose.position.x=px; pose.pose.position.y=py; pose.pose.orientation.w=1.0; path.poses.append(pose)
        self.truth_trajectory_pub.publish(path)
    @staticmethod
    def _yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def _slam_pose(self,msg):
        p=msg.pose.pose.position; self.slam_pose_samples+=1; self.slam_pose_xy=(float(p.x),float(p.y)); self.slam_pose_goal_error_m=self._estimated_goal_error(self.slam_pose_xy); self._check_robot_goal_reached(self.slam_pose_xy,"slam_pose")
    def _estimated_goal_error(self,point):
        return math.hypot(float(point[0])-self.goal_estimated_xy[0],float(point[1])-self.goal_estimated_xy[1])
    def _check_robot_goal_reached(self,point,source):
        error=self._estimated_goal_error(point)
        self.robot_goal_error_m=error; self.robot_estimated_pose_xy=(float(point[0]),float(point[1])); self.robot_estimated_pose_source=source
        if not self.done and error<=self.goal_reached_tolerance_m:
            self.robot_goal_reached=True; self.robot_goal_reached_source=source; self._finish("GOAL_REACHED")
    def _world_to_start_relative(self,point):
        dx,dy=point[0]-self.start_world[0],point[1]-self.start_world[1]; c,s=math.cos(self.spawn_yaw),math.sin(self.spawn_yaw)
        return c*dx+s*dy,-s*dx+c*dy
    def _localization_metrics(self,csv_path):
        if not self.odom_eval or not self.actual_eval:return {"aligned_samples":0}
        actual=np.asarray(self.actual_eval,dtype=float); odom=np.asarray(self.odom_eval,dtype=float); rotation=actual[0,3]-odom[0,3]; c,s=math.cos(rotation),math.sin(rotation)
        predicted=np.column_stack((actual[0,1]+c*(odom[:,1]-odom[0,1])-s*(odom[:,2]-odom[0,2]),actual[0,2]+s*(odom[:,1]-odom[0,1])+c*(odom[:,2]-odom[0,2])))
        indices=np.searchsorted(actual[:,0],odom[:,0]); indices=np.clip(indices,0,len(actual)-1); previous=np.maximum(0,indices-1); choose_previous=np.abs(actual[previous,0]-odom[:,0])<np.abs(actual[indices,0]-odom[:,0]); indices=np.where(choose_previous,previous,indices)
        truth=actual[indices,1:3]; position=np.linalg.norm(predicted-truth,axis=1); predicted_yaw=odom[:,3]+rotation; yaw=np.abs(np.arctan2(np.sin(predicted_yaw-actual[indices,3]),np.cos(predicted_yaw-actual[indices,3])))
        with csv_path.open("w",newline="",encoding="utf-8") as output:
            writer=csv.writer(output); writer.writerow(["sim_time_s","odom_world_x","odom_world_y","truth_x","truth_y","x_error_m","y_error_m","position_error_m","odom_yaw_world_rad","truth_yaw_rad","yaw_error_rad"])
            writer.writerows((odom[i,0],predicted[i,0],predicted[i,1],truth[i,0],truth[i,1],predicted[i,0]-truth[i,0],predicted[i,1]-truth[i,1],position[i],predicted_yaw[i],actual[indices[i],3],yaw[i]) for i in range(len(odom)))
        metrics=compute_localization_metrics(list(self.odom_eval),list(self.actual_eval))
        if metrics.get("available"):return metrics
        odom_distance=float(np.linalg.norm(np.diff(predicted,axis=0),axis=1).sum()); truth_distance=float(np.linalg.norm(np.diff(truth,axis=0),axis=1).sum())
        return {"available":False,"aligned_samples":len(position),"position_rmse_m":float(np.sqrt(np.mean(position**2))),"position_mae_m":float(position.mean()),"position_p95_m":float(np.percentile(position,95)),"final_position_error_m":float(position[-1]),"yaw_rmse_rad":float(np.sqrt(np.mean(yaw**2))),"yaw_p95_rad":float(np.percentile(yaw,95)),"odom_distance_m":odom_distance,"truth_distance_m":truth_distance,"distance_scale":odom_distance/truth_distance if truth_distance>1e-6 else None}
    def _map(self,msg):
        self.map=msg
        known=int(np.count_nonzero(np.asarray(msg.data)>=0)); elapsed=self._now()-self.started_at
        if self.first_map_time is None:self.first_map_time=elapsed
        if not self.known_cell_growth or self.known_cell_growth[-1][1]!=known:self.known_cell_growth.append([elapsed,known])
        if self.live:
            self.live.mkdir(parents=True,exist_ok=True); _write_png(self.live/"navigation_map.png",self._map_pixels())
    def _live_plot(self):
        if self.live and not self.done:
            self.live.mkdir(parents=True,exist_ok=True); self._command_svg(self.live/"command_comparison.svg")
    def _motion(self,msg):
        try:self.motion=json.loads(msg.data)
        except Exception:self.motion={"status":msg.data}
        status=str(self.motion.get("status",""))
        if is_terminal_run_status(status):self._finish(status)
    def _odom_status(self,msg):
        try:self.odom_quality=json.loads(msg.data)
        except Exception:self.odom_quality={"status":msg.data}
    def _exploration(self,msg):
        try:self.exploration=json.loads(msg.data)
        except Exception:self.exploration={"status":msg.data}
        if is_terminal_run_status(self.exploration.get("status","")):self._finish(str(self.exploration.get("status","")))
    def _m_explore(self,msg):
        status=str(msg.status); self.exploration={"algorithm":"m-explore-ros2","status":status}
    def _map_pixels(self,overlay=True):
        if self.map is None:return np.full((480,640,3),205,np.uint8)
        a=np.asarray(self.map.data,dtype=np.int16).reshape(self.map.info.height,self.map.info.width); g=np.full(a.shape,205,np.uint8); g[a==0]=254; g[a>=65]=0; image=np.repeat(np.flipud(g)[:,:,None],3,axis=2)
        if not overlay:return image
        def pixel(point):
            col=round((point[0]-self.map.info.origin.position.x)/self.map.info.resolution); row=round((point[1]-self.map.info.origin.position.y)/self.map.info.resolution)
            return col,self.map.info.height-1-row
        def mark(point,color,radius=2):
            x,y=pixel(point)
            if 0<=x<image.shape[1] and 0<=y<image.shape[0]:image[max(0,y-radius):y+radius+1,max(0,x-radius):x+radius+1]=color
        for _,x,y,_,_ in self.odom:mark((x,y),(34,197,94),1)
        for point in (self.paths[-1] if self.paths else []):mark(point,(56,189,248),1)
        if self.frontier_goal:mark(self.frontier_goal,(245,158,11),3)
        if self.fallback_goal:mark(self.fallback_goal,(168,85,247),3)
        if self.marker_goal:mark(self.marker_goal,(239,68,68),3)
        return image
    def _truth_grid(self,width,height,resolution,origin_x,origin_y):
        """Rasterize the MuJoCo maze into an OccupancyGrid-shaped array.

        Rows use ROS OccupancyGrid order (bottom to top). Cells outside the
        generated maze are unknown so they cannot accidentally affect scoring.
        """
        rows,cols=np.indices((height,width),dtype=float)
        ox=origin_x+(cols+.5)*resolution; oy=origin_y+(rows+.5)*resolution
        c,s=math.cos(self.spawn_yaw),math.sin(self.spawn_yaw)
        wx=self.start_world[0]+c*ox-s*oy; wy=self.start_world[1]+s*ox+c*oy
        cell_w=physical_cell_width_m(self.maze.spec); cell_l=physical_cell_length_m(self.maze.spec)
        mc=np.floor(wx/cell_w+self.maze.spec.width_cells/2).astype(int)
        mr=np.floor(self.maze.spec.height_cells/2-wy/cell_l).astype(int)
        valid=(mr>=0)&(mr<self.maze.spec.height_cells)&(mc>=0)&(mc<self.maze.spec.width_cells)
        truth=np.full((height,width),-1,dtype=np.int16)
        truth[valid]=np.where(self.maze.grid[mr[valid],mc[valid]]==WALL,100,0)
        return truth
    @staticmethod
    def _occupancy_pixels(grid):
        gray=np.full(grid.shape,205,np.uint8); gray[grid==0]=254; gray[grid>=65]=0
        return np.repeat(np.flipud(gray)[:,:,None],3,axis=2)
    def _full_truth_grid(self,resolution):
        """Return the complete maze in the SLAM map frame and its origin."""
        half_w=self.maze.spec.width_cells*physical_cell_width_m(self.maze.spec)/2
        half_h=self.maze.spec.height_cells*physical_cell_length_m(self.maze.spec)/2
        world=np.asarray([[-half_w,-half_h],[-half_w,half_h],[half_w,-half_h],[half_w,half_h]])
        c,s=math.cos(self.spawn_yaw),math.sin(self.spawn_yaw)
        delta=world-np.asarray(self.start_world)
        map_xy=np.column_stack((c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1]))
        origin=np.floor(map_xy.min(axis=0)/resolution)*resolution
        upper=np.ceil(map_xy.max(axis=0)/resolution)*resolution
        width,height=np.maximum(1,np.ceil((upper-origin)/resolution).astype(int))
        return self._truth_grid(int(width),int(height),resolution,float(origin[0]),float(origin[1])),float(origin[0]),float(origin[1])
    def _trajectory_svg(self,path):
        actual=[(x,y) for _,x,y in self.actual_odom]; odom=[(x,y) for _,x,y,_,_ in self.odom]; plan=self.paths[-1] if self.paths else []
        points=actual+odom+plan or [(0,0),(1,1)]; xs=[p[0] for p in points]; ys=[p[1] for p in points]; xmin,xmax=min(xs)-.5,max(xs)+.5; ymin,ymax=min(ys)-.5,max(ys)+.5
        def line(seq):return " ".join(f"{40+640*(x-xmin)/max(.1,xmax-xmin):.1f},{680-640*(y-ymin)/max(.1,ymax-ymin):.1f}" for x,y in seq)
        path.write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="720" height="720"><rect width="100%" height="100%" fill="white"/><polyline points="{line(plan)}" fill="none" stroke="#38bdf8" stroke-width="4"/><polyline points="{line(odom)}" fill="none" stroke="#a855f7" stroke-width="4"/><polyline points="{line(actual)}" fill="none" stroke="#22c55e" stroke-width="4"/><g font-family="sans-serif"><text x="20" y="24" fill="#38bdf8">Nav2</text><text x="90" y="24" fill="#a855f7">Sensor odom</text><text x="220" y="24" fill="#22c55e">Actual ground truth</text></g></svg>\n',encoding="utf-8")
    def _slam_truth(self,path):
        """Write SLAM/truth/error panels and score only SLAM-observed cells."""
        if self.map is None:return {"evaluated_cells":0}
        info=self.map.info
        slam_grid=np.asarray(self.map.data,dtype=np.int16).reshape(info.height,info.width)
        truth_grid=self._truth_grid(info.width,info.height,info.resolution,info.origin.position.x,info.origin.position.y)
        known=(slam_grid>=0); valid=(truth_grid>=0); evaluated=known&valid
        slam_occupied=slam_grid>=65; truth_occupied=truth_grid>=65
        tp=int(np.count_nonzero(evaluated&slam_occupied&truth_occupied)); tn=int(np.count_nonzero(evaluated&~slam_occupied&~truth_occupied))
        fp=int(np.count_nonzero(evaluated&slam_occupied&~truth_occupied)); fn=int(np.count_nonzero(evaluated&~slam_occupied&truth_occupied))
        count=tp+tn+fp+fn

        # The truth panel deliberately hides unsearched cells. The error panel
        # uses green=agreement, red=false obstacle, orange=missed wall, gray=not evaluated.
        masked_truth=truth_grid.copy(); masked_truth[~known]=-1
        errors=np.full((info.height,info.width,3),(156,163,175),np.uint8)
        errors[evaluated]=(34,197,94); errors[evaluated&slam_occupied&~truth_occupied]=(239,68,68); errors[evaluated&~slam_occupied&truth_occupied]=(245,158,11)
        panels=(self._occupancy_pixels(slam_grid),self._occupancy_pixels(masked_truth),np.flipud(errors))
        gap=np.full((info.height,12,3),70,np.uint8); _write_png(path,np.concatenate((panels[0],gap,panels[1],gap,panels[2]),axis=1))
        truth_occ=tp+fn; predicted_occ=tp+fp; union=tp+fp+fn
        return {"evaluation_scope":"slam_known_cells_only","unknown_slam_cells_ignored":int(np.count_nonzero(~known)),"evaluated_cells":count,
                "true_occupied":tp,"true_free":tn,"false_occupied":fp,"missed_occupied":fn,
                "accuracy":(tp+tn)/count if count else None,"occupied_precision":tp/predicted_occ if predicted_occ else None,
                "occupied_recall":tp/truth_occ if truth_occ else None,"occupied_iou":tp/union if union else None}
    def _command_svg(self,path):
        sensor=[(t,vx,wz) for t,_,_,vx,wz in self.odom]
        series=[("Nav2",self.raw,"#38bdf8"),("Applied",self.applied,"#f59e0b"),("D435i odom",sensor,"#a855f7"),("Physical eval",self.measured,"#22c55e")]; values=[r for _,s,_ in series for r in s]; start=min((r[0] for r in values),default=0); end=max((r[0] for r in values),default=1); span=max(.1,end-start)
        def poly(rows,i,mid,scale):return " ".join(f"{50+820*(r[0]-start)/span:.1f},{mid-r[i]*scale:.1f}" for r in rows)
        lines="".join(f'<polyline points="{poly(rows,1,130,150)}" fill="none" stroke="{c}" stroke-width="2"/><polyline points="{poly(rows,2,310,100)}" fill="none" stroke="{c}" stroke-width="2"/>' for _,rows,c in series)
        legend="".join(f'<text x="{300+i*140}" y="25" fill="{color}">{name}</text>' for i,(name,_,color) in enumerate(series))
        path.write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="900" height="400"><rect width="100%" height="100%" fill="#111827"/><g fill="white"><text x="20" y="25">linear.x</text><text x="20" y="220">angular.z</text>{legend}</g>{lines}</svg>\n',encoding="utf-8")
    def _path_progress(self,distance_traveled):
        try:
            plan=plan_oracle_path(self.maze,simplify=False)
            path=[(x,y) for x,y,_ in plan.waypoints]
        except (PlanningError,ValueError,KeyError) as exc:
            metrics=path_progress_metrics(path_points=[],trajectory_points=[],distance_traveled_m=distance_traveled)
            metrics["path_progress_error"]=f"could not compute ground-truth path: {exc}"
            return metrics
        return path_progress_metrics(path_points=path,trajectory_points=[(x,y) for _,x,y in self.actual],distance_traveled_m=distance_traveled)
    def _finish(self,status):
        if self.done:return
        self.done=True
        if status=="RUNNING":status="INTERRUPTED"
        physical_goal_error=None
        maze_goal_reached=False
        robot_vs_ground_truth_pose_error=None
        goal=self.goal_world
        if self.actual:
            physical_goal_error=math.hypot(self.actual[-1][1]-goal[0],self.actual[-1][2]-goal[1])
            maze_goal_reached=physical_goal_error<=self.goal_reached_tolerance_m
            if self.robot_estimated_pose_xy is not None and self.actual_odom:
                truth_xy=(self.actual_odom[-1][1],self.actual_odom[-1][2])
                robot_vs_ground_truth_pose_error=math.hypot(self.robot_estimated_pose_xy[0]-truth_xy[0],self.robot_estimated_pose_xy[1]-truth_xy[1])
        paths={name:self.out/name for name in ("summary.json","dashboard.html","map.png","map.pgm","map.yaml","ground_truth_map.png","ground_truth_map.pgm","ground_truth_map.yaml","slam_vs_maze.png","trajectory.svg","command_timeline.svg","cmd_vel.csv","localization_comparison.csv")}
        pixels=self._map_pixels(); _write_png(paths["map.png"],pixels)
        gray=self._map_pixels(False)[:,:,0]; paths["map.pgm"].write_bytes(f"P5\n{gray.shape[1]} {gray.shape[0]}\n255\n".encode()+gray.tobytes())
        if self.map:
            paths["map.yaml"].write_text(f'image: map.pgm\nresolution: {self.map.info.resolution}\norigin: [{self.map.info.origin.position.x}, {self.map.info.origin.position.y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',encoding="utf-8")
        comparison=self._slam_truth(paths["slam_vs_maze.png"])
        if self.map:
            truth,truth_x,truth_y=self._full_truth_grid(self.map.info.resolution); truth_pixels=self._occupancy_pixels(truth); truth_gray=truth_pixels[:,:,0]
            _write_png(paths["ground_truth_map.png"],truth_pixels)
            paths["ground_truth_map.pgm"].write_bytes(f"P5\n{truth_gray.shape[1]} {truth_gray.shape[0]}\n255\n".encode()+truth_gray.tobytes())
            paths["ground_truth_map.yaml"].write_text(f"image: ground_truth_map.pgm\nresolution: {self.map.info.resolution}\norigin: [{truth_x}, {truth_y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",encoding="utf-8")
        self._trajectory_svg(paths["trajectory.svg"]); self._command_svg(paths["command_timeline.svg"])
        with paths["cmd_vel.csv"].open("w",newline="",encoding="utf-8") as f:
            w=csv.writer(f); w.writerow(["source","sim_time_s","linear_x","angular_z"])
            sensor=[(t,vx,wz) for t,_,_,vx,wz in self.odom]
            for name,rows in (("nav2",self.raw),("applied",self.applied),("d435i_odom",sensor),("ground_truth_eval",self.measured)):w.writerows((name,*r) for r in rows)
        known_start=0; known_end=int(sum(v>=0 for v in self.map.data)) if self.map else 0
        map_stats={"coverage_fraction":0.0}
        if self.map:
            raw_stats=occupancy_stats(self.map.data)
            truth_grid,truth_x,truth_y=self._full_truth_grid(self.map.info.resolution)
            map_stats={**raw_stats,"raw_known_coverage_fraction":raw_stats["coverage_fraction"],"raw_known_cells":raw_stats["known_cells"],"raw_total_cells":raw_stats["total_cells"],**projected_free_space_coverage_stats(self.map.data,slam_width=int(self.map.info.width),slam_height=int(self.map.info.height),slam_origin_x=float(self.map.info.origin.position.x),slam_origin_y=float(self.map.info.origin.position.y),truth_data=truth_grid.ravel().tolist(),truth_width=int(truth_grid.shape[1]),truth_height=int(truth_grid.shape[0]),truth_origin_x=truth_x,truth_origin_y=truth_y,resolution=float(self.map.info.resolution))}
        distance=sum(math.hypot(self.actual[i][1]-self.actual[i-1][1],self.actual[i][2]-self.actual[i-1][2]) for i in range(1,len(self.actual)))
        progress=self._path_progress(distance)
        localization=self._localization_metrics(paths["localization_comparison.csv"])
        completed=is_run_success_status(status)
        wall_elapsed=max(1e-9,time.monotonic()-(self.clock_start_wall or time.monotonic()))
        sim_elapsed=max(0.0,(self.clock_last_sim or self._now())-(self.clock_start_sim or 0.0))
        terminal_source=termination_source(status)
        artifacts={k:str(v) for k,v in paths.items()}
        live_dashboard=self.out/"live_dashboard"
        if live_dashboard.is_dir():
            artifacts["live_dashboard"]=str(live_dashboard)
            for name in ("kpis.latest.json","kpi_stream.ndjson","events.ndjson","timeseries_downsampled.csv","map_thumb.png"):
                path=live_dashboard/name
                if path.exists():artifacts[f"live_{path.stem.replace('.','_')}"]=str(path)
        summary={"status":"completed" if completed else "failed","phase":6,"seed":self.seed,"final_status":status,"termination_source":terminal_source,"timeout_limit_s":self.duration_s,"duration_limit_s":self.duration_s,"goal_source":"robot_estimated_pose_reaches_generated_maze_goal","generated_maze_goal_world_xy":[goal[0],goal[1]],"generated_maze_goal_estimated_xy":[self.goal_estimated_xy[0],self.goal_estimated_xy[1]],"goal_reached_tolerance_m":self.goal_reached_tolerance_m,"goal_reached_tolerance_basis":self.goal_reached_tolerance_basis,"goal_reached_tolerance_cell_width_fraction":self.goal_reached_tolerance_cell_width_fraction,"robot_goal_reached":self.robot_goal_reached,"robot_goal_reached_source":self.robot_goal_reached_source,"robot_estimated_pose_source":self.robot_estimated_pose_source,"robot_goal_error_m":self.robot_goal_error_m,"robot_estimated_pose_xy":list(self.robot_estimated_pose_xy) if self.robot_estimated_pose_xy is not None else None,"slam_pose_xy":list(self.slam_pose_xy) if self.slam_pose_xy is not None else None,"slam_pose_goal_error_m":self.slam_pose_goal_error_m,"odom_pose_xy":list(self.odom_pose_xy) if self.odom_pose_xy is not None else None,"odom_goal_error_m":self.odom_goal_error_m,"ground_truth_pose_estimated_frame_xy":[self.actual_odom[-1][1],self.actual_odom[-1][2]] if self.actual_odom else None,"robot_vs_ground_truth_pose_error_m":robot_vs_ground_truth_pose_error,"maze_goal_reached":maze_goal_reached,"physical_goal_error_m":physical_goal_error,"loaded_map":False,"slam_mode":"livox_mid360_slam_toolbox_with_sensor_odometry","ground_truth_used_for_navigation":False,"ground_truth_usage":"evaluation_and_comparison_only","odometry_source":"d435i_scan_odometry_livox_scan_imu_cmd_prediction","mapping_sensor":"simulated_livox_mid360_360_degree_laserscan","exploration_algorithm":"m-explore-ros2","odometry_quality":self.odom_quality or {"source":"d435i_scan_odometry","ground_truth":False},"sim_elapsed_s":sim_elapsed,"wall_elapsed_s":wall_elapsed,"realtime_factor":sim_elapsed/wall_elapsed,"known_cells_initial":known_start,"known_cells_final":known_end,"distance_traveled_m":distance,**progress,"exploration":self.exploration,"motion":self.motion,"samples":{"nav2_commands":len(self.raw),"applied_commands":len(self.applied),"navigation_odom":len(self.odom),"slam_pose_estimates":self.slam_pose_samples,"ground_truth_evaluation":len(self.actual)},"artifacts":artifacts}
        summary["mapping"]={**map_stats,"first_map_time_s":self.first_map_time,"known_cell_growth":self.known_cell_growth,"nav2_plan_updates":len(self.paths)}
        summary["mapping"]["ground_truth_comparison"]=comparison
        summary["localization_evaluation_ground_truth_only"]=localization
        paths["summary.json"].write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
        progress_panel=f'<section><h2>Path Progress</h2><pre>{escape(json.dumps({k:summary.get(k) for k in ("ground_truth_path_length_m","best_progress_along_path_m","final_progress_along_path_m","best_path_completion_fraction","final_path_completion_fraction","remaining_path_distance_m","path_efficiency","path_progress_warning","path_progress_error") if k in summary},indent=2))}</pre></section>'
        panels=progress_panel+"".join(f'<section><h2>{title}</h2><img src="{file}">{caption}</section>' for title,file,caption in (("Explored-area comparison","slam_vs_maze.png","<p>Left: SLAM. Middle: true occupancy only where SLAM observed. Right: green agreement, red false obstacle, orange missed wall, gray ignored/unsearched.</p>"),("Complete ground-truth occupancy grid","ground_truth_map.png","<p>Evaluation only; never supplied to navigation.</p>"),("Trajectories","trajectory.svg",""),("Commands","command_timeline.svg","")))
        paths["dashboard.html"].write_text(f'<!doctype html><html><head><meta charset="utf-8"><style>body{{background:#111827;color:white;font-family:sans-serif}}main{{display:grid;grid-template-columns:repeat(2,1fr);gap:1rem}}section{{background:#1f2937;padding:1rem}}img{{width:100%;image-rendering:auto}}</style></head><body><h1>m-explore + Livox SLAM — {escape(status)}</h1><p>Nav2 localization uses sensor-derived odometry and SLAM TF. MuJoCo ground truth is recorded only for offline comparison. Unsearched SLAM cells are excluded from comparison metrics.</p><main>{panels}</main><pre>{escape(json.dumps(summary,indent=2))}</pre></body></html>',encoding="utf-8")
        finalize_manifest(self.out,status,paths["summary.json"])
        print(json.dumps(summary,indent=2),flush=True)


def main():
    rclpy.init(); node=ExplorationReporter()
    try:
        while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:node._finish("INTERRUPTED")
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0
