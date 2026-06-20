"""Run and report the monitor-only Nav2 versus oracle evaluation."""

from __future__ import annotations

from html import escape
from pathlib import Path
import csv
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from maze.generator import generate_maze_from_config
from maze.grid import FREE, WALL, physical_cell_length_m, physical_cell_width_m
from nav.planner import plan_oracle_path
from sim.config import load_config
from sim.mujoco_runner import _write_png
from sim.world_builder import cell_to_world_xy


TERMINAL_MOTION = {"GOAL_REACHED", "ZERO_COMMAND_TIMEOUT", "TIMEOUT", "FALL_DETECTED", "FAILED"}


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _length(points: list[tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def align_commands(oracle, nav2, start: float, rate_hz: float = 10.0, freshness_s: float = 0.5):
    """Zero-order-hold both streams onto a shared simulation-time grid."""
    if not oracle or not nav2:
        return []
    end = min(oracle[-1][0], nav2[-1][0])
    if end <= start:
        return []
    ot = np.asarray([row[0] for row in oracle]); nt = np.asarray([row[0] for row in nav2])
    rows = []
    for current in np.arange(start, end + 1e-9, 1.0 / rate_hz):
        oi = int(np.searchsorted(ot, current, side="right") - 1)
        ni = int(np.searchsorted(nt, current, side="right") - 1)
        if oi < 0 or ni < 0 or current - ot[oi] > freshness_s or current - nt[ni] > freshness_s:
            continue
        rows.append((float(current), oracle[oi][1], oracle[oi][2], nav2[ni][1], nav2[ni][2]))
    return rows


def command_metrics(rows):
    def axis(index_oracle, index_nav2):
        if not rows:
            return {"mae": 0.0, "rmse": 0.0, "correlation": 0.0, "sign_agreement": 0.0}
        a = np.asarray([r[index_oracle] for r in rows], dtype=float)
        b = np.asarray([r[index_nav2] for r in rows], dtype=float)
        difference = b - a
        correlation = float(np.corrcoef(a, b)[0, 1]) if np.std(a) > 1e-9 and np.std(b) > 1e-9 else 0.0
        signs_a = np.where(np.abs(a) < 0.01, 0, np.sign(a)); signs_b = np.where(np.abs(b) < 0.01, 0, np.sign(b))
        return {"mae": float(np.mean(np.abs(difference))), "rmse": float(np.sqrt(np.mean(difference ** 2))),
                "correlation": correlation, "sign_agreement": float(np.mean(signs_a == signs_b))}
    return {"linear_x": axis(1, 3), "angular_z": axis(2, 4), "aligned_sample_count": len(rows)}


class Probe(Node):
    def __init__(self):
        super().__init__("nav2_probe")
        defaults = [("seed", 123), ("duration_s", 600.0), ("output_dir", "/workspace/runs/visual"),
                    ("live_visual_dir", ""), ("config_path", "/workspace/configs/default.yaml"),
                    ("corridor_width_m", 2.0), ("map_to_odom_path", ""), ("goal_map_to_odom_path", "")]
        for name, default in defaults:
            self.declare_parameter(name, default)
        self.seed = int(self.get_parameter("seed").value)
        self.duration = float(self.get_parameter("duration_s").value)
        self.out = Path(str(self.get_parameter("output_dir").value)); self.out.mkdir(parents=True, exist_ok=True)
        live = str(self.get_parameter("live_visual_dir").value); self.live = Path(live) if live else None
        self.config = load_config(str(self.get_parameter("config_path").value))
        corridor_width = float(self.get_parameter("corridor_width_m").value)
        self.config["maze"]["cell_size_m"] = corridor_width
        self.config["maze"]["cell_width_m"] = corridor_width
        self.config["maze"]["cell_length_m"] = corridor_width
        self.maze = generate_maze_from_config(self.config, self.seed)
        self.oracle_plan = plan_oracle_path(self.maze, safety_radius_m=float(self.config["robot"]["safety_radius_m"]), simplify=False)
        self.oracle_points = [cell_to_world_xy(self.maze, cell) for cell in self.oracle_plan.cells]
        self.tf_path = Path(str(self.get_parameter("map_to_odom_path").value))
        self.tf_values = json.loads(self.tf_path.read_text(encoding="utf-8")) if self.tf_path.is_file() else None
        goal_tf_path = Path(str(self.get_parameter("goal_map_to_odom_path").value))
        self.goal_tf_values = json.loads(goal_tf_path.read_text(encoding="utf-8")) if goal_tf_path.is_file() else None
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.lifecycle_states = {}
        self.lifecycle_clients = {name: self.create_client(GetState, f"/{name}/get_state") for name in
                                  ("map_server", "planner_server", "controller_server", "bt_navigator", "behavior_server")}
        self.global_map = self.local_map = self.saved_map = self.path = None
        self.plans = []; self.oracle_commands = []; self.nav_commands = []; self.actual = []
        self.goal_sent = self.goal_accepted = self.done = self.success = False
        self.goal_accepted_time = None; self.motion = {}; self.stop_reason = "duration_timeout"; self.finish_pending = False
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self._map, map_qos)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: self._cost(m, True), 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", lambda m: self._cost(m, False), 10)
        self.create_subscription(NavPath, "/plan", self._path, 20)
        self.create_subscription(Twist, "/cmd_vel", self._nav_command, 50)
        self.create_subscription(TwistStamped, "/oracle_cmd_vel", self._oracle_command, 50)
        self.create_subscription(Odometry, "/odom", self._odom, 50)
        self.create_subscription(String, "/mapping/status", self._motion, 20)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.goal_timer = self.create_timer(1.0, self._try_goal, clock=steady)
        self.plot_timer = self.create_timer(0.5, self._live_plot, clock=steady)
        self.watchdog = self.create_timer(max(30.0, self.duration * 2.0), self._finish, clock=steady)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _map(self, message): self.saved_map = message
    def _cost(self, message, global_map):
        if global_map: self.global_map = message
        else: self.local_map = message
    def _path(self, message):
        self.path = message
        self.plans.append((self._now(), [(p.pose.position.x, p.pose.position.y) for p in message.poses]))
    def _nav_command(self, message): self.nav_commands.append((self._now(), float(message.linear.x), float(message.angular.z)))
    def _oracle_command(self, message): self.oracle_commands.append((_stamp_seconds(message.header.stamp), float(message.twist.linear.x), float(message.twist.angular.z)))
    def _odom(self, message): self.actual.append((_stamp_seconds(message.header.stamp), float(message.pose.pose.position.x), float(message.pose.pose.position.y)))
    def _motion(self, message):
        try: self.motion = json.loads(message.data)
        except json.JSONDecodeError: self.motion = {"status": message.data}
        status = str(self.motion.get("status", ""))
        if status in TERMINAL_MOTION and not self.finish_pending:
            self.stop_reason = str(self.motion.get("stop_reason", status.lower()))
            self.finish_pending = True
            self.create_timer(1.0, self._finish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _check_lifecycle(self):
        for name, client in self.lifecycle_clients.items():
            if client.service_is_ready():
                future = client.call_async(GetState.Request())
                future.add_done_callback(lambda result, node_name=name: self._lifecycle_response(node_name, result))
    def _lifecycle_response(self, name, future):
        try: self.lifecycle_states[name] = future.result().current_state.label
        except Exception: self.lifecycle_states[name] = "unavailable"

    def _try_goal(self):
        self._check_lifecycle()
        if self.goal_sent or self.saved_map is None or not all(self.lifecycle_states.get(n) == "active" for n in self.lifecycle_clients): return
        if not self.action.wait_for_server(timeout_sec=0.05): return
        goal_world = cell_to_world_xy(self.maze, self.maze.spec.goal_cell)
        if self.goal_tf_values:
            t, q = self.goal_tf_values["translation"], self.goal_tf_values["rotation"]
            tx, ty = t["x"], t["y"]
            qx, qy, qz, qw = q["x"], q["y"], q["z"], q["w"]
        else:
            try: transform = self.tf.lookup_transform("map", "odom", rclpy.time.Time())
            except Exception: return
            q = transform.transform.rotation; tx, ty = transform.transform.translation.x, transform.transform.translation.y
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        gx = tx + cosine * goal_world[0] - sine * goal_world[1]
        gy = ty + sine * goal_world[0] + cosine * goal_world[1]
        goal = NavigateToPose.Goal(); goal.pose.header.frame_id = "map"; goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx; goal.pose.pose.position.y = gy; goal.pose.pose.orientation.w = 1.0
        self.goal_sent = True; self.goal_xy = (gx, gy)
        future = self.action.send_goal_async(goal); future.add_done_callback(self._goal_response)
    def _goal_response(self, future):
        try: self.goal_accepted = bool(future.result().accepted)
        except Exception: self.goal_accepted = False
        if self.goal_accepted: self.goal_accepted_time = self._now()

    def _live_plot(self):
        if not self.live: return
        self.live.mkdir(parents=True, exist_ok=True)
        rows = align_commands(self.oracle_commands, self.nav_commands, self.goal_accepted_time or self._now())
        self._write_command_svg(self.live / "command_comparison.svg", rows)

    def _write_command_svg(self, path, rows):
        width, height = 900, 420
        def poly(index, top, scale):
            if not rows: return ""
            start, span = rows[0][0], max(rows[-1][0] - rows[0][0], 0.1)
            return " ".join(f"{60 + 810 * (r[0] - start) / span:.1f},{top - r[index] * scale:.1f}" for r in rows)
        latest_o = self.oracle_commands[-1][1:] if self.oracle_commands else (0.0, 0.0)
        latest_n = self.nav_commands[-1][1:] if self.nav_commands else (0.0, 0.0)
        status = f"goal={'accepted' if self.goal_accepted else 'waiting'}  oracle=({latest_o[0]:.2f},{latest_o[1]:.2f})  nav2=({latest_n[0]:.2f},{latest_n[1]:.2f})  monitor_only"
        path.write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}"><rect width="100%" height="100%" fill="#111827"/><g stroke="#475569"><line x1="60" y1="130" x2="870" y2="130"/><line x1="60" y1="310" x2="870" y2="310"/></g><g fill="white" font-family="sans-serif"><text x="20" y="28">{escape(status)}</text><text x="20" y="95">linear.x</text><text x="20" y="275">angular.z</text><text x="650" y="400" fill="#22c55e">oracle</text><text x="740" y="400" fill="#38bdf8">Nav2</text></g><polyline points="{poly(1,130,90)}" fill="none" stroke="#22c55e" stroke-width="3"/><polyline points="{poly(3,130,90)}" fill="none" stroke="#38bdf8" stroke-width="3"/><polyline points="{poly(2,310,80)}" fill="none" stroke="#22c55e" stroke-width="3"/><polyline points="{poly(4,310,80)}" fill="none" stroke="#38bdf8" stroke-width="3"/></svg>\n''', encoding="utf-8")

    def _map_to_world(self, point):
        values = self.goal_tf_values or self.tf_values
        if not values: return point
        t, q = values["translation"], values["rotation"]
        yaw = math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))
        dx, dy = point[0] - t["x"], point[1] - t["y"]
        return (math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy)

    def _planner_metrics(self):
        latest_map = self.plans[-1][1] if self.plans else []
        latest = [self._map_to_world(point) for point in latest_map]
        valid = [] ; clearances = []
        cell_width = physical_cell_width_m(self.maze.spec)
        cell_length = physical_cell_length_m(self.maze.spec)
        wall_centers = [cell_to_world_xy(self.maze, (r, c)) for r in range(self.maze.spec.height_cells) for c in range(self.maze.spec.width_cells) if self.maze.grid[r, c] == WALL]
        for x, y in latest:
            col = round(x / cell_width + (self.maze.spec.width_cells - 1) / 2); row = round((self.maze.spec.height_cells - 1) / 2 - y / cell_length)
            valid.append(0 <= row < self.maze.spec.height_cells and 0 <= col < self.maze.spec.width_cells and self.maze.grid[row, col] == FREE)
            clearances.append(min((math.hypot(x-wx, y-wy) - min(cell_width, cell_length) / 2 for wx, wy in wall_centers), default=0.0))
        lengths = [_length([self._map_to_world(p) for p in points]) for _, points in self.plans]
        goal_error = math.hypot(latest[-1][0] - self.oracle_points[-1][0], latest[-1][1] - self.oracle_points[-1][1]) if latest else None
        return {"nav2_path_length_m": _length(latest), "oracle_path_length_m": _length(self.oracle_points),
                "path_length_ratio": _length(latest) / _length(self.oracle_points) if latest else 0.0,
                "free_space_valid_fraction": sum(valid) / len(valid) if valid else 0.0,
                "minimum_wall_clearance_m": min(clearances) if clearances else None, "replan_count": len(self.plans),
                "successive_plan_length_change_mean_m": float(np.mean(np.abs(np.diff(lengths)))) if len(lengths) > 1 else 0.0,
                "goal_endpoint_error_m": goal_error}, latest

    def _trajectory_svg(self, path, nav_points):
        size = 720; width = self.maze.spec.width_cells; height = self.maze.spec.height_cells
        cell_width = physical_cell_width_m(self.maze.spec); cell_length = physical_cell_length_m(self.maze.spec)
        def pixel(point): return ((point[0] / cell_width + (width - 1) / 2 + .5) * size / width, ((height - 1) / 2 - point[1] / cell_length + .5) * size / height)
        def line(points): return " ".join(f"{x:.1f},{y:.1f}" for x, y in map(pixel, points))
        walls = "".join(f'<rect x="{c*size/width:.1f}" y="{r*size/height:.1f}" width="{size/width:.1f}" height="{size/height:.1f}" fill="#111827"/>' for r in range(height) for c in range(width) if self.maze.grid[r,c] == WALL)
        actual = [(x, y) for _, x, y in self.actual]
        path.write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{size}"><rect width="100%" height="100%" fill="white"/>{walls}<polyline points="{line(self.oracle_points)}" fill="none" stroke="#22c55e" stroke-width="5"/><polyline points="{line(nav_points)}" fill="none" stroke="#38bdf8" stroke-width="5"/><polyline points="{line(actual)}" fill="none" stroke="#f97316" stroke-width="5"/><rect x="12" y="12" width="300" height="74" fill="white" opacity=".9"/><g font-family="sans-serif" font-size="16"><text x="25" y="34" fill="#22c55e">Oracle reference</text><text x="25" y="56" fill="#38bdf8">Nav2 latest plan</text><text x="25" y="78" fill="#f97316">Actual G1 trajectory</text></g></svg>\n''', encoding="utf-8")

    def _slam_truth(self, path):
        if self.saved_map is None: return
        slam = self._pixels(self.saved_map); truth = np.full_like(slam, 205)
        info = self.saved_map.info
        for display_row in range(info.height):
            grid_row = info.height - 1 - display_row
            for col in range(info.width):
                map_point = (info.origin.position.x + (col + .5) * info.resolution, info.origin.position.y + (grid_row + .5) * info.resolution)
                x, y = self._map_to_world(map_point)
                cell_width = physical_cell_width_m(self.maze.spec); cell_length = physical_cell_length_m(self.maze.spec)
                maze_col = round(x / cell_width + (self.maze.spec.width_cells - 1) / 2); maze_row = round((self.maze.spec.height_cells - 1) / 2 - y / cell_length)
                if 0 <= maze_row < self.maze.spec.height_cells and 0 <= maze_col < self.maze.spec.width_cells:
                    truth[display_row, col] = 0 if self.maze.grid[maze_row, maze_col] == WALL else 254
        gap = np.full((slam.shape[0], 12, 3), 100, np.uint8); self._png(path, np.concatenate([slam, gap, truth], axis=1))

    @staticmethod
    def _pixels(message):
        a=np.asarray(message.data,dtype=np.int16).reshape(message.info.height,message.info.width); g=np.full(a.shape,205,np.uint8); g[a==0]=254; g[a>=65]=0; g[(a>0)&(a<65)]=128
        return np.repeat(np.flipud(g)[:,:,None],3,axis=2)

    def _finish(self):
        if self.done: return
        self.done = True
        prefix = f"nav2_slam_seed-{self.seed}"
        paths = {key: self.out / f"{prefix}_{name}" for key, name in {"rviz":"rviz.png","costmaps":"costmaps.png","path":"path.svg","cmd":"cmd_vel.csv","dashboard":"dashboard.html","summary":"summary.json","comparison_csv":"command_comparison.csv","comparison_svg":"command_comparison.svg","slam_truth":"slam_vs_maze.png","trajectory":"trajectory_overlay.svg"}.items()}
        rows = align_commands(self.oracle_commands, self.nav_commands, self.goal_accepted_time or float("inf")); metrics = command_metrics(rows)
        comparison_start = self.goal_accepted_time or 0.0
        comparison_end = min(self.oracle_commands[-1][0], self.nav_commands[-1][0]) if self.oracle_commands and self.nav_commands else comparison_start
        possible_samples = max(0, int((comparison_end - comparison_start) * 10) + 1)
        metrics["coverage_fraction"] = len(rows) / max(1, possible_samples)
        with paths["comparison_csv"].open("w", newline="", encoding="utf-8") as output:
            writer=csv.writer(output); writer.writerow(["sim_time_s","oracle_linear_x","oracle_angular_z","nav2_linear_x","nav2_angular_z","linear_difference","angular_difference"])
            writer.writerows([(*r, r[3]-r[1], r[4]-r[2]) for r in rows])
        with paths["cmd"].open("w", newline="", encoding="utf-8") as output:
            writer=csv.writer(output); writer.writerow(["sim_time_s","linear_x","angular_z"]); writer.writerows(self.nav_commands)
        self._write_command_svg(paths["comparison_svg"], rows)
        planner, nav_world = self._planner_metrics(); self._trajectory_svg(paths["trajectory"], nav_world); self._slam_truth(paths["slam_truth"])
        if self.global_map is not None and self.local_map is not None:
            images=[self._pixels(self.global_map),self._pixels(self.local_map)]; h=max(i.shape[0] for i in images); padded=[]
            for image in images:
                p=np.full((h,image.shape[1],3),205,np.uint8); p[:image.shape[0]]=image; padded.append(p)
            self._png(paths["costmaps"],np.concatenate(padded,axis=1)); paths["rviz"].write_bytes(paths["costmaps"].read_bytes())
        paths["path"].write_text(paths["trajectory"].read_text(encoding="utf-8"),encoding="utf-8")
        lifecycle_active=all(self.lifecycle_states.get(n)=="active" for n in self.lifecycle_clients)
        finite=all(math.isfinite(v) for row in rows for v in row)
        self.success=bool(self.goal_accepted and lifecycle_active and len(self.plans)>0 and rows and finite and paths["slam_truth"].is_file() and paths["trajectory"].is_file() and self.motion.get("status") not in {"FALL_DETECTED","FAILED"})
        summary={"status":"completed" if self.success else "failed","phase":5,"seed":self.seed,"evaluation_mode":"two_stage_saved_map_shadow_control","cmd_vel_application":"monitor_only","replay_transform":"initial_slam_map_to_odom","goal_and_plan_evaluation_transform":"final_slam_map_to_odom","stop_reason":self.stop_reason,"motion":self.motion,"goal_sent":self.goal_sent,"goal_accepted":self.goal_accepted,"lifecycle_states":self.lifecycle_states,"lifecycle_servers_active":lifecycle_active,"command_comparison":metrics,"planner_evaluation":planner,"actual_trajectory_samples":len(self.actual),"rviz_capture_mode":"headless_equivalent","artifacts":{k:str(v) for k,v in paths.items()}}
        paths["summary"].write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
        panels=[("SLAM Map vs Ground-Truth Maze",paths["slam_truth"].name),("Oracle vs Nav2 Commands",paths["comparison_svg"].name),("Actual, Oracle and Nav2 Paths",paths["trajectory"].name)]
        html="".join(f'<section><h2>{escape(title)}</h2><img src="{source}"></section>' for title,source in panels)
        paths["dashboard"].write_text(f'<!doctype html><html><head><meta charset="utf-8"><style>body{{background:#111827;color:white;font-family:sans-serif}}main{{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:1rem}}section{{background:#1f2937;padding:1rem}}img{{width:100%}}</style></head><body><h1>Two-Stage Nav2 Evaluation — {summary["status"]}</h1><p>Nav2 remained monitor-only. Command metrics measure shadow-controller agreement; planner metrics evaluate the global path.</p><main>{html}</main><pre>{escape(json.dumps(summary,indent=2))}</pre></body></html>',encoding="utf-8")
        print(json.dumps(summary,indent=2),flush=True)
        self.watchdog.cancel()

    @staticmethod
    def _png(path,pixels):
        tmp=path.with_name(".tmp_"+path.name);_write_png(tmp,pixels);tmp.replace(path)


def main():
    rclpy.init(); node=Probe()
    try:
        while rclpy.ok() and not node.done: rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt: pass
    finally:
        success=node.success; node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0 if success else 1
