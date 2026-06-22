"""Publish MuJoCo D435i and G1 state through ROS 2 Humble messages."""

from __future__ import annotations

from pathlib import Path
from array import array
import math
import threading
import time
import copy
import json

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from maze.generator import generate_maze_from_config
from maze.validator import validate_maze
from sim.config import load_config
from sim.d435i_sensor import D435iSpec
from sim.mujoco_runner import import_mujoco
from sim.mujoco_runner import _write_png
from sim.oracle_motion_session import OracleMotionSession
from sim.nav2_motion_session import Nav2MotionSession
from sim.locomotion_policy_adapter import VelocityCommand
from sim.world_builder import build_maze_world, cell_to_world_xy


def _yaw_for_first_corridor(config: dict, seed: int) -> float:
    maze = generate_maze_from_config(config, seed)
    path = validate_maze(
        maze,
        safety_radius_m=float(config["robot"]["safety_radius_m"]),
        min_corridor_width_m=float(config["maze"]["min_corridor_width_m"]),
        max_corridor_width_m=(
            float(config["maze"]["max_corridor_width_m"])
            if "max_corridor_width_m" in config["maze"]
            else None
        ),
    ).path
    if len(path) < 2:
        return 0.0
    start = cell_to_world_xy(maze, path[0])
    following = cell_to_world_xy(maze, path[1])
    return math.atan2(following[1] - start[1], following[0] - start[0])


class G1MujocoBridge(Node):
    def __init__(self) -> None:
        super().__init__("g1_mujoco_bridge")
        self.declare_parameter("seed", 123)
        self.declare_parameter("config_path", "/workspace/configs/default.yaml")
        self.declare_parameter("output_dir", "/workspace/runs/visual")
        self.declare_parameter("clock_rate_hz", 100.0)
        self.declare_parameter("joint_tf_rate_hz", 50.0)
        self.declare_parameter("imu_rate_hz", 50.0)
        self.declare_parameter("camera_rate_hz", 10.0)
        self.declare_parameter("camera_enabled", True)
        self.declare_parameter("livox_mid360_enabled", False)
        self.declare_parameter("livox_scan_rate_hz", 10.0)
        self.declare_parameter("livox_horizontal_bins", 720)
        self.declare_parameter("livox_range_min_m", 0.10)
        self.declare_parameter("livox_range_max_m", 40.0)
        self.declare_parameter("livox_frame_id", "livox_mid360_frame")
        self.declare_parameter("livox_mount_pos_m", [0.05, 0.0, 0.29])
        self.declare_parameter("scan_command_guard_enabled", True)
        self.declare_parameter("scan_forward_stop_m", 0.90)
        self.declare_parameter("scan_reverse_stop_m", 0.70)
        self.declare_parameter("scan_side_stop_m", 0.45)
        self.declare_parameter("scan_guard_angle_deg", 35.0)
        self.declare_parameter("depth_only", False)
        self.declare_parameter("hold_pose", True)
        self.declare_parameter("live_visual_dir", "")
        self.declare_parameter("live_scan_panel", False)
        self.declare_parameter("live_map_panel", False)
        self.declare_parameter("live_nav_panel", False)
        self.declare_parameter("mujoco_viewer", False)
        self.declare_parameter("motion_mode", "hold")
        self.declare_parameter("motion_duration_s", 300.0)
        self.declare_parameter("corridor_width_m", 1.6)
        self.declare_parameter("publish_map_to_odom", True)
        self.declare_parameter("zero_command_timeout_s", 20.0)
        self.declare_parameter("focused_nav_visuals", False)
        self.declare_parameter("navigation_visuals", False)
        self.declare_parameter("unitree_rl_gym_repo", "/workspace/third_party/unitree_rl_gym")
        self.declare_parameter("locomotion_policy", "")
        self.declare_parameter("external_navigation_odom", False)
        self.declare_parameter("ground_truth_navigation_odom", False)
        self.declare_parameter("initial_spawn_yaw_rad", float("nan"))
        self.seed = int(self.get_parameter("seed").value)
        self.config = copy.deepcopy(load_config(str(self.get_parameter("config_path").value)))
        self.motion_mode = str(self.get_parameter("motion_mode").value)
        configured_policy = str(self.get_parameter("locomotion_policy").value).strip()
        if self.motion_mode == "oracle_mapping":
            self.locomotion_policy = configured_policy or "unitree_rl_gym_native"
        else:
            self.locomotion_policy = configured_policy or str(
                self.config.get("nav2_navigation", {}).get("locomotion_policy", "unitree_rl_gym_native")
            )
        if self.motion_mode in {"oracle_mapping", "nav2_navigation"}:
            corridor_width = float(self.get_parameter("corridor_width_m").value)
            self.config["maze"]["cell_size_m"] = corridor_width
            self.config["maze"]["cell_width_m"] = corridor_width
            self.config["maze"]["cell_length_m"] = corridor_width
        width_m = int(self.config["maze"]["width_cells"]) * float(self.config["maze"].get("cell_width_m", self.config["maze"]["cell_size_m"]))
        length_m = int(self.config["maze"]["height_cells"]) * float(self.config["maze"].get("cell_length_m", self.config["maze"]["cell_size_m"]))
        self.maze_extent_m = max(width_m, length_m)
        self.spec = D435iSpec.from_config(self.config)
        if self.spec is None:
            raise RuntimeError("D435i must be enabled for the ROS bridge")
        output_dir = Path(str(self.get_parameter("output_dir").value))
        self.world = build_maze_world(self.config, self.seed, output_dir)
        self.mujoco = import_mujoco()
        self.model = self.mujoco.MjModel.from_xml_path(self.world.model_xml_path)
        self.data = self.mujoco.MjData(self.model)
        self.data.qpos[:3] = np.asarray(self.world.start_world_xyz)
        configured_yaw=float(self.get_parameter("initial_spawn_yaw_rad").value)
        if not math.isfinite(configured_yaw):configured_yaw=float(self.config.get("nav2_navigation",{}).get("initial_spawn_yaw_rad",0.0))
        yaw = configured_yaw if self.motion_mode == "nav2_navigation" else _yaw_for_first_corridor(self.config, self.seed)
        self.data.qpos[3:7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        self.mujoco.mj_forward(self.model, self.data)
        self.held_qpos = self.data.qpos.copy()
        self.state_lock = threading.RLock()
        self.motion_session = None
        if self.motion_mode == "nav2_navigation" and bool(self.get_parameter("ground_truth_navigation_odom").value):
            raise RuntimeError("MuJoCo ground truth odometry is forbidden for Nav2 navigation; use sensor/SLAM odometry only.")
        if self.motion_mode == "oracle_mapping":
            self.motion_session = OracleMotionSession(
                self.mujoco, self.model, self.data, self.config, self.seed,
                Path(self.world.model_xml_path),
                float(self.get_parameter("motion_duration_s").value),
                float(self.get_parameter("zero_command_timeout_s").value),
                policy=self.locomotion_policy,
                unitree_rl_gym_repo=Path(str(self.get_parameter("unitree_rl_gym_repo").value)),
            )
        elif self.motion_mode == "nav2_navigation":
            start_x, start_y = float(self.world.start_world_xyz[0]), float(self.world.start_world_xyz[1])
            self.motion_session = Nav2MotionSession(
                self.mujoco, self.model, self.data, self.config,
                Path(self.world.model_xml_path),
                float(self.get_parameter("motion_duration_s").value), start_x, start_y, yaw,
                bool(self.get_parameter("ground_truth_navigation_odom").value),
                policy=self.locomotion_policy,
                unitree_rl_gym_repo=Path(str(self.get_parameter("unitree_rl_gym_repo").value)),
            )
        elif self.motion_mode != "hold":
            raise ValueError(f"unsupported motion_mode: {self.motion_mode}")

        self.clock_rate = float(self.get_parameter("clock_rate_hz").value)
        self.joint_every = self._period_ticks("joint_tf_rate_hz")
        self.imu_every = self._period_ticks("imu_rate_hz")
        self.camera_every = self._period_ticks("camera_rate_hz")
        self.camera_enabled = bool(self.get_parameter("camera_enabled").value)
        self.livox_enabled = bool(self.get_parameter("livox_mid360_enabled").value)
        self.livox_every = self._period_ticks("livox_scan_rate_hz") if self.livox_enabled else 1
        self.livox_bins = int(self.get_parameter("livox_horizontal_bins").value)
        self.livox_range_min = float(self.get_parameter("livox_range_min_m").value)
        self.livox_range_max = float(self.get_parameter("livox_range_max_m").value)
        self.livox_frame = str(self.get_parameter("livox_frame_id").value)
        self.livox_mount = np.asarray(self.get_parameter("livox_mount_pos_m").value, dtype=np.float64)
        self.livox_front_min = math.inf
        self.livox_rear_min = math.inf
        self.livox_left_min = math.inf
        self.livox_right_min = math.inf
        if self.livox_enabled:
            if self.livox_bins < 4:
                raise ValueError("livox_horizontal_bins must be at least 4")
            if self.livox_mount.shape != (3,):
                raise ValueError("livox_mount_pos_m must contain exactly three numbers")
            if not 0.0 <= self.livox_range_min < self.livox_range_max:
                raise ValueError("Livox range must satisfy 0 <= min < max")
        self.depth_only = bool(self.get_parameter("depth_only").value)
        self.hold_pose = bool(self.get_parameter("hold_pose").value)
        live_visual_dir = str(self.get_parameter("live_visual_dir").value)
        self.live_visual_dir = Path(live_visual_dir) if live_visual_dir else None
        self.live_scan_panel = bool(self.get_parameter("live_scan_panel").value)
        self.live_map_panel = bool(self.get_parameter("live_map_panel").value)
        self.live_nav_panel = bool(self.get_parameter("live_nav_panel").value)
        self.focused_nav_visuals = bool(self.get_parameter("focused_nav_visuals").value)
        self.navigation_visuals = bool(self.get_parameter("navigation_visuals").value)
        self.viewer_plan_goal_xy = None
        self.viewer_frontier_goal_xy = None
        self.viewer_marker_goal_xy = None
        self.viewer_fallback_goal_xy = None
        if self.live_visual_dir is not None:
            self._write_live_page()
        self.tick_count = 0
        self.mujoco_viewer = None
        self.mujoco_viewer_opened = False
        if bool(self.get_parameter("mujoco_viewer").value):
            try:
                import mujoco.viewer
            except Exception as exc:
                raise RuntimeError(f"MuJoCo viewer is unavailable in this environment: {exc}") from exc
            self.mujoco_viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._show_maze_geoms_in_viewer(self.mujoco_viewer)
            self.mujoco_viewer_opened = True
            self.get_logger().info("MuJoCo passive viewer opened")

        self.clock_pub = self.create_publisher(ClockMessage, "/clock", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", qos_profile_sensor_data)
        self.rgb_pub = self.create_publisher(Image, "/camera/color/image_raw", qos_profile_sensor_data)
        self.rgb_info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 10)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_rect_raw", qos_profile_sensor_data)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", 10)
        self.livox_pub = (
            self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
            if self.livox_enabled else None
        )
        self.motion_status_pub = self.create_publisher(String, "/mapping/status", 10)
        self.navigation_status_pub = self.create_publisher(String, "/navigation/status", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self.command_odom_pub = self.create_publisher(Odometry, "/debug/command_odom", 20)
        self.ground_truth_odom_pub = self.create_publisher(Odometry, "/ground_truth/odom", 20)
        self.oracle_cmd_pub = (
            self.create_publisher(TwistStamped, "/oracle_cmd_vel", 20)
            if self.motion_mode == "oracle_mapping" else None
        )
        self.applied_cmd_pub = self.create_publisher(TwistStamped, "/applied_cmd_vel", 20)
        self.achieved_velocity_pub = self.create_publisher(TwistStamped, "/ground_truth/achieved_velocity", 20)
        if self.motion_mode == "nav2_navigation":
            self.create_subscription(Twist, "/cmd_vel", self._nav2_command, 50)
            self.create_subscription(NavPath, "/plan", self._nav2_plan, 10)
            self.create_subscription(PoseStamped, "/exploration/frontier_goal", self._frontier_goal, 10)
            self.create_subscription(PoseStamped, "/exploration/marker_goal", self._marker_goal, 10)
            self.create_subscription(PoseStamped, "/exploration/fallback_goal", self._fallback_goal, 10)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / self.clock_rate, self._tick, clock=steady_clock)
        self.stop_camera = threading.Event()
        self.camera_thread = None
        if self.camera_enabled:
            camera_rate = float(self.get_parameter("camera_rate_hz").value)
            self.camera_period = 1.0 / camera_rate
            self.camera_thread = threading.Thread(
                target=self._camera_loop, name="d435i-renderer", daemon=True
            )
            self.camera_thread.start()
        self.get_logger().info(
            f"publishing seed {self.seed}: clock {self.clock_rate:g} Hz, "
            f"camera {'on' if self.camera_enabled else 'off'}, "
            f"Livox Mid-360 {'on' if self.livox_enabled else 'off'}"
        )

    def _period_ticks(self, parameter: str) -> int:
        rate = float(self.get_parameter(parameter).value)
        if rate <= 0.0 or rate > self.clock_rate:
            raise ValueError(f"{parameter} must be in (0, clock_rate_hz]")
        return max(1, round(self.clock_rate / rate))

    @staticmethod
    def _show_maze_geoms_in_viewer(viewer) -> None:
        # Maze floor/walls live in geom group 4 so the simulated Livox can raycast
        # just the environment. MuJoCo's passive viewer hides that group by default.
        if hasattr(viewer, "opt") and len(viewer.opt.geomgroup) > 4:
            viewer.opt.geomgroup[4] = 1

    def _nav2_plan(self, message: NavPath) -> None:
        if message.poses:
            pose = message.poses[-1].pose.position
            self.viewer_plan_goal_xy = (float(pose.x), float(pose.y))

    def _frontier_goal(self, message: PoseStamped) -> None:
        self.viewer_frontier_goal_xy = (float(message.pose.position.x), float(message.pose.position.y))

    def _marker_goal(self, message: PoseStamped) -> None:
        self.viewer_marker_goal_xy = (float(message.pose.position.x), float(message.pose.position.y))

    def _fallback_goal(self, message: PoseStamped) -> None:
        self.viewer_fallback_goal_xy = (float(message.pose.position.x), float(message.pose.position.y))

    def _update_mujoco_viewer_overlays(self) -> None:
        if self.mujoco_viewer is None:
            return
        user_scn = getattr(self.mujoco_viewer, "user_scn", None)
        if user_scn is None or not hasattr(user_scn, "geoms"):
            return
        try:
            with self.mujoco_viewer.lock():
                user_scn.ngeom = 0
                self._append_viewer_sphere(user_scn, self.world.goal_world_xyz[:2], 0.55, 0.34, (0.95, 0.10, 0.08, 0.95))
                self._append_viewer_sphere(user_scn, self.viewer_plan_goal_xy, 0.85, 0.26, (0.12, 0.62, 1.0, 0.95))
                self._append_viewer_sphere(user_scn, self.viewer_frontier_goal_xy, 1.15, 0.22, (0.96, 0.62, 0.04, 0.95))
                self._append_viewer_sphere(user_scn, self.viewer_marker_goal_xy, 1.45, 0.22, (0.95, 0.15, 0.15, 0.95))
                self._append_viewer_sphere(user_scn, self.viewer_fallback_goal_xy, 1.75, 0.22, (0.66, 0.33, 0.97, 0.95))
        except Exception as exc:
            self.get_logger().debug("Could not update MuJoCo viewer overlays: %s", exc)

    def _append_viewer_sphere(self, user_scn, xy, z: float, radius: float, rgba) -> None:
        if xy is None or user_scn.ngeom >= len(user_scn.geoms):
            return
        position = np.asarray([float(xy[0]), float(xy[1]), float(z)], dtype=np.float64)
        size = np.asarray([float(radius), 0.0, 0.0], dtype=np.float64)
        matrix = np.eye(3, dtype=np.float64).reshape(9)
        color = np.asarray(rgba, dtype=np.float32)
        self.mujoco.mjv_initGeom(
            user_scn.geoms[user_scn.ngeom],
            self.mujoco.mjtGeom.mjGEOM_SPHERE,
            size,
            position,
            matrix,
            color,
        )
        user_scn.ngeom += 1

    def _stamp(self) -> Time:
        seconds = float(self.data.time)
        whole = int(seconds)
        return Time(sec=whole, nanosec=int(round((seconds - whole) * 1_000_000_000)))

    def _tick(self) -> None:
        dt = 1.0 / self.clock_rate
        with self.state_lock:
            if self.motion_session is not None:
                if self.tick_count % max(1, round(self.clock_rate * self.motion_session.control_dt)) == 0:
                    self.motion_session.step()
            elif self.hold_pose:
                self.data.time += dt
            else:
                target = self.data.time + dt
                while self.data.time < target:
                    self.mujoco.mj_step(self.model, self.data)
        self.tick_count += 1
        stamp = self._stamp()
        self.clock_pub.publish(ClockMessage(clock=stamp))
        if self.motion_session is not None:
            summary = self.motion_session.summary()
            status_message = String(data=json.dumps(summary, sort_keys=True))
            if self.motion_mode == "oracle_mapping":
                self.motion_status_pub.publish(status_message)
            else:
                self.navigation_status_pub.publish(status_message)
            command = TwistStamped()
            command.header.stamp = stamp
            command.header.frame_id = "base_link"
            command.twist.linear.x = float(self.motion_session.last_command.vx)
            command.twist.angular.z = float(self.motion_session.last_command.yaw_rate)
            if self.motion_mode == "oracle_mapping":
                if self.oracle_cmd_pub is not None:
                    self.oracle_cmd_pub.publish(command)
            else:
                self.applied_cmd_pub.publish(command)
                achieved = TwistStamped()
                achieved.header.stamp = stamp
                achieved.header.frame_id = "base_link"
                achieved.twist.linear.x = float(self.motion_session.achieved.vx)
                achieved.twist.linear.y = float(self.motion_session.achieved.vy)
                achieved.twist.angular.z = float(self.motion_session.achieved.yaw_rate)
                self.achieved_velocity_pub.publish(achieved)
        if self.tick_count % self.joint_every == 0:
            self._publish_joints(stamp)
            self._publish_dynamic_tf(stamp)
            self._publish_odom(stamp)
            self._publish_ground_truth_odom(stamp)
        if self.tick_count % self.imu_every == 0:
            self._publish_imu(stamp)
        if self.livox_enabled and self.tick_count % self.livox_every == 0:
            self._publish_livox_scan(stamp)

    def _publish_livox_scan(self, stamp: Time) -> None:
        """Publish a planar navigation slice of the simulated Livox Mid-360."""
        pelvis_id = self._body_id("pelvis")
        pelvis_rotation = self.data.xmat[pelvis_id].reshape(3, 3)
        yaw = math.atan2(float(pelvis_rotation[1, 0]), float(pelvis_rotation[0, 0]))
        cy, sy = math.cos(yaw), math.sin(yaw)
        planar_rotation = np.asarray(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
        origin = self.data.xpos[pelvis_id] + planar_rotation @ self.livox_mount
        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / self.livox_bins
        geom_groups = np.asarray([0, 0, 0, 0, 1, 0], dtype=np.uint8)
        geom_id = np.asarray([-1], dtype=np.int32)
        ranges = np.full(self.livox_bins, np.inf, dtype=np.float32)
        for index in range(self.livox_bins):
            angle = angle_min + index * angle_increment + yaw
            direction = np.asarray((math.cos(angle), math.sin(angle), 0.0), dtype=np.float64)
            distance = float(
                self.mujoco.mj_ray(
                    self.model, self.data, origin, direction, geom_groups, 1, -1, geom_id
                )
            )
            if self.livox_range_min <= distance <= self.livox_range_max:
                ranges[index] = distance
        self._update_livox_guard_ranges(ranges, angle_min, angle_increment)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.livox_frame
        scan.angle_min = angle_min
        scan.angle_increment = angle_increment
        scan.angle_max = angle_min + (self.livox_bins - 1) * angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / float(self.get_parameter("livox_scan_rate_hz").value)
        scan.range_min = self.livox_range_min
        scan.range_max = self.livox_range_max
        scan.ranges = ranges.tolist()
        self.livox_pub.publish(scan)

    def _camera_loop(self) -> None:
        renderer = self.mujoco.Renderer(self.model, width=self.spec.width, height=self.spec.height)
        snapshot = self.mujoco.MjData(self.model)
        try:
            while not self.stop_camera.is_set():
                started = time.monotonic()
                try:
                    with self.state_lock:
                        self.mujoco.mj_copyData(snapshot, self.model, self.data)
                        stamp = self._stamp()
                    self._publish_cameras(stamp, renderer, snapshot)
                except Exception:
                    if self.stop_camera.is_set() or not rclpy.ok():
                        break
                    raise
                remaining = self.camera_period - (time.monotonic() - started)
                if remaining > 0.0:
                    self.stop_camera.wait(remaining)
        finally:
            renderer.close()

    def _nav2_command(self, message: Twist) -> None:
        with self.state_lock:
            if isinstance(self.motion_session, Nav2MotionSession):
                command = VelocityCommand(
                    vx=float(message.linear.x), vy=float(message.linear.y), yaw_rate=float(message.angular.z)
                )
                command = self._scan_guarded_command(command)
                self.motion_session.set_armed(True)
                self.motion_session.set_command(command)

    def _update_livox_guard_ranges(self, ranges: np.ndarray, angle_min: float, angle_increment: float) -> None:
        guard_angle = math.radians(float(self.get_parameter("scan_guard_angle_deg").value))
        angles = angle_min + np.arange(len(ranges), dtype=np.float64) * angle_increment
        finite = np.isfinite(ranges)
        front = finite & (np.abs(angles) <= guard_angle)
        rear = finite & (np.abs(np.abs(angles) - math.pi) <= guard_angle)
        left = finite & (np.abs(angles - math.pi / 2.0) <= guard_angle)
        right = finite & (np.abs(angles + math.pi / 2.0) <= guard_angle)
        self.livox_front_min = float(np.min(ranges[front])) if np.any(front) else math.inf
        self.livox_rear_min = float(np.min(ranges[rear])) if np.any(rear) else math.inf
        self.livox_left_min = float(np.min(ranges[left])) if np.any(left) else math.inf
        self.livox_right_min = float(np.min(ranges[right])) if np.any(right) else math.inf

    def _scan_guarded_command(self, command: VelocityCommand) -> VelocityCommand:
        if not (self.livox_enabled and bool(self.get_parameter("scan_command_guard_enabled").value)):
            return command
        vx = command.vx
        yaw_rate = command.yaw_rate
        if vx > 0.0 and self.livox_front_min < float(self.get_parameter("scan_forward_stop_m").value):
            vx = 0.0
        if vx < 0.0 and self.livox_rear_min < float(self.get_parameter("scan_reverse_stop_m").value):
            vx = 0.0
        side_stop = float(self.get_parameter("scan_side_stop_m").value)
        if yaw_rate > 0.0 and self.livox_left_min < side_stop:
            yaw_rate = 0.0
        if yaw_rate < 0.0 and self.livox_right_min < side_stop:
            yaw_rate = 0.0
        return VelocityCommand(vx=vx, vy=command.vy, yaw_rate=yaw_rate)

    def _publish_joints(self, stamp: Time) -> None:
        message = JointState()
        message.header.stamp = stamp
        message.header.frame_id = "base_link"
        for joint_id in range(self.model.njnt):
            joint_type = self.model.jnt_type[joint_id]
            if joint_type not in (
                self.mujoco.mjtJoint.mjJNT_HINGE,
                self.mujoco.mjtJoint.mjJNT_SLIDE,
            ):
                continue
            message.name.append(self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, joint_id))
            message.position.append(float(self.data.qpos[self.model.jnt_qposadr[joint_id]]))
            message.velocity.append(float(self.data.qvel[self.model.jnt_dofadr[joint_id]]))
            message.effort.append(0.0)
        self.joint_pub.publish(message)

    def _publish_imu(self, stamp: Time) -> None:
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = self.spec.imu_frame
        site_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_SITE, self.spec.imu_frame)
        quat = self._matrix_quaternion(self.data.site_xmat[site_id])
        message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z = quat
        gyro = self._sensor_values("d435i_angular_velocity")
        accel = self._sensor_values("d435i_linear_acceleration")
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = gyro
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = accel
        message.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        message.angular_velocity_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]
        message.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.imu_pub.publish(message)

    def _publish_odom(self, stamp: Time) -> None:
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        if isinstance(self.motion_session, Nav2MotionSession):
            x, y, yaw = self.motion_session.navigation_pose()
            message.pose.pose.position.x, message.pose.pose.position.y = x, y
            message.pose.pose.position.z = float(self.world.start_world_xyz[2])
            message.pose.pose.orientation.w, message.pose.pose.orientation.z = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
            message.twist.twist.linear.x = float(self.motion_session.last_command.vx)
            message.twist.twist.angular.z = float(self.motion_session.last_command.yaw_rate)
            message.pose.covariance[0] = message.pose.covariance[7] = 0.05
            message.pose.covariance[35] = 0.02
        else:
            pelvis = self._body_id("pelvis")
            message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = [float(v) for v in self.data.xpos[pelvis]]
            quat = self._matrix_quaternion(self.data.xmat[pelvis])
            message.pose.pose.orientation.w, message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z = quat
            message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = [float(v) for v in self.data.qvel[:3]]
            message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z = [float(v) for v in self.data.qvel[3:6]]
            message.pose.covariance[0] = message.pose.covariance[7] = 0.0025
            message.pose.covariance[35] = 0.005
        if isinstance(self.motion_session, Nav2MotionSession) and bool(self.get_parameter("external_navigation_odom").value):
            message.header.frame_id = "debug_command_odom"
            message.child_frame_id = "debug_command_base_link"
            self.command_odom_pub.publish(message)
        else:
            self.odom_pub.publish(message)

    def _publish_ground_truth_odom(self, stamp: Time) -> None:
        pelvis = self._body_id("pelvis")
        message = Odometry()
        message.header.stamp, message.header.frame_id, message.child_frame_id = stamp, "world", "ground_truth/base_link"
        message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = [float(v) for v in self.data.xpos[pelvis]]
        quat = self._matrix_quaternion(self.data.xmat[pelvis])
        message.pose.pose.orientation.w, message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z = quat
        message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = [float(v) for v in self.data.qvel[:3]]
        message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z = [float(v) for v in self.data.qvel[3:6]]
        self.ground_truth_odom_pub.publish(message)

    def _sensor_values(self, name: str) -> tuple[float, float, float]:
        sensor_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_SENSOR, name)
        address = self.model.sensor_adr[sensor_id]
        values = self.data.sensordata[address:address + 3]
        return tuple(float(value) for value in values)

    def _publish_cameras(self, stamp: Time, renderer, render_data) -> None:
        if not self.depth_only:
            renderer.update_scene(render_data, camera=self.spec.rgb_camera_name)
            rgb = renderer.render().copy()
            rgb_message = self._image(stamp, self.spec.rgb_optical_frame, "rgb8", rgb)
            self.rgb_pub.publish(rgb_message)
            self.rgb_info_pub.publish(self._camera_info(stamp, self.spec.rgb_optical_frame, self.spec.rgb_camera_name))

        renderer.enable_depth_rendering()
        renderer.update_scene(render_data, camera=self.spec.depth_camera_name)
        depth = renderer.render().copy().astype(np.float32)
        renderer.disable_depth_rendering()
        depth_message = self._image(stamp, self.spec.depth_optical_frame, "32FC1", depth)
        self.depth_pub.publish(depth_message)
        self.depth_info_pub.publish(self._camera_info(stamp, self.spec.depth_optical_frame, self.spec.depth_camera_name))
        if self.live_visual_dir is not None:
            if not self.depth_only:
                self._write_live_frame("camera_rgb.png", rgb)
                if self.navigation_visuals:
                    self._write_live_frame("marker_detection.png", rgb)
            depth_visual = self._depth_visual(depth)
            self._write_live_frame("camera_depth.png", depth_visual)
            robot_camera = self.mujoco.MjvCamera()
            robot_camera.type = self.mujoco.mjtCamera.mjCAMERA_FREE
            robot_camera.lookat[:] = render_data.xpos[self._body_id("pelvis")]
            robot_camera.distance = 5.5
            robot_camera.azimuth = 135.0
            robot_camera.elevation = -42.0
            renderer.update_scene(render_data, camera=robot_camera)
            robot_view = renderer.render().copy()
            self._write_live_frame("robot_maze.png", robot_view)
            if self.focused_nav_visuals:
                overhead = self.mujoco.MjvCamera()
                overhead.type = self.mujoco.mjtCamera.mjCAMERA_FREE
                overhead.lookat[:] = [0.0, 0.0, 0.0]
                overhead.distance = max(self.maze_extent_m * 1.15, 8.0)
                overhead.azimuth = 0.0
                overhead.elevation = -90.0
                renderer.update_scene(render_data, camera=overhead)
                self._write_live_frame("maze_overhead.png", renderer.render().copy())

    def _depth_visual(self, depth: np.ndarray) -> np.ndarray:
        minimum, maximum = self.spec.depth_visual_min_m, self.spec.depth_visual_max_m
        valid = np.isfinite(depth) & (depth > 0.0)
        clipped = np.clip(depth, minimum, maximum)
        gray = np.where(valid, 255.0 * (maximum - clipped) / (maximum - minimum), 0.0).astype(np.uint8)
        return np.repeat(gray[:, :, None], 3, axis=2)

    def _write_live_frame(self, filename: str, pixels: np.ndarray) -> None:
        path = self.live_visual_dir / filename
        temporary = path.with_name(f".{path.stem}.tmp.png")
        _write_png(temporary, pixels)
        temporary.replace(path)

    def _write_live_page(self) -> None:
        self.live_visual_dir.mkdir(parents=True, exist_ok=True)
        page = self.live_visual_dir / "ros_bridge_live.html"
        if self.navigation_visuals:
            placeholder = self.live_visual_dir / "command_comparison.svg"
            if not placeholder.exists():
                placeholder.write_text('<svg xmlns="http://www.w3.org/2000/svg" width="900" height="420"><rect width="100%" height="100%" fill="#111827"/><text x="30" y="60" fill="white" font-family="sans-serif" font-size="24">Waiting for Nav2 control…</text></svg>\n', encoding="utf-8")
            page.write_text("""<!doctype html><html lang="en"><head><meta charset="utf-8"><title>G1 Phase 6 Navigation</title>
<style>body{font-family:sans-serif;background:#0b1120;color:#e5e7eb;margin:1rem}header{display:flex;gap:.75rem;align-items:center}.live{color:#4ade80}main{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:1rem}section{background:#1f2937;padding:.75rem;border-radius:.5rem}img{width:100%;height:auto}@media(max-width:900px){main{grid-template-columns:1fr}}</style></head><body><header><h1>G1 Nav2-Controlled Navigation</h1><strong class="live">● LIVE</strong></header><main>
<section><h2>Robot in Maze</h2><img id="robot" src="robot_maze.png"></section>
<section><h2>RGB Camera + Goal Detection</h2><img id="rgb" src="marker_detection.png"></section>
<section><h2>SLAM Map, Goal and Nav2 Path</h2><img id="map" src="navigation_map.png"></section>
<section><h2>Raw, Applied and Achieved Commands</h2><img id="commands" src="command_comparison.svg"></section>
</main><script>setInterval(()=>{const t=Date.now();for(const [id,file] of [['robot','robot_maze.png'],['rgb','marker_detection.png'],['map','navigation_map.png'],['commands','command_comparison.svg']])document.getElementById(id).src=file+'?t='+t},500);</script></body></html>
""", encoding="utf-8")
            return
        if self.focused_nav_visuals:
            placeholder = self.live_visual_dir / "command_comparison.svg"
            if not placeholder.exists():
                placeholder.write_text('<svg xmlns="http://www.w3.org/2000/svg" width="900" height="420"><rect width="100%" height="100%" fill="#111827"/><text x="30" y="60" fill="white" font-family="sans-serif" font-size="24">Waiting for Nav2 command comparison…</text></svg>\n', encoding="utf-8")
            page.write_text("""<!doctype html><html lang="en"><head><meta charset="utf-8"><title>G1 Nav2 Evaluation</title>
<style>body{font-family:sans-serif;background:#0b1120;color:#e5e7eb;margin:1rem}header{display:flex;gap:.75rem;align-items:center}.live{color:#4ade80}main{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:1rem}section{background:#1f2937;padding:.75rem;border-radius:.5rem}img{width:100%;height:auto}@media(max-width:900px){main{grid-template-columns:1fr}}</style></head><body><header><h1>G1 Nav2 Shadow Evaluation</h1><strong class="live">● LIVE</strong></header><main>
<section><h2>Robot in Maze</h2><img id="robot" src="robot_maze.png"></section>
<section><h2>RGB Camera</h2><img id="rgb" src="camera_rgb.png"></section>
<section><h2>Maze — Bird's-eye View</h2><img id="overhead" src="maze_overhead.png"></section>
<section><h2>Nav2 Commands vs Oracle</h2><img id="commands" src="command_comparison.svg"></section>
</main><script>setInterval(()=>{const t=Date.now();for(const [id,file] of [['robot','robot_maze.png'],['rgb','camera_rgb.png'],['overhead','maze_overhead.png'],['commands','command_comparison.svg']])document.getElementById(id).src=file+'?t='+t},500);</script></body></html>\n""", encoding="utf-8")
            return
        document = """<!doctype html><html lang="en"><head><meta charset="utf-8"><title>G1 Live Sensor View</title>
<style>body{font-family:sans-serif;background:#0b1120;color:#e5e7eb;margin:1rem}header{display:flex;align-items:center;gap:.75rem}.live{color:#4ade80}main{display:grid;grid-template-columns:2fr 1fr;gap:1rem}.side{display:grid;gap:1rem}section{background:#1f2937;padding:.75rem;border-radius:.5rem}img{display:block;width:100%;height:auto}@media(max-width:900px){main{grid-template-columns:1fr}}</style></head>
<body><header><h1>G1 MuJoCo + D435i</h1><strong class="live">● LIVE</strong></header><main>
<section><h2>Robot in Maze</h2><img id="robot" src="robot_maze.png" alt="Robot in maze"></section><div class="side">
<section><h2>RGB Camera</h2><img id="rgb" src="camera_rgb.png" alt="RGB camera"></section>
<section><h2>Depth Camera</h2><img id="depth" src="camera_depth.png" alt="Depth camera"></section></div></main>
<script>setInterval(()=>{const t=Date.now();for(const [id,file] of [['robot','robot_maze.png'],['rgb','camera_rgb.png'],['depth','camera_depth.png']])document.getElementById(id).src=file+'?t='+t},350);</script></body></html>\n"""
        if self.live_scan_panel:
            document = document.replace(
                "</section></div></main>",
                "</section><section><h2>LaserScan Overlay</h2><img id=\"scan\" src=\"scan_overlay.png\" alt=\"LaserScan maze overlay\"></section></div></main>",
            ).replace(
                "['depth','camera_depth.png']])",
                "['depth','camera_depth.png'],['scan','scan_overlay.png']])",
            )
        if self.live_map_panel:
            document = document.replace(
                "</section></div></main>",
                "</section><section><h2>SLAM Occupancy Map</h2><img id=\"slam\" src=\"slam_map.png\" alt=\"SLAM occupancy map\"></section></div></main>",
            ).replace(
                "]])document.getElementById",
                ",['slam','slam_map.png']])document.getElementById",
            )
        if self.live_nav_panel:
            document = document.replace("</section></div></main>", "</section><section><h2>Nav2 Costmaps</h2><img id=\"costmaps\" src=\"nav2_costmaps.png\"></section><section><h2>Nav2 Path</h2><img id=\"navpath\" src=\"nav2_path.svg\"></section></div></main>").replace("]])document.getElementById", ",['costmaps','nav2_costmaps.png'],['navpath','nav2_path.svg']])document.getElementById")
        page.write_text(document, encoding="utf-8")

    @staticmethod
    def _image(stamp: Time, frame: str, encoding: str, pixels: np.ndarray) -> Image:
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = frame
        message.height, message.width = pixels.shape[:2]
        message.encoding = encoding
        message.is_bigendian = 0
        message.step = int(pixels.strides[0])
        message.data = array("B", pixels.tobytes())
        return message

    def _camera_info(self, stamp: Time, frame: str, camera_name: str) -> CameraInfo:
        metadata = self.spec.camera_metadata(camera_name)
        fx, fy = float(metadata["fx_px"]), float(metadata["fy_px"])
        cx, cy = float(metadata["cx_px"]), float(metadata["cy_px"])
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = frame
        message.width, message.height = self.spec.width, self.spec.height
        message.distortion_model = "plumb_bob"
        message.d = [0.0] * 5
        message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return message

    def _publish_static_tf(self) -> None:
        stamp = self._stamp()
        transforms = []
        if bool(self.get_parameter("publish_map_to_odom").value):
            transforms.append(self._transform(stamp, "map", "odom", np.zeros(3), np.eye(3)))
        if self.livox_enabled:
            transforms.append(
                self._transform(stamp, "base_link", self.livox_frame, self.livox_mount, np.eye(3))
            )
        parent_id = self._body_id_optional(self.spec.parent_body)
        if parent_id is None:
            parent_id = self._body_id("pelvis")
        parent_frame = self._body_frame(self.spec.parent_body)
        d435_id = self._body_id(self.spec.link_name)
        transforms.append(self._relative_body_transform(stamp, parent_id, d435_id, parent_frame, self.spec.link_name))
        transforms.append(self._transform(stamp, self.spec.link_name, self.spec.imu_frame, np.zeros(3), np.eye(3)))
        for camera_name, frame_name in (
            (self.spec.rgb_camera_name, self.spec.rgb_optical_frame),
            (self.spec.depth_camera_name, self.spec.depth_optical_frame),
        ):
            camera_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            body_rotation = self.data.xmat[d435_id].reshape(3, 3)
            camera_rotation = self.data.cam_xmat[camera_id].reshape(3, 3)
            optical_rotation = camera_rotation @ np.diag([1.0, -1.0, -1.0])
            relative_rotation = body_rotation.T @ optical_rotation
            relative_position = body_rotation.T @ (self.data.cam_xpos[camera_id] - self.data.xpos[d435_id])
            transforms.append(self._transform(stamp, self.spec.link_name, frame_name, relative_position, relative_rotation))
        self.static_tf_pub.sendTransform(transforms)

    def _publish_dynamic_tf(self, stamp: Time) -> None:
        pelvis_id = self._body_id("pelvis")
        if isinstance(self.motion_session, Nav2MotionSession):
            x, y, yaw = self.motion_session.navigation_pose()
            rotation = np.asarray([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
            base_transform = self._transform(stamp, "odom", "base_link", np.asarray([x, y, float(self.world.start_world_xyz[2])]), rotation)
        else:
            base_transform = self._transform(stamp, "odom", "base_link", self.data.xpos[pelvis_id], self.data.xmat[pelvis_id])
        transforms = []
        parent_id = self._body_id_optional(self.spec.parent_body)
        if parent_id is not None and self.spec.parent_body != "pelvis":
            transforms.append(self._relative_body_transform(stamp, pelvis_id, parent_id, "base_link", self._body_frame(self.spec.parent_body)))
        if not (isinstance(self.motion_session, Nav2MotionSession) and bool(self.get_parameter("external_navigation_odom").value)):
            transforms.insert(0, base_transform)
        if transforms:
            self.tf_pub.sendTransform(transforms)

    def _body_id(self, name: str) -> int:
        body_id = self._body_id_optional(name)
        if body_id is None:
            raise RuntimeError(f"MuJoCo body does not exist: {name}")
        return body_id

    def _body_id_optional(self, name: str) -> int | None:
        body_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_BODY, name)
        return None if body_id < 0 else int(body_id)

    @staticmethod
    def _body_frame(name: str) -> str:
        if name == "pelvis":
            return "base_link"
        if name == "torso_link_rev_1_0":
            return "torso_link"
        return name

    def _relative_body_transform(self, stamp, parent_id, child_id, parent_frame, child_frame):
        parent_rotation = self.data.xmat[parent_id].reshape(3, 3)
        child_rotation = self.data.xmat[child_id].reshape(3, 3)
        position = parent_rotation.T @ (self.data.xpos[child_id] - self.data.xpos[parent_id])
        rotation = parent_rotation.T @ child_rotation
        return self._transform(stamp, parent_frame, child_frame, position, rotation)

    def _matrix_quaternion(self, rotation) -> tuple[float, float, float, float]:
        output = np.zeros(4, dtype=np.float64)
        self.mujoco.mju_mat2Quat(output, np.asarray(rotation, dtype=np.float64).reshape(9))
        return tuple(float(value) for value in output)

    def _transform(self, stamp, parent, child, position, rotation) -> TransformStamped:
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x = float(position[0])
        message.transform.translation.y = float(position[1])
        message.transform.translation.z = float(position[2])
        quat = self._matrix_quaternion(rotation)
        message.transform.rotation.w = quat[0]
        message.transform.rotation.x = quat[1]
        message.transform.rotation.y = quat[2]
        message.transform.rotation.z = quat[3]
        return message

    def destroy_node(self):
        if self.mujoco_viewer is not None:
            self.mujoco_viewer.close()
            self.mujoco_viewer = None
        self.stop_camera.set()
        if self.camera_thread is not None:
            self.camera_thread.join(timeout=2.0)
        return super().destroy_node()


def main() -> int:
    rclpy.init()
    node = G1MujocoBridge()
    try:
        if node.mujoco_viewer is None:
            rclpy.spin(node)
        else:
            while rclpy.ok() and node.mujoco_viewer.is_running():
                rclpy.spin_once(node, timeout_sec=0.01)
                node._update_mujoco_viewer_overlays()
                node.mujoco_viewer.sync()
                if (
                    node.motion_session is not None
                    and node.motion_session.status not in {"WAITING_FOR_CMD", "RUNNING"}
                ):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
