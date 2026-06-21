"""Live sidecar KPI monitor for navigation runs.

The monitor subscribes to existing ROS topics, keeps bounded in-memory windows,
and serves a tiny browser dashboard over Server-Sent Events. It never feeds data
back into navigation.
"""

from __future__ import annotations

from collections import defaultdict, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import csv
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlparse

import numpy as np
import rclpy
from explore_lite_msgs.msg import ExploreStatus
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from g1_nav_bringup.live_kpi_metrics import (
    command_smoothness,
    drop_fraction,
    localization_metrics,
    message_rate,
    occupancy_stats,
    path_length,
    scan_clearance,
)
from maze.generator import generate_maze_from_config
from nav.planner import PlanningError, plan_oracle_path
from sim.config import load_config
from sim.mujoco_runner import _write_png
from sim.world_builder import cell_to_world_xy


def _stamp(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _rate_window() -> deque[float]:
    return deque(maxlen=1024)


class DashboardState:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.sequence = 0
        self.latest: dict[str, object] = {}
        self.closed = False

    def publish(self, snapshot: dict[str, object]) -> None:
        with self.condition:
            self.sequence += 1
            self.latest = dict(snapshot)
            self.latest["sequence"] = self.sequence
            self.condition.notify_all()

    def close(self) -> None:
        with self.condition:
            self.closed = True
            self.condition.notify_all()


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def make_handler(state: DashboardState, directory: Path):
    root = directory.resolve()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args) -> None:  # noqa: D401
            return

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/events":
                self._events()
                return
            name = parsed.path.lstrip("/") or "index.html"
            self._file(name)

        def _events(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            last_sequence = -1
            try:
                while True:
                    with state.condition:
                        state.condition.wait(timeout=15.0)
                        if state.closed:
                            return
                        sequence = state.sequence
                        snapshot = dict(state.latest)
                    if sequence != last_sequence and snapshot:
                        payload = json.dumps(snapshot, separators=(",", ":"), sort_keys=True)
                        self.wfile.write(f"id: {sequence}\nevent: kpis\ndata: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                        last_sequence = sequence
                    else:
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
            except (BrokenPipeError, ConnectionError, OSError):
                return

        def _file(self, name: str) -> None:
            candidate = (root / name).resolve()
            try:
                candidate.relative_to(root)
            except ValueError:
                self.send_error(403)
                return
            if not candidate.is_file():
                self.send_error(404)
                return
            data = candidate.read_bytes()
            content_type = "application/octet-stream"
            if candidate.suffix == ".html":
                content_type = "text/html; charset=utf-8"
            elif candidate.suffix == ".json":
                content_type = "application/json; charset=utf-8"
            elif candidate.suffix == ".ndjson":
                content_type = "application/x-ndjson; charset=utf-8"
            elif candidate.suffix == ".csv":
                content_type = "text/csv; charset=utf-8"
            elif candidate.suffix == ".png":
                content_type = "image/png"
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

    return Handler


class LiveKpiMonitor(Node):
    def __init__(self) -> None:
        super().__init__("live_kpi_monitor")
        for name, default in (
            ("output_dir", "/workspace/runs"),
            ("seed", 123),
            ("duration_s", 600.0),
            ("config_path", "/workspace/configs/default.yaml"),
            ("corridor_width_m", 2.0),
            ("dashboard_port", 8765),
            ("dashboard_bind_address", "127.0.0.1"),
            ("dashboard_rate_hz", 2.0),
            ("dashboard_visual_rate_hz", 1.0),
            ("evaluation_ground_truth_topic", "/ground_truth/odom"),
        ):
            self.declare_parameter(name, default)

        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.live_dir = self.output_dir / "live_dashboard"
        self.live_dir.mkdir(parents=True, exist_ok=True)
        self.state = DashboardState()
        self.started_wall = time.monotonic()
        self.seed = int(self.get_parameter("seed").value)
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.config = self._load_config()
        self.maze, self.goal_xy, self.optimal_path_m = self._maze_reference()
        self.expected_rates = self._expected_rates()

        self.topic_times: defaultdict[str, deque[float]] = defaultdict(_rate_window)
        self.topic_counts: defaultdict[str, int] = defaultdict(int)
        self.invalid_counts: defaultdict[str, int] = defaultdict(int)
        self.latest_stamps: dict[str, float] = {}
        self.clock_start_sim: float | None = None
        self.clock_last_sim: float | None = None
        self.clock_start_wall: float | None = None
        self._goal_reached_sim_s: float | None = None
        self.latest_map: OccupancyGrid | None = None
        self.latest_map_stats: dict[str, object] = {}
        self.first_map_wall: float | None = None
        self.latest_scan: dict[str, object] = {}
        self.motion: dict[str, object] = {}
        self.exploration: dict[str, object] = {}
        self.odom_quality: dict[str, object] = {}
        self.current_goals: dict[str, tuple[float, float]] = {}
        self.latest_plan: list[tuple[float, float]] = []
        self.raw_cmds: deque[tuple[float, float, float]] = deque(maxlen=600)
        self.applied_cmds: deque[tuple[float, float, float]] = deque(maxlen=600)
        self.achieved_cmds: deque[tuple[float, float, float]] = deque(maxlen=600)
        self.odom_eval: deque[tuple[float, float, float, float]] = deque(maxlen=1200)
        self.truth_eval: deque[tuple[float, float, float, float]] = deque(maxlen=1200)
        self.actual_xy: deque[tuple[float, float]] = deque(maxlen=5000)
        self._last_store_times: defaultdict[str, float] = defaultdict(lambda: -1e9)
        self._last_status: dict[str, str] = {}
        self._cpu_previous: tuple[float, int, int, int] | None = None
        self._http_server: ReusableThreadingHTTPServer | None = None
        self._http_thread: threading.Thread | None = None

        self.kpi_pub = self.create_publisher(String, "/dashboard/kpis", 10)
        self._write_index()
        self._start_http_server()
        self._subscribe()

        dashboard_rate = max(0.2, float(self.get_parameter("dashboard_rate_hz").value))
        visual_rate = float(self.get_parameter("dashboard_visual_rate_hz").value)
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._init_timeseries()
        self._publish_snapshot()
        self.create_timer(1.0 / dashboard_rate, self._publish_snapshot, clock=steady_clock)
        if visual_rate > 0.0:
            self.create_timer(1.0 / visual_rate, self._render_map_thumbnail, clock=steady_clock)

    def _load_config(self) -> dict:
        config = load_config(str(self.get_parameter("config_path").value))
        corridor_width = float(self.get_parameter("corridor_width_m").value)
        config["maze"]["cell_size_m"] = corridor_width
        config["maze"]["cell_width_m"] = corridor_width
        config["maze"]["cell_length_m"] = corridor_width
        return config

    def _maze_reference(self):
        maze = generate_maze_from_config(self.config, self.seed)
        goal = cell_to_world_xy(maze, maze.spec.goal_cell)
        try:
            plan = plan_oracle_path(maze, simplify=False)
            points = [(x, y) for x, y, _ in plan.waypoints]
            optimal = path_length(points)
        except (PlanningError, ValueError, KeyError) as exc:
            self.get_logger().warning(f"Could not compute oracle path length for live dashboard: {exc}")
            optimal = None
        return maze, goal, optimal

    def _expected_rates(self) -> dict[str, float | None]:
        scan_rate = float(self.config.get("livox_mid360", {}).get("scan_rate_hz", 10.0))
        state_rate = float(self.config.get("logging", {}).get("state_rate_hz", 50.0))
        return {
            "clock": 50.0,
            "scan": scan_rate,
            "imu": 50.0,
            "odom": scan_rate,
            "ground_truth": state_rate,
            "applied_cmd": state_rate,
            "achieved_velocity": state_rate,
            "cmd_vel": None,
            "map": None,
        }

    def _subscribe(self) -> None:
        self.create_subscription(ClockMessage, "/clock", self._clock_msg, 10)
        self.create_subscription(OccupancyGrid, "/map", self._map, 5)
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", self._imu, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._cmd, 20)
        self.create_subscription(TwistStamped, "/applied_cmd_vel", self._applied_cmd, 20)
        self.create_subscription(TwistStamped, "/ground_truth/achieved_velocity", self._achieved_cmd, 20)
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        truth_topic = str(self.get_parameter("evaluation_ground_truth_topic").value).strip()
        if truth_topic:
            self.create_subscription(Odometry, truth_topic, self._truth, 20)
        self.create_subscription(NavPath, "/plan", self._plan, 10)
        self.create_subscription(PoseStamped, "/exploration/frontier_goal", lambda m: self._goal("frontier", m), 10)
        self.create_subscription(PoseStamped, "/exploration/marker_goal", lambda m: self._goal("marker", m), 10)
        self.create_subscription(PoseStamped, "/exploration/fallback_goal", lambda m: self._goal("fallback", m), 10)
        self.create_subscription(String, "/navigation/status", lambda m: self._json_status("navigation", m), 20)
        self.create_subscription(String, "/exploration/status", lambda m: self._json_status("exploration", m), 20)
        self.create_subscription(String, "/odometry/d435i_status", lambda m: self._json_status("odometry", m), 20)
        self.create_subscription(ExploreStatus, "/explore/status", self._explore_status, 20)

    def _clock_msg(self, msg: ClockMessage) -> None:
        now = _stamp(msg.clock)
        self._seen("clock", now)
        if self.clock_start_sim is None:
            self.clock_start_sim = now
            self.clock_start_wall = time.monotonic()
        self.clock_last_sim = now

    def _map(self, msg: OccupancyGrid) -> None:
        self._seen("map", _stamp(msg.header.stamp))
        self.latest_map = msg
        self.latest_map_stats = occupancy_stats(msg.data)
        if self.first_map_wall is None:
            self.first_map_wall = time.monotonic()

    def _scan(self, msg: LaserScan) -> None:
        values = scan_clearance(msg.ranges, msg.range_min, msg.range_max)
        self.latest_scan = values
        self._seen("scan", _stamp(msg.header.stamp), int(values.get("invalid_ranges") or 0))

    def _imu(self, msg: Imu) -> None:
        values = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        invalid = sum(0 if math.isfinite(float(value)) else 1 for value in values)
        self._seen("imu", _stamp(msg.header.stamp), invalid)

    def _cmd(self, msg: Twist) -> None:
        self._seen("cmd_vel")
        self._store_command("raw", self.raw_cmds, time.monotonic(), msg.linear.x, msg.angular.z)

    def _applied_cmd(self, msg: TwistStamped) -> None:
        t = _stamp(msg.header.stamp)
        self._seen("applied_cmd", t)
        self._store_command("applied", self.applied_cmds, t, msg.twist.linear.x, msg.twist.angular.z)

    def _achieved_cmd(self, msg: TwistStamped) -> None:
        t = _stamp(msg.header.stamp)
        self._seen("achieved_velocity", t)
        self._store_command("achieved", self.achieved_cmds, t, msg.twist.linear.x, msg.twist.angular.z)

    def _odom(self, msg: Odometry) -> None:
        t = _stamp(msg.header.stamp)
        self._seen("odom", t)
        if t - self._last_store_times["odom_eval"] >= 0.1:
            self._last_store_times["odom_eval"] = t
            self.odom_eval.append((t, float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), _yaw(msg.pose.pose.orientation)))

    def _truth(self, msg: Odometry) -> None:
        t = _stamp(msg.header.stamp)
        self._seen("ground_truth", t)
        x, y = float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)
        if t - self._last_store_times["truth_eval"] >= 0.1:
            self._last_store_times["truth_eval"] = t
            self.truth_eval.append((t, x, y, _yaw(msg.pose.pose.orientation)))
            self.actual_xy.append((x, y))

    def _plan(self, msg: NavPath) -> None:
        self._seen("plan", _stamp(msg.header.stamp))
        self.latest_plan = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]

    def _goal(self, name: str, msg: PoseStamped) -> None:
        self._seen(f"{name}_goal", _stamp(msg.header.stamp))
        self.current_goals[name] = (float(msg.pose.position.x), float(msg.pose.position.y))
        self._event("goal_update", {"kind": name, "xy": self.current_goals[name]})

    def _json_status(self, source: str, msg: String) -> None:
        self._seen(f"{source}_status")
        try:
            values = json.loads(msg.data)
        except Exception:
            values = {"status": msg.data}
        if source == "navigation":
            self.motion = values
        elif source == "exploration":
            self.exploration = values
        else:
            self.odom_quality = values
        status = str(values.get("status", ""))
        if status and self._last_status.get(source) != status:
            self._last_status[source] = status
            self._event("status_change", {"source": source, "status": status})

    def _explore_status(self, msg: ExploreStatus) -> None:
        self._seen("m_explore_status")
        status = str(msg.status)
        self.exploration = {"algorithm": "m-explore-ros2", "status": status}
        if self._last_status.get("m_explore") != status:
            self._last_status["m_explore"] = status
            self._event("status_change", {"source": "m_explore", "status": status})

    def _store_command(self, key: str, rows: deque[tuple[float, float, float]], t: float, linear_x: float, angular_z: float) -> None:
        invalid = 0 if math.isfinite(float(linear_x)) and math.isfinite(float(angular_z)) else 1
        self.invalid_counts[key] += invalid
        if t - self._last_store_times[key] >= 0.1:
            self._last_store_times[key] = t
            rows.append((float(t), float(linear_x), float(angular_z)))

    def _seen(self, topic: str, stamp: float | None = None, invalid: int = 0) -> None:
        self.topic_times[topic].append(time.monotonic())
        self.topic_counts[topic] += 1
        self.invalid_counts[topic] += int(invalid)
        if stamp is not None and math.isfinite(float(stamp)):
            self.latest_stamps[topic] = float(stamp)

    def _publish_snapshot(self) -> None:
        snapshot = self._snapshot()
        payload = json.dumps(snapshot, indent=2, sort_keys=True) + "\n"
        self._write_atomic(self.live_dir / "kpis.latest.json", payload.encode("utf-8"))
        with (self.live_dir / "kpi_stream.ndjson").open("a", encoding="utf-8") as output:
            output.write(json.dumps(snapshot, sort_keys=True) + "\n")
        self._append_timeseries(snapshot)
        compact = json.dumps(snapshot, separators=(",", ":"), sort_keys=True)
        self.kpi_pub.publish(String(data=compact))
        self.state.publish(snapshot)

    def _snapshot(self) -> dict[str, object]:
        now_wall = time.monotonic()
        sim_elapsed = self._sim_elapsed()
        wall_elapsed = now_wall - self.started_wall
        distance = self._distance_traveled()
        goal_error = self._goal_error()
        path_efficiency = distance / self.optimal_path_m if distance is not None and self.optimal_path_m and self.optimal_path_m > 1e-6 else None
        localization = localization_metrics(list(self.odom_eval), list(self.truth_eval))
        smoothness = command_smoothness(list(self.applied_cmds))
        topic_health = self._topic_health(now_wall)
        map_stats = dict(self.latest_map_stats)
        status = str(self.motion.get("status") or self.exploration.get("status") or "RUNNING")
        wall_contacts = self._wall_contact_count()
        recovery_events = self.motion.get("recovery_events", [])
        stuck_events = len(recovery_events) if isinstance(recovery_events, list) else 0
        realtime_factor = sim_elapsed / wall_elapsed if wall_elapsed > 1e-9 else None
        solve_status = self._demo_solve_status(status, goal_error, bool(self.motion or self.exploration or self.actual_xy))
        if solve_status == "yes" and self._goal_reached_sim_s is None:
            self._goal_reached_sim_s = sim_elapsed
        time_to_goal_s = self._goal_reached_sim_s if self._goal_reached_sim_s is not None else sim_elapsed
        time_to_goal_status = "final" if solve_status == "yes" else ("no goal" if solve_status == "no" else solve_status)
        capture_health = self._capture_health(topic_health)
        snapshot = {
            "schema_version": 1,
            "generated_at_wall_s": time.time(),
            "run": self._run_metadata(),
            "demo": {
                "solve": solve_status,
                "time_to_goal_s": time_to_goal_s,
                "time_to_goal_status": time_to_goal_status,
                "collisions_stuck": {
                    "wall_collisions": wall_contacts,
                    "stuck_events": stuck_events,
                    "summary": f"{wall_contacts} collisions, {stuck_events} stuck/recoveries",
                },
                "capture_health": capture_health,
            },
            "status": {
                "mission_status": status,
                "navigation": self.motion,
                "exploration": self.exploration,
                "odometry_quality": self.odom_quality,
            },
            "success": {
                "held_out_solve_rate": None,
                "held_out_solve_rate_note": "aggregate-only: collect N>=20 seed summaries",
                "single_run_goal_reached": bool(goal_error is not None and goal_error <= 0.5),
                "single_run_terminal_status": status,
            },
            "mission": {
                "sim_elapsed_s": sim_elapsed,
                "wall_elapsed_s": wall_elapsed,
                "time_remaining_s": max(0.0, self.duration_s - sim_elapsed) if self.duration_s > 0 else None,
                "distance_traveled_m": distance,
                "optimal_path_m": self.optimal_path_m,
                "path_efficiency": path_efficiency,
                "final_goal_error_m": goal_error,
                "latest_plan_length_m": path_length(self.latest_plan),
                "current_goals": self.current_goals,
            },
            "safety_motion": {
                "wall_collisions": wall_contacts,
                "min_wall_clearance_m": self.latest_scan.get("min_clearance_m"),
                "median_scan_clearance_m": self.latest_scan.get("median_clearance_m"),
                "stuck_events_and_recoveries": stuck_events,
                "smoothness": smoothness,
                "max_contact_force_n": self.motion.get("max_contact_force_n"),
            },
            "localization": localization,
            "mapping": {
                **map_stats,
                "first_map_time_wall_s": None if self.first_map_wall is None else self.first_map_wall - self.started_wall,
                "map_update_rate_hz": topic_health.get("map", {}).get("rate_hz"),
                "nav2_plan_updates": self.topic_counts.get("plan", 0),
            },
            "data_quality": {
                "topics": topic_health,
                "inter_sensor_sync_error_ms": self._sensor_sync_error_ms(),
                "schema_completeness_fraction": self._schema_completeness(),
                "nan_or_invalid_counts": dict(self.invalid_counts),
                "process": self._process_stats(),
                "realtime_factor": realtime_factor,
            },
            "reliability": {
                "mtbf_m_per_failure": self._mtbf(distance, wall_contacts, status),
                "recovery_success_rate": None,
                "solve_rate_vs_complexity": "aggregate-only",
                "failure_taxonomy": self._failure_taxonomy(status),
                "realtime_factor": realtime_factor,
            },
            "artifacts": {
                "live_dashboard": str(self.live_dir),
                "kpis_latest": str(self.live_dir / "kpis.latest.json"),
                "map_thumbnail": str(self.live_dir / "map_thumb.png"),
            },
        }
        return snapshot

    def _run_metadata(self) -> dict[str, object]:
        manifest_path = self.output_dir / "run_manifest.json"
        manifest = {}
        if manifest_path.is_file():
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            except Exception:
                manifest = {}
        return {
            "run_directory": str(self.output_dir),
            "seed": self.seed,
            "duration_s": self.duration_s,
            "cell_size_m": float(self.get_parameter("corridor_width_m").value),
            "config_path": str(self.get_parameter("config_path").value),
            "git": manifest.get("git"),
            "parameters": manifest.get("parameters"),
        }

    def _sim_elapsed(self) -> float:
        if self.clock_start_sim is not None and self.clock_last_sim is not None:
            return max(0.0, self.clock_last_sim - self.clock_start_sim)
        return max(0.0, time.monotonic() - self.started_wall)

    def _distance_traveled(self) -> float | None:
        value = self.motion.get("distance_traveled_m")
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            return float(value)
        if len(self.actual_xy) >= 2:
            return path_length(list(self.actual_xy))
        if len(self.odom_eval) >= 2:
            return path_length([(x, y) for _, x, y, _ in self.odom_eval])
        return None

    def _goal_error(self) -> float | None:
        if not self.actual_xy:
            return None
        x, y = self.actual_xy[-1]
        return math.hypot(x - self.goal_xy[0], y - self.goal_xy[1])

    def _wall_contact_count(self) -> int:
        counts = self.motion.get("contact_counts", {})
        if isinstance(counts, dict):
            try:
                return int(counts.get("wall", 0))
            except Exception:
                return 0
        return 0

    @staticmethod
    def _demo_solve_status(status: str, goal_error: float | None, has_run_data: bool) -> str:
        normalized = str(status or "").upper()
        if goal_error is not None and goal_error <= 0.5:
            return "yes"
        if normalized == "GOAL_REACHED":
            return "yes"
        if not has_run_data:
            return "waiting"
        terminal_without_goal = {
            "EXPLORATION_COMPLETE",
            "RETURNED_TO_ORIGIN",
            "TIME_LIMIT_REACHED",
            "TIMEOUT",
            "FAILED",
            "ABORTED",
            "NAV2_ABORTED",
            "FALL_DETECTED",
            "COLLISION_ABORT",
            "COMMAND_TIMEOUT",
            "ODOMETRY_LOST",
            "SCAN_LOST",
            "STUCK",
        }
        if normalized in terminal_without_goal:
            return "no"
        if any(token in normalized for token in ("ABORT", "TIMEOUT", "LOST", "COLLISION", "FALL", "STUCK")):
            return "no"
        return "running"

    def _capture_health(self, topic_health: dict[str, dict[str, object]]) -> dict[str, object]:
        rates: dict[str, float | None] = {}
        pieces = []
        missing = []
        drops = []
        for name in ("scan", "imu", "odom", "map"):
            health = topic_health.get(name, {})
            rate = health.get("rate_hz")
            count = int(health.get("count") or 0)
            rates[name] = float(rate) if isinstance(rate, (int, float)) and math.isfinite(float(rate)) else None
            if count > 0 and rates[name] is not None:
                pieces.append(f"{name} {rates[name]:.1f} Hz")
            else:
                missing.append(name)
            drop = health.get("drop_fraction")
            if isinstance(drop, (int, float)) and math.isfinite(float(drop)):
                drops.append(float(drop))
        max_drop = max(drops) if drops else None
        sync_error = self._sensor_sync_error_ms()
        if pieces:
            summary_parts = list(pieces[:3])
            if max_drop is not None:
                summary_parts.append(f"max drop {max_drop * 100.0:.1f}%")
            if sync_error is not None:
                summary_parts.append(f"sync {sync_error:.0f} ms")
            if missing:
                summary_parts.append("waiting " + "/".join(missing))
            summary = ", ".join(summary_parts)
        else:
            summary = "waiting for scan/imu/odom/map"
        return {
            "summary": summary,
            "rate_hz": rates,
            "max_drop_fraction": max_drop,
            "inter_sensor_sync_error_ms": sync_error,
            "missing_topics": missing,
        }

    def _topic_health(self, now: float) -> dict[str, dict[str, object]]:
        names = sorted(set(self.expected_rates) | set(self.topic_times))
        result = {}
        for name in names:
            rate = message_rate(list(self.topic_times.get(name, ())), now=now, window_s=5.0)
            target = self.expected_rates.get(name)
            result[name] = {
                "count": self.topic_counts.get(name, 0),
                "rate_hz": rate,
                "target_hz": target,
                "drop_fraction": drop_fraction(rate, target),
                "invalid_count": self.invalid_counts.get(name, 0),
            }
        return result

    def _sensor_sync_error_ms(self) -> float | None:
        stamps = [self.latest_stamps[name] for name in ("scan", "imu", "odom") if name in self.latest_stamps]
        if len(stamps) < 2:
            return None
        return (max(stamps) - min(stamps)) * 1000.0

    def _schema_completeness(self) -> float:
        checks = [
            self.clock_last_sim is not None,
            bool(self.motion),
            bool(self.exploration),
            bool(self.latest_map_stats),
            bool(self.latest_scan),
            bool(self.odom_eval),
            self.optimal_path_m is not None,
            self.goal_xy is not None,
        ]
        return sum(1 for value in checks if value) / len(checks)

    def _process_stats(self) -> dict[str, object]:
        stats: dict[str, object] = {"pid": os.getpid()}
        try:
            rss_pages = int(Path("/proc/self/statm").read_text(encoding="utf-8").split()[1])
            stats["rss_mb"] = rss_pages * os.sysconf("SC_PAGE_SIZE") / 1_000_000.0
        except Exception:
            pass
        try:
            meminfo = Path("/proc/meminfo").read_text(encoding="utf-8").splitlines()
            values = {}
            for line in meminfo:
                key, raw = line.split(":", 1)
                values[key] = float(raw.strip().split()[0]) * 1024.0
            total = values.get("MemTotal")
            available = values.get("MemAvailable")
            if total and available:
                stats["system_memory_used_fraction"] = (total - available) / total
        except Exception:
            pass
        try:
            now = time.monotonic()
            stat = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()[1:]
            cpu_values = [int(value) for value in stat]
            total = sum(cpu_values)
            idle = cpu_values[3] + cpu_values[4]
            proc = Path("/proc/self/stat").read_text(encoding="utf-8").split()
            proc_ticks = int(proc[13]) + int(proc[14])
            if self._cpu_previous is not None:
                last_wall, last_total, last_idle, last_proc = self._cpu_previous
                total_delta = max(1, total - last_total)
                idle_delta = max(0, idle - last_idle)
                elapsed = max(1e-9, now - last_wall)
                ticks_per_second = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
                stats["system_cpu_fraction"] = max(0.0, min(1.0, 1.0 - idle_delta / total_delta))
                stats["process_cpu_fraction"] = max(0.0, (proc_ticks - last_proc) / ticks_per_second / elapsed)
            self._cpu_previous = (now, total, idle, proc_ticks)
        except Exception:
            pass
        return stats

    @staticmethod
    def _failure_taxonomy(status: str) -> str:
        if status in {"GOAL_REACHED", "EXPLORATION_COMPLETE", "RETURNED_TO_ORIGIN"}:
            return "none"
        if "STUCK" in status:
            return "stuck"
        if "COLLISION" in status:
            return "hit"
        if "LOST" in status:
            return "lost"
        if "TIMEOUT" in status:
            return "timeout"
        if status == "RUNNING":
            return "running"
        return status.lower() if status else "unknown"

    @staticmethod
    def _mtbf(distance: float | None, wall_contacts: int, status: str) -> float | None:
        failures = wall_contacts
        if status not in {"RUNNING", "GOAL_REACHED", "EXPLORATION_COMPLETE", "RETURNED_TO_ORIGIN"}:
            failures += 1
        if failures <= 0 or distance is None:
            return None
        return distance / failures

    def _render_map_thumbnail(self) -> None:
        if self.latest_map is None:
            return
        start = time.monotonic()
        msg = self.latest_map
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        gray = np.full(grid.shape, 205, dtype=np.uint8)
        gray[grid == 0] = 254
        gray[grid >= 65] = 0
        gray[(grid > 0) & (grid < 65)] = 128
        rgb = np.repeat(np.flipud(gray)[:, :, None], 3, axis=2)
        max_side = 720
        scale = max(1, math.ceil(max(rgb.shape[0], rgb.shape[1]) / max_side))
        if scale > 1:
            rgb = rgb[::scale, ::scale]
        tmp = self.live_dir / ".map_thumb.tmp.png"
        _write_png(tmp, rgb)
        tmp.replace(self.live_dir / "map_thumb.png")
        if time.monotonic() - start > 0.2:
            self.get_logger().debug(f"map thumbnail render took {time.monotonic() - start:.3f}s")

    def _init_timeseries(self) -> None:
        path = self.live_dir / "timeseries_downsampled.csv"
        if path.exists() and path.stat().st_size > 0:
            return
        with path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow([
                "wall_elapsed_s",
                "sim_elapsed_s",
                "status",
                "distance_traveled_m",
                "path_efficiency",
                "goal_error_m",
                "coverage_fraction",
                "min_clearance_m",
                "ate_rmse_m",
                "realtime_factor",
            ])

    def _append_timeseries(self, snapshot: dict[str, object]) -> None:
        mission = snapshot.get("mission", {})
        mapping = snapshot.get("mapping", {})
        safety = snapshot.get("safety_motion", {})
        localization = snapshot.get("localization", {})
        data_quality = snapshot.get("data_quality", {})
        status = snapshot.get("status", {})
        with (self.live_dir / "timeseries_downsampled.csv").open("a", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow([
                mission.get("wall_elapsed_s"),
                mission.get("sim_elapsed_s"),
                status.get("mission_status"),
                mission.get("distance_traveled_m"),
                mission.get("path_efficiency"),
                mission.get("final_goal_error_m"),
                mapping.get("coverage_fraction"),
                safety.get("min_wall_clearance_m"),
                localization.get("position_rmse_m") if isinstance(localization, dict) else None,
                data_quality.get("realtime_factor") if isinstance(data_quality, dict) else None,
            ])

    def _event(self, event: str, detail: dict[str, object]) -> None:
        row = {"wall_time_s": time.time(), "sim_elapsed_s": self._sim_elapsed(), "event": event, "detail": detail}
        with (self.live_dir / "events.ndjson").open("a", encoding="utf-8") as output:
            output.write(json.dumps(row, sort_keys=True) + "\n")

    @staticmethod
    def _write_atomic(path: Path, data: bytes) -> None:
        temporary = path.with_name(f".{path.name}.tmp")
        temporary.write_bytes(data)
        temporary.replace(path)

    def _start_http_server(self) -> None:
        bind = str(self.get_parameter("dashboard_bind_address").value)
        port = int(self.get_parameter("dashboard_port").value)
        try:
            self._http_server = ReusableThreadingHTTPServer((bind, port), make_handler(self.state, self.live_dir))
        except OSError as exc:
            self.get_logger().error(f"Live KPI dashboard HTTP server could not bind {bind}:{port}: {exc}")
            return
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, name="live-kpi-http", daemon=True)
        self._http_thread.start()
        self.get_logger().info(f"Live KPI dashboard: http://127.0.0.1:{port}/index.html")

    def _write_index(self) -> None:
        (self.live_dir / "index.html").write_text(INDEX_HTML, encoding="utf-8")
        placeholder = self.live_dir / "map_thumb.png"
        if not placeholder.exists():
            pixels = np.full((180, 260, 3), 235, dtype=np.uint8)
            _write_png(placeholder, pixels)

    def destroy_node(self):
        self.state.close()
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
        if self._http_thread is not None:
            self._http_thread.join(timeout=1.0)
        return super().destroy_node()


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 Live KPIs</title>
<style>
:root{color-scheme:light;--ink:#171717;--muted:#6b7280;--line:#d9dee7;--panel:#f8fafc;--good:#15803d;--warn:#b45309;--bad:#b91c1c;--accent:#0f766e}
*{box-sizing:border-box}body{margin:0;font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;color:var(--ink);background:#ffffff;letter-spacing:0}
header{display:flex;justify-content:space-between;gap:1rem;align-items:flex-start;padding:18px 20px;border-bottom:1px solid var(--line);background:#fff;position:sticky;top:0;z-index:2}
h1{font-size:18px;line-height:1.2;margin:0;font-weight:720}.sub{color:var(--muted);font-size:12px;margin-top:4px}.pill{border:1px solid var(--line);border-radius:8px;padding:6px 9px;font-size:12px;background:var(--panel);white-space:nowrap}.live{color:var(--good);font-weight:700}
main{padding:18px 20px}.content{display:grid;grid-template-columns:1.1fr .9fr;gap:18px}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:18px}
section{min-width:0}.demo-panel{margin:0 0 20px;padding-bottom:18px;border-bottom:1px solid var(--line)}.demo-panel h2{font-size:11px;text-transform:uppercase;letter-spacing:.08em;color:var(--muted);margin:0 0 8px}.demo-panel h3{font-size:20px;margin:0 0 10px}.demo-panel .row{grid-template-columns:minmax(0,220px) minmax(0,1fr)}.demo-panel .value{text-align:left;max-width:none;color:var(--ink);font-weight:650}
.kpi-group h2{font-size:11px;text-transform:uppercase;letter-spacing:.08em;color:var(--muted);margin:0 0 8px}.kpi-group h3{font-size:16px;margin:0 0 8px}
.rows{border-top:1px solid var(--line)}.row{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:10px;align-items:baseline;border-bottom:1px solid var(--line);min-height:32px;padding:6px 0}.label{font-size:13px;min-width:0}.value{font-size:13px;color:var(--muted);font-variant-numeric:tabular-nums;text-align:right;max-width:220px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.map-panel{border-left:1px solid var(--line);padding-left:18px}.map-panel img{display:block;width:100%;height:auto;border:1px solid var(--line);border-radius:8px;background:#eef2f7;image-rendering:auto}.status-line{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin:12px 0}.stat{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:8px}.stat b{display:block;font-size:11px;color:var(--muted);font-weight:650;text-transform:uppercase;letter-spacing:.06em}.stat span{display:block;margin-top:4px;font-size:15px;font-variant-numeric:tabular-nums}
.good{color:var(--good)}.warn{color:var(--warn)}.bad{color:var(--bad)}@media(max-width:980px){.content{grid-template-columns:1fr}.map-panel{border-left:0;padding-left:0}.grid{grid-template-columns:1fr}}@media(max-width:540px){header{display:block}.pill{display:inline-block;margin-top:10px}.status-line{grid-template-columns:1fr}.row,.demo-panel .row{grid-template-columns:1fr}.value{text-align:left;max-width:none}}
</style>
</head>
<body>
<header><div><h1>G1 Live KPI Monitor</h1><div class="sub" id="run">Waiting for data</div></div><div class="pill"><span id="conn" class="warn">connecting</span></div></header>
<main>
<section class="demo-panel">
<h2>On Screen, Live</h2>
<h3>The Live Demo</h3>
<div class="rows" id="demoRows"></div>
</section>
<div class="content">
<div class="grid" id="groups"></div>
<aside class="map-panel">
<h2>Map</h2>
<img id="map" src="map_thumb.png" alt="SLAM occupancy map">
<div class="status-line">
<div class="stat"><b>Status</b><span id="status">-</span></div>
<div class="stat"><b>RTF</b><span id="rtf">-</span></div>
<div class="stat"><b>Coverage</b><span id="coverage">-</span></div>
</div>
<section class="kpi-group"><h2>Data Quality</h2><h3>Topic Health</h3><div class="rows" id="topics"></div></section>
</aside>
</div>
</main>
<script>
const groups=[
 {key:"success",eyebrow:"Success",title:"Held-out solve rate",rows:[["Single-run result","success.single_run_terminal_status"],["Reached generated goal","success.single_run_goal_reached"],["Held-out solve rate","success.held_out_solve_rate_note"]]},
 {key:"mission",eyebrow:"Mission",title:"Efficiency and speed",rows:[["Path efficiency","mission.path_efficiency","ratio"],["Time elapsed","mission.sim_elapsed_s","s"],["Distance traveled","mission.distance_traveled_m","m"],["Final goal error","mission.final_goal_error_m","m"]]},
 {key:"safety_motion",eyebrow:"Safety and motion",title:"Did it move cleanly?",rows:[["Wall collisions","safety_motion.wall_collisions"],["Min wall clearance","safety_motion.min_wall_clearance_m","m"],["Stuck events and recoveries","safety_motion.stuck_events_and_recoveries"],["Yaw accel RMS","safety_motion.smoothness.yaw_accel_rms_radps2","rad/s2"]]},
 {key:"localization",eyebrow:"Localization",title:"Did it know where it was?",rows:[["ATE vs ground truth","localization.position_rmse_m","m"],["Final odom error","localization.final_position_error_m","m"],["Drift scale","localization.distance_scale","ratio"],["Aligned samples","localization.aligned_samples"]]},
 {key:"mapping",eyebrow:"Mapping",title:"Did it map the maze?",rows:[["Map coverage","mapping.coverage_fraction","pct"],["Known cells","mapping.known_cells"],["Free cells","mapping.free_cells"],["Plan updates","mapping.nav2_plan_updates"]]},
 {key:"reliability",eyebrow:"Reliability",title:"Would it run unsupervised?",rows:[["MTBF","reliability.mtbf_m_per_failure","m"],["Failure taxonomy","reliability.failure_taxonomy"],["Realtime factor","reliability.realtime_factor","ratio"],["Solve vs complexity","reliability.solve_rate_vs_complexity"]]}
];
function path(obj,key){return key.split(".").reduce((v,k)=>v&&Object.prototype.hasOwnProperty.call(v,k)?v[k]:undefined,obj)}
function fmt(value,unit){if(value===undefined||value===null||Number.isNaN(value))return "N/A";if(typeof value==="boolean")return value?"yes":"no";if(typeof value==="number"){if(unit==="pct")return (100*value).toFixed(1)+"%";if(unit==="ratio")return value.toFixed(2);if(unit==="s")return value.toFixed(1)+" s";if(unit==="m")return value.toFixed(2)+" m";if(unit==="rad/s2")return value.toFixed(2)+" rad/s2";return Math.abs(value)>=100?value.toFixed(0):value.toFixed(3)}return String(value)}
function renderDemo(snapshot){const demo=path(snapshot,"demo")||{};const collisions=demo.collisions_stuck||{};const capture=demo.capture_health||{};const time=fmt(demo.time_to_goal_s,"s");const state=demo.time_to_goal_status?String(demo.time_to_goal_status):"waiting";const rows=[["Solve","reached goal: "+fmt(demo.solve)],["Time-to-goal",time+" ("+state+")"],["Collisions / stuck",collisions.summary||"waiting"],["Capture health",capture.summary||"waiting"]];const root=document.getElementById("demoRows");root.innerHTML="";for(const row of rows){const line=document.createElement("div");line.className="row";line.innerHTML="<div class='label'></div><div class='value'></div>";line.children[0].textContent=row[0];line.children[1].textContent=row[1];root.appendChild(line)}}
function render(snapshot){renderDemo(snapshot);document.getElementById("conn").textContent="live";document.getElementById("conn").className="live";document.getElementById("run").textContent=(snapshot.run.run_directory||"run")+" | seed "+snapshot.run.seed+" | cell "+snapshot.run.cell_size_m+" m";document.getElementById("status").textContent=fmt(path(snapshot,"status.mission_status"));document.getElementById("rtf").textContent=fmt(path(snapshot,"data_quality.realtime_factor"),"ratio");document.getElementById("coverage").textContent=fmt(path(snapshot,"mapping.coverage_fraction"),"pct");document.getElementById("map").src="map_thumb.png?t="+Date.now();
 const root=document.getElementById("groups");root.innerHTML="";for(const group of groups){const el=document.createElement("section");el.className="kpi-group";el.innerHTML="<h2>"+group.eyebrow+"</h2><h3>"+group.title+"</h3><div class='rows'></div>";const rows=el.querySelector(".rows");for(const row of group.rows){const line=document.createElement("div");line.className="row";line.innerHTML="<div class='label'></div><div class='value'></div>";line.children[0].textContent=row[0];line.children[1].textContent=fmt(path(snapshot,row[1]),row[2]);rows.appendChild(line)}root.appendChild(el)}
 const topics=document.getElementById("topics");topics.innerHTML="";const topicData=path(snapshot,"data_quality.topics")||{};for(const name of Object.keys(topicData).sort()){const row=document.createElement("div");row.className="row";const t=topicData[name];const drop=t.drop_fraction===null||t.drop_fraction===undefined?"":", drop "+fmt(t.drop_fraction,"pct");row.innerHTML="<div class='label'></div><div class='value'></div>";row.children[0].textContent=name;row.children[1].textContent=fmt(t.rate_hz)+" Hz"+drop;topics.appendChild(row)}}
function poll(){fetch("kpis.latest.json?ts="+Date.now()).then(r=>r.ok?r.json():null).then(j=>{if(j)render(j)}).catch(()=>{})}
try{const events=new EventSource("events");events.addEventListener("kpis",e=>render(JSON.parse(e.data)));events.onerror=()=>{document.getElementById("conn").textContent="polling";document.getElementById("conn").className="warn";setTimeout(poll,750)}}catch(e){setInterval(poll,1000)}
setInterval(poll,5000);
</script>
</body>
</html>
"""


def main() -> int:
    rclpy.init()
    node = LiveKpiMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
