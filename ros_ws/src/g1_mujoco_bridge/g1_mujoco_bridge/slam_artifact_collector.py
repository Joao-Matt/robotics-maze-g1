"""Collect SLAM Toolbox maps, metrics, and visual evidence."""

from __future__ import annotations

from html import escape
from pathlib import Path
import json
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from maze.generator import generate_maze_from_config
from maze.grid import WALL
from sim.config import load_config
from sim.mujoco_runner import _write_png


class SlamArtifactCollector(Node):
    def __init__(self) -> None:
        super().__init__("slam_artifact_collector")
        self.declare_parameter("seed", 123)
        self.declare_parameter("output_dir", "/workspace/runs/visual")
        self.declare_parameter("duration_s", 300.0)
        self.declare_parameter("live_visual_dir", "")
        self.declare_parameter("config_path", "/workspace/configs/default.yaml")
        self.declare_parameter("corridor_width_m", 2.0)
        self.seed = int(self.get_parameter("seed").value)
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.duration = float(self.get_parameter("duration_s").value)
        live = str(self.get_parameter("live_visual_dir").value)
        self.live_dir = Path(live) if live else None
        if self.live_dir:
            self.live_dir.mkdir(parents=True, exist_ok=True)
        config = load_config(str(self.get_parameter("config_path").value))
        config["maze"]["cell_size_m"] = float(self.get_parameter("corridor_width_m").value)
        self.truth_maze = generate_maze_from_config(config, self.seed)
        self.latest_map = None
        self.map_count = 0
        self.scan_times = []
        self.motion = {}
        self.tf_edges = set()
        self.done = False
        self.finish_requested = False
        self.success = False
        self.map_to_odom = None
        self.initial_map_to_odom = None
        self.started = time.monotonic()
        self.create_subscription(OccupancyGrid, "/map", self._map, 10)
        self.create_subscription(LaserScan, "/scan", lambda _: self.scan_times.append(time.monotonic()), qos_profile_sensor_data)
        self.create_subscription(String, "/mapping/status", self._status, 10)
        self.create_subscription(TFMessage, "/tf", self._tf, 20)
        static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, "/tf_static", self._tf, static_qos)
        self.timer = self.create_timer(self.duration, self._finish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _map(self, message: OccupancyGrid) -> None:
        self.latest_map = message
        self.map_count += 1
        if self.live_dir:
            self._png(self.live_dir / "slam_map.png", self._map_pixels(message))

    def _status(self, message: String) -> None:
        try:
            self.motion = json.loads(message.data)
        except json.JSONDecodeError:
            self.motion = {"status": message.data}
        status = str(self.motion.get("status", ""))
        if status in {"GOAL_REACHED", "ZERO_COMMAND_TIMEOUT", "TIMEOUT", "FALL_DETECTED", "FAILED"} and not self.finish_requested:
            self.finish_requested = True
            self.create_timer(0.5, self._finish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self.tf_edges.add((transform.header.frame_id, transform.child_frame_id))
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                self.map_to_odom = {
                    "translation": {"x": transform.transform.translation.x, "y": transform.transform.translation.y, "z": transform.transform.translation.z},
                    "rotation": {"x": transform.transform.rotation.x, "y": transform.transform.rotation.y, "z": transform.transform.rotation.z, "w": transform.transform.rotation.w},
                }
                if self.initial_map_to_odom is None:
                    self.initial_map_to_odom = self.map_to_odom

    @staticmethod
    def _map_pixels(message: OccupancyGrid) -> np.ndarray:
        grid = np.asarray(message.data, dtype=np.int16).reshape(message.info.height, message.info.width)
        gray = np.full(grid.shape, 205, dtype=np.uint8)
        gray[grid == 0] = 254
        gray[grid >= 65] = 0
        gray[(grid > 0) & (grid < 65)] = 128
        gray = np.flipud(gray)
        return np.repeat(gray[:, :, None], 3, axis=2)

    @staticmethod
    def _rate(times) -> float:
        return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0

    def _finish(self) -> None:
        prefix = f"slam_seed-{self.seed}"
        paths = {
            "pgm": self.output_dir / f"{prefix}_map.pgm",
            "yaml": self.output_dir / f"{prefix}_map.yaml",
            "rviz": self.output_dir / f"{prefix}_rviz.png",
            "dashboard": self.output_dir / f"{prefix}_dashboard.html",
            "tf": self.output_dir / f"{prefix}_tf_tree.svg",
            "summary": self.output_dir / f"{prefix}_summary.json",
            "bag": self.output_dir / f"{prefix}_bag",
            "map_to_odom": self.output_dir / f"{prefix}_map_to_odom.json",
            "initial_map_to_odom": self.output_dir / f"{prefix}_map_to_odom_initial.json",
        }
        known = occupied = free = 0
        if self.latest_map:
            grid = np.asarray(self.latest_map.data, dtype=np.int16)
            known, occupied, free = int(np.count_nonzero(grid >= 0)), int(np.count_nonzero(grid >= 65)), int(np.count_nonzero(grid == 0))
            pixels = self._map_pixels(self.latest_map)
            self._png(paths["rviz"], pixels)
            self._write_pgm(paths["pgm"], pixels[:, :, 0])
            info = self.latest_map.info
            paths["yaml"].write_text(
                f"image: {paths['pgm'].name}\nresolution: {info.resolution}\norigin: [{info.origin.position.x}, {info.origin.position.y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
                encoding="utf-8",
            )
        self._write_tf(paths["tf"])
        if self.map_to_odom:
            paths["map_to_odom"].write_text(json.dumps(self.map_to_odom, indent=2) + "\n", encoding="utf-8")
        if self.initial_map_to_odom:
            paths["initial_map_to_odom"].write_text(json.dumps(self.initial_map_to_odom, indent=2) + "\n", encoding="utf-8")
        scan_rate = self._rate(self.scan_times)
        distance = float(self.motion.get("distance_traveled_m", 0.0))
        tf_valid = ("map", "odom") in self.tf_edges and ("odom", "base_link") in self.tf_edges
        motion_status = str(self.motion.get("status", ""))
        motion_ok = motion_status not in {"FALL_DETECTED", "FAILED"}
        truth_comparison = self._truth_comparison(self.latest_map) if self.latest_map else {}
        self.success = bool(self.latest_map and self.map_count >= 1 and occupied > 0 and free > 0 and scan_rate > 0.8 and tf_valid and distance > 0.05 and motion_ok)
        summary = {
            "status": "completed" if self.success else "failed", "phase": 4, "seed": self.seed,
            "elapsed_wall_s": time.monotonic() - self.started, "scan_rate_hz": scan_rate,
            "map_message_count": self.map_count, "known_cells": known, "occupied_cells": occupied,
            "free_cells": free, "map_coverage_fraction": known / len(self.latest_map.data) if self.latest_map else 0.0,
            "motion": self.motion, "odom_source": "mujoco_ground_truth", "tf_valid": tf_valid,
            "ground_truth_comparison": truth_comparison,
            "rviz_capture_mode": "headless_equivalent", "artifacts": {k: str(v) for k, v in paths.items()},
        }
        paths["summary"].write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self._dashboard(paths["dashboard"], summary, paths)
        print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
        self.done = True
        self.timer.cancel()

    def _truth_comparison(self, message: OccupancyGrid) -> dict[str, object]:
        grid = np.asarray(message.data, dtype=np.int16).reshape(message.info.height, message.info.width)
        occupied_rows, occupied_cols = np.where(grid >= 65)
        matches = 0
        cell = self.truth_maze.spec.cell_size_m
        for row, col in zip(occupied_rows, occupied_cols):
            x = message.info.origin.position.x + (col + 0.5) * message.info.resolution
            y = message.info.origin.position.y + (row + 0.5) * message.info.resolution
            maze_col = round(x / cell + (self.truth_maze.spec.width_cells - 1) / 2.0)
            maze_row = round((self.truth_maze.spec.height_cells - 1) / 2.0 - y / cell)
            if 0 <= maze_row < self.truth_maze.spec.height_cells and 0 <= maze_col < self.truth_maze.spec.width_cells:
                matches += int(self.truth_maze.grid[maze_row, maze_col] == WALL)
        count = len(occupied_rows)
        return {
            "evaluation_only": True,
            "occupied_wall_match_fraction": matches / count if count else 0.0,
            "matched_occupied_cells": matches,
            "occupied_cells_evaluated": count,
        }

    @staticmethod
    def _write_pgm(path: Path, gray: np.ndarray) -> None:
        with path.open("wb") as output:
            output.write(f"P5\n{gray.shape[1]} {gray.shape[0]}\n255\n".encode("ascii"))
            output.write(gray.tobytes())

    @staticmethod
    def _png(path: Path, pixels: np.ndarray) -> None:
        tmp = path.with_name(f".{path.stem}.tmp.png")
        _write_png(tmp, pixels)
        tmp.replace(path)

    def _write_tf(self, path: Path) -> None:
        rows = "".join(f'<text x="30" y="{45+i*28}">{escape(a)} → {escape(b)}</text>' for i, (a, b) in enumerate(sorted(self.tf_edges)))
        path.write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="800" height="500"><rect width="100%" height="100%" fill="#111827"/><g fill="#e5e7eb" font-size="18">{rows}</g></svg>\n', encoding="utf-8")

    @staticmethod
    def _dashboard(path, summary, paths) -> None:
        path.write_text(f'''<!doctype html><html><head><meta charset="utf-8"><title>SLAM Map</title><style>body{{background:#111827;color:#eee;font-family:sans-serif}}main{{display:grid;grid-template-columns:1fr 1fr;gap:1rem}}img{{max-width:100%;background:#fff}}pre{{white-space:pre-wrap}}</style></head><body><h1>SLAM Mapping — {escape(summary["status"])}</h1><main><section><h2>Occupancy Map</h2><img src="{paths["rviz"].name}"></section><section><h2>TF Tree</h2><img src="{paths["tf"].name}"></section></main><pre>{escape(json.dumps(summary, indent=2, sort_keys=True))}</pre></body></html>\n''', encoding="utf-8")


def main() -> int:
    rclpy.init()
    node = SlamArtifactCollector()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        success = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1
