"""Validate depth-to-scan output and create Phase 3 visual evidence."""

from __future__ import annotations

from html import escape
from pathlib import Path
import json
import math
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage

from maze.generator import generate_maze_from_config
from maze.grid import WALL, physical_cell_length_m, physical_cell_width_m
from scripts.run_d435i_visual_check import normalize_depth
from sim.config import load_config
from sim.mujoco_runner import _write_png


class ScanArtifactCollector(Node):
    def __init__(self) -> None:
        super().__init__("d435i_scan_artifact_collector")
        self.declare_parameter("seed", 123)
        self.declare_parameter("config_path", "/workspace/configs/default.yaml")
        self.declare_parameter("output_dir", "/workspace/runs/visual")
        self.declare_parameter("duration_s", 8.0)
        self.declare_parameter("expected_scan_rate_hz", 3.0)
        self.declare_parameter("bounded", True)
        self.declare_parameter("live_visual_dir", "")
        self.declare_parameter("corridor_width_m", 0.0)
        self.seed = int(self.get_parameter("seed").value)
        self.config = load_config(str(self.get_parameter("config_path").value))
        corridor_width = float(self.get_parameter("corridor_width_m").value)
        if corridor_width > 0.0:
            self.config["maze"]["cell_size_m"] = corridor_width
            self.config["maze"]["cell_width_m"] = corridor_width
            self.config["maze"]["cell_length_m"] = corridor_width
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.duration = float(self.get_parameter("duration_s").value)
        self.expected_rate = float(self.get_parameter("expected_scan_rate_hz").value)
        self.bounded = bool(self.get_parameter("bounded").value)
        live_dir = str(self.get_parameter("live_visual_dir").value)
        self.live_dir = Path(live_dir) if live_dir else None
        if self.live_dir is not None:
            self.live_dir.mkdir(parents=True, exist_ok=True)

        self.maze = generate_maze_from_config(self.config, self.seed)
        self.latest_scan: LaserScan | None = None
        self.latest_rgb: Image | None = None
        self.latest_depth: Image | None = None
        self.scan_arrivals: list[float] = []
        self.depth_arrivals: list[float] = []
        self.last_stamp: tuple[int, int] | None = None
        self.timestamps_monotonic = True
        self.transforms: dict[str, tuple[str, np.ndarray]] = {}
        self.done = False
        self.success = False
        self.finish_timer = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.startup_timer = None
        if self.bounded:
            self.startup_timer = self.create_timer(
                self.duration + 10.0, self._startup_timeout, clock=self.steady_clock
            )

        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/color/image_raw", self._rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self._depth, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self._tf, 10)
        static_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(TFMessage, "/tf_static", self._tf, static_qos)
        self.get_logger().info("waiting for /scan")

    def _scan(self, message: LaserScan) -> None:
        self.latest_scan = message
        self.scan_arrivals.append(time.monotonic())
        stamp = (int(message.header.stamp.sec), int(message.header.stamp.nanosec))
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.timestamps_monotonic = False
        self.last_stamp = stamp
        if self.bounded and self.finish_timer is None:
            if self.startup_timer is not None:
                self.startup_timer.cancel()
            self.finish_timer = self.create_timer(self.duration, self._finish, clock=self.steady_clock)
            self.get_logger().info(f"/scan received; collecting for {self.duration:g} seconds")
        if self.live_dir is not None:
            overlay, _ = self._scan_overlay(message)
            if overlay is not None:
                self._atomic_png(self.live_dir / "scan_overlay.png", overlay)

    def _rgb(self, message: Image) -> None:
        self.latest_rgb = message

    def _depth(self, message: Image) -> None:
        self.latest_depth = message
        self.depth_arrivals.append(time.monotonic())

    def _tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self.transforms[transform.child_frame_id] = (
                transform.header.frame_id,
                self._transform_matrix(transform),
            )

    @staticmethod
    def _transform_matrix(transform) -> np.ndarray:
        q = transform.transform.rotation
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        rotation = np.array([
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
        ])
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ]
        return matrix

    def _map_transform(self, frame: str) -> np.ndarray | None:
        matrix = np.eye(4)
        current = frame
        visited = set()
        while current != "map":
            if current in visited or current not in self.transforms:
                return None
            visited.add(current)
            parent, parent_from_child = self.transforms[current]
            matrix = parent_from_child @ matrix
            current = parent
        return matrix

    def _scan_overlay(self, scan: LaserScan) -> tuple[np.ndarray | None, dict[str, float | int]]:
        map_from_scan = self._map_transform(scan.header.frame_id)
        if map_from_scan is None:
            return None, {}
        size = 768
        image = np.full((size, size, 3), 238, dtype=np.uint8)
        cell_width = physical_cell_width_m(self.maze.spec)
        cell_length = physical_cell_length_m(self.maze.spec)
        extent_x = self.maze.spec.width_cells * cell_width
        extent_y = self.maze.spec.height_cells * cell_length

        def pixel(x: float, y: float) -> tuple[int, int]:
            px = int(np.clip((x / extent_x + 0.5) * size, 0, size - 1))
            py = int(np.clip((0.5 - y / extent_y) * size, 0, size - 1))
            return px, py

        cell_w = size / self.maze.spec.width_cells
        cell_h = size / self.maze.spec.height_cells
        for row in range(self.maze.spec.height_cells):
            for col in range(self.maze.spec.width_cells):
                if self.maze.grid[row, col] != WALL:
                    continue
                x0, x1 = int(col * cell_w), int((col + 1) * cell_w) + 1
                y0, y1 = int(row * cell_h), int((row + 1) * cell_h) + 1
                image[y0:y1, x0:x1] = [30, 35, 45]

        valid_points = []
        aligned = 0
        for index, raw_range in enumerate(scan.ranges):
            distance = float(raw_range)
            if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            point_scan = np.array([distance * math.cos(angle), distance * math.sin(angle), 0.0, 1.0])
            point_map = map_from_scan @ point_scan
            x, y = float(point_map[0]), float(point_map[1])
            valid_points.append((x, y))
            if self._near_wall(x, y):
                aligned += 1
            px, py = pixel(x, y)
            image[max(0, py-2):py+3, max(0, px-2):px+3] = [230, 45, 45]

        robot = map_from_scan @ np.array([0.0, 0.0, 0.0, 1.0])
        rx, ry = pixel(float(robot[0]), float(robot[1]))
        image[max(0, ry-6):ry+7, max(0, rx-6):rx+7] = [30, 190, 80]
        count = len(valid_points)
        return image, {
            "valid_range_count": count,
            "wall_aligned_count": aligned,
            "wall_alignment_fraction": aligned / count if count else 0.0,
        }

    def _near_wall(self, x: float, y: float) -> bool:
        cell_width = physical_cell_width_m(self.maze.spec)
        cell_length = physical_cell_length_m(self.maze.spec)
        col_float = x / cell_width + (self.maze.spec.width_cells - 1) / 2.0
        row_float = (self.maze.spec.height_cells - 1) / 2.0 - y / cell_length
        for row in range(max(0, math.floor(row_float - 0.6)), min(self.maze.spec.height_cells, math.ceil(row_float + 0.6) + 1)):
            for col in range(max(0, math.floor(col_float - 0.6)), min(self.maze.spec.width_cells, math.ceil(col_float + 0.6) + 1)):
                if self.maze.grid[row, col] != WALL:
                    continue
                if abs(col_float - col) <= 0.62 and abs(row_float - row) <= 0.62:
                    return True
        return False

    @staticmethod
    def _rate(arrivals: list[float]) -> float:
        if len(arrivals) < 2:
            return 0.0
        return (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])

    def _finish(self) -> None:
        paths = self._paths()
        scan = self.latest_scan
        overlay, alignment = self._scan_overlay(scan) if scan else (None, {})
        scan_rate = self._rate(self.scan_arrivals)
        depth_rate = self._rate(self.depth_arrivals)
        valid_ranges = int(alignment.get("valid_range_count", 0))
        alignment_fraction = float(alignment.get("wall_alignment_fraction", 0.0))
        frame_valid = bool(scan and scan.header.frame_id == "d435i_link" and self._map_transform(scan.header.frame_id) is not None)
        scan_shape_valid = bool(
            scan and scan.angle_increment > 0.0 and scan.angle_max > scan.angle_min
            and math.isclose(scan.range_min, 0.45, abs_tol=1e-5)
            and math.isclose(scan.range_max, 8.0, abs_tol=1e-5)
        )
        rate_valid = 0.6 * self.expected_rate <= scan_rate <= 1.4 * self.expected_rate
        topics = dict(self.get_topic_names_and_types())
        forbidden = sorted(topic for topic in topics if "slam" in topic or "nav2" in topic or topic == "/map")
        self.success = bool(
            scan and overlay is not None and self.latest_rgb and self.latest_depth and valid_ranges > 0
            and alignment_fraction >= 0.6 and frame_valid and scan_shape_valid and rate_valid
            and self.timestamps_monotonic and not forbidden
        )
        if overlay is not None:
            self._atomic_png(paths["overlay"], overlay)
            self._atomic_png(paths["rviz"], overlay)
        if self.latest_rgb is not None:
            rgb = np.frombuffer(self.latest_rgb.data, dtype=np.uint8).reshape(
                self.latest_rgb.height, self.latest_rgb.width, 3
            )
            self._atomic_png(paths["rgb"], rgb)
        if self.latest_depth is not None:
            depth = np.frombuffer(self.latest_depth.data, dtype=np.float32).reshape(
                self.latest_depth.height, self.latest_depth.width
            )
            depth_rgb, _ = normalize_depth(depth, 0.15, 8.0)
            self._atomic_png(paths["depth"], depth_rgb)
        rates = {"scan_hz": scan_rate, "depth_hz": depth_rate}
        paths["rates"].write_text(
            f"/scan: {scan_rate:.3f} Hz ({len(self.scan_arrivals)} messages)\n"
            f"/camera/depth/image_rect_raw: {depth_rate:.3f} Hz ({len(self.depth_arrivals)} messages)\n",
            encoding="utf-8",
        )
        summary = {
            "status": "completed" if self.success else "failed",
            "phase": 3,
            "seed": self.seed,
            "scan_frame": scan.header.frame_id if scan else None,
            "scan_configuration": {
                "range_min_m": scan.range_min if scan else None,
                "range_max_m": scan.range_max if scan else None,
                "angle_min_rad": scan.angle_min if scan else None,
                "angle_max_rad": scan.angle_max if scan else None,
                "angle_increment_rad": scan.angle_increment if scan else None,
                "scan_height": 1,
            },
            "topic_rates": rates,
            "topic_rates_valid": rate_valid,
            "tf_valid": frame_valid,
            "scan_shape_valid": scan_shape_valid,
            "timestamps_monotonic": self.timestamps_monotonic,
            "alignment": alignment,
            "forbidden_topics": forbidden,
            "rviz_capture_mode": "headless_equivalent",
            "artifacts": {name: str(path) for name, path in paths.items()},
        }
        paths["summary"].write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self._write_dashboard(paths["dashboard"], summary, paths)
        print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
        self.done = True
        if self.finish_timer is not None:
            self.finish_timer.cancel()

    def _paths(self) -> dict[str, Path]:
        prefix = f"d435i_scan_seed-{self.seed}"
        return {
            "rviz": self.output_dir / f"{prefix}_rviz.png",
            "rgb": self.output_dir / f"{prefix}_rgb.png",
            "depth": self.output_dir / f"{prefix}_depth.png",
            "overlay": self.output_dir / f"{prefix}_scan_overlay.png",
            "rates": self.output_dir / f"{prefix}_topic_rates.txt",
            "dashboard": self.output_dir / f"{prefix}_dashboard.html",
            "summary": self.output_dir / f"{prefix}_summary.json",
        }

    @staticmethod
    def _atomic_png(path: Path, pixels: np.ndarray) -> None:
        temporary = path.with_name(f".{path.stem}.tmp.png")
        _write_png(temporary, pixels)
        temporary.replace(path)

    @staticmethod
    def _write_dashboard(path: Path, summary: dict, paths: dict[str, Path]) -> None:
        panels = "".join(
            f'<section><h2>{escape(title)}</h2><img src="{artifact.name}"></section>'
            for title, artifact in (
                ("RViz-equivalent Scan View", paths["rviz"]),
                ("Maze Scan Alignment", paths["overlay"]),
                ("RGB", paths["rgb"]),
                ("Depth", paths["depth"]),
            )
        )
        path.write_text(
            f"""<!doctype html><html><head><meta charset="utf-8"><title>D435i Scan Check</title>
<style>body{{font-family:sans-serif;background:#111827;color:#e5e7eb;margin:1.5rem}}main{{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:1rem}}section,pre{{background:#1f2937;padding:1rem;border-radius:.5rem}}img{{width:100%}}pre{{white-space:pre-wrap}}</style></head><body>
<h1>D435i Depth-to-Scan — {escape(str(summary['status']))}</h1><p>RViz capture mode: {escape(str(summary['rviz_capture_mode']))}</p>
<main>{panels}</main><pre>{escape(json.dumps(summary, indent=2, sort_keys=True))}</pre></body></html>\n""",
            encoding="utf-8",
        )

    def _startup_timeout(self) -> None:
        self.get_logger().error("timed out waiting for /scan")
        self.done = True


def main() -> int:
    rclpy.init()
    node = ScanArtifactCollector()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        success = node.success or not node.bounded
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1
