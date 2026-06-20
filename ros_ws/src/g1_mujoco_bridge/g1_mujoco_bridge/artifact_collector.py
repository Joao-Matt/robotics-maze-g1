"""Collect bounded ROS bridge evidence and write the Phase 2 dashboard."""

from __future__ import annotations

from collections import defaultdict
from html import escape
from pathlib import Path
import json
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from tf2_msgs.msg import TFMessage

from scripts.run_d435i_visual_check import normalize_depth
from sim.config import load_config
from sim.d435i_sensor import D435iSpec
from sim.mujoco_runner import _write_png


REQUIRED_TOPICS = (
    "/clock",
    "/joint_states",
    "/imu/data",
    "/camera/color/image_raw",
    "/camera/color/camera_info",
    "/camera/depth/image_rect_raw",
    "/camera/depth/camera_info",
    "/tf",
    "/tf_static",
)


class ArtifactCollector(Node):
    def __init__(self) -> None:
        super().__init__("ros_bridge_artifact_collector")
        self.declare_parameter("seed", 123)
        self.declare_parameter("config_path", "/workspace/configs/default.yaml")
        self.declare_parameter("output_dir", "/workspace/runs/visual")
        self.declare_parameter("duration_s", 8.0)
        self.declare_parameter("expected_clock_rate_hz", 100.0)
        self.declare_parameter("expected_joint_tf_rate_hz", 50.0)
        self.declare_parameter("expected_imu_rate_hz", 50.0)
        self.declare_parameter("expected_camera_rate_hz", 3.0)
        self.seed = int(self.get_parameter("seed").value)
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.duration = float(self.get_parameter("duration_s").value)
        camera_rate = float(self.get_parameter("expected_camera_rate_hz").value)
        joint_tf_rate = float(self.get_parameter("expected_joint_tf_rate_hz").value)
        self.expected_rates = {
            "/clock": float(self.get_parameter("expected_clock_rate_hz").value),
            "/joint_states": joint_tf_rate,
            "/tf": joint_tf_rate,
            "/imu/data": float(self.get_parameter("expected_imu_rate_hz").value),
            "/camera/color/image_raw": camera_rate,
            "/camera/color/camera_info": camera_rate,
            "/camera/depth/image_rect_raw": camera_rate,
            "/camera/depth/camera_info": camera_rate,
        }
        self.spec = D435iSpec.from_config(load_config(str(self.get_parameter("config_path").value)))
        if self.spec is None:
            raise RuntimeError("D435i must be enabled for bridge artifact collection")

        self.received: dict[str, list[float]] = defaultdict(list)
        self.last_stamps: dict[str, tuple[int, int]] = {}
        self.monotonic_stamps = True
        self.latest_rgb: Image | None = None
        self.latest_depth: Image | None = None
        self.rgb_info: CameraInfo | None = None
        self.depth_info: CameraInfo | None = None
        self.joint_message: JointState | None = None
        self.imu_message: Imu | None = None
        self.tf_edges: set[tuple[str, str]] = set()
        self.success = False
        self.done = False
        self.finish_timer = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.startup_timer = self.create_timer(
            self.duration + 10.0, self._startup_timeout, clock=self.steady_clock
        )

        self.create_subscription(ClockMessage, "/clock", lambda msg: self._record("/clock", msg.clock), 10)
        self.create_subscription(JointState, "/joint_states", self._joint, 10)
        self.create_subscription(Imu, "/imu/data", self._imu, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/color/image_raw", self._rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self._depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._rgb_info, 10)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info", self._depth_info, 10)
        self.create_subscription(TFMessage, "/tf", lambda msg: self._tf(msg, "/tf"), 10)
        static_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(TFMessage, "/tf_static", lambda msg: self._tf(msg, "/tf_static"), static_qos)
        self.get_logger().info(f"waiting for /clock, then collecting for {self.duration:g} seconds")

    def _record(self, topic: str, stamp) -> None:
        if topic == "/clock" and self.finish_timer is None:
            self.startup_timer.cancel()
            self.finish_timer = self.create_timer(self.duration, self._finish, clock=self.steady_clock)
            self.get_logger().info("/clock received; evidence window started")
        now = time.monotonic()
        self.received[topic].append(now)
        current = (int(stamp.sec), int(stamp.nanosec))
        previous = self.last_stamps.get(topic)
        if previous is not None and current < previous:
            self.monotonic_stamps = False
        self.last_stamps[topic] = current

    def _startup_timeout(self) -> None:
        self.get_logger().error("timed out waiting for /clock")
        self.done = True
        self.startup_timer.cancel()

    def _joint(self, message: JointState) -> None:
        self.joint_message = message
        self._record("/joint_states", message.header.stamp)

    def _imu(self, message: Imu) -> None:
        self.imu_message = message
        self._record("/imu/data", message.header.stamp)

    def _rgb(self, message: Image) -> None:
        self.latest_rgb = message
        self._record("/camera/color/image_raw", message.header.stamp)

    def _depth(self, message: Image) -> None:
        self.latest_depth = message
        self._record("/camera/depth/image_rect_raw", message.header.stamp)

    def _rgb_info(self, message: CameraInfo) -> None:
        self.rgb_info = message
        self._record("/camera/color/camera_info", message.header.stamp)

    def _depth_info(self, message: CameraInfo) -> None:
        self.depth_info = message
        self._record("/camera/depth/camera_info", message.header.stamp)

    def _tf(self, message: TFMessage, topic: str) -> None:
        for transform in message.transforms:
            self.tf_edges.add((transform.header.frame_id, transform.child_frame_id))
        if message.transforms:
            self._record(topic, message.transforms[0].header.stamp)

    def _rates(self) -> dict[str, dict[str, float | int]]:
        result = {}
        for topic, arrivals in sorted(self.received.items()):
            span = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0.0
            rate = (len(arrivals) - 1) / span if span > 0.0 else 0.0
            result[topic] = {"message_count": len(arrivals), "rate_hz": rate}
        return result

    def _finish(self) -> None:
        paths = self._paths()
        discovered = dict(self.get_topic_names_and_types())
        rates = self._rates()
        camera_info_valid = self._camera_info_valid(self.rgb_info, "rgb") and self._camera_info_valid(
            self.depth_info, "depth"
        )
        required_edges = {
            ("map", "odom"),
            ("odom", "base_link"),
            ("base_link", "torso_link"),
            ("torso_link", self.spec.link_name),
            (self.spec.link_name, self.spec.rgb_optical_frame),
            (self.spec.link_name, self.spec.depth_optical_frame),
            (self.spec.link_name, self.spec.imu_frame),
        }
        minimum_counts = {topic: 3 for topic in REQUIRED_TOPICS}
        minimum_counts["/tf_static"] = 1
        counts_valid = all(
            len(self.received.get(topic, [])) >= minimum for topic, minimum in minimum_counts.items()
        )
        rates_valid = all(
            topic in rates and 0.7 * target <= float(rates[topic]["rate_hz"]) <= 1.3 * target
            for topic, target in self.expected_rates.items()
        )
        topics_valid = all(topic in discovered for topic in REQUIRED_TOPICS)
        frames_valid = required_edges.issubset(self.tf_edges)
        images_valid = self.latest_rgb is not None and self.latest_depth is not None
        joint_valid = bool(
            self.joint_message
            and self.joint_message.name
            and len(self.joint_message.name) == len(self.joint_message.position)
        )
        imu_valid = bool(self.imu_message and self.imu_message.header.frame_id == self.spec.imu_frame)
        forbidden = sorted(
            topic for topic in discovered if topic == "/scan" or "slam" in topic or "nav2" in topic
        )
        self.success = all(
            (topics_valid, counts_valid, rates_valid, camera_info_valid, frames_valid, images_valid, joint_valid,
             imu_valid, self.monotonic_stamps, not forbidden)
        )

        depth_stats = {}
        if images_valid:
            rgb = np.frombuffer(self.latest_rgb.data, dtype=np.uint8).reshape(
                self.latest_rgb.height, self.latest_rgb.width, 3
            )
            _write_png(paths["rgb"], rgb)
            depth = np.frombuffer(self.latest_depth.data, dtype=np.float32).reshape(
                self.latest_depth.height, self.latest_depth.width
            )
            depth_rgb, depth_stats = normalize_depth(
                depth, self.spec.depth_visual_min_m, self.spec.depth_visual_max_m
            )
            _write_png(paths["depth"], depth_rgb)

        topic_lines = [f"{name}: {', '.join(types)}" for name, types in sorted(discovered.items())]
        paths["topics"].write_text("\n".join(topic_lines) + "\n", encoding="utf-8")
        rate_lines = [
            f"{topic}: {values['rate_hz']:.3f} Hz ({values['message_count']} messages)"
            for topic, values in rates.items()
        ]
        paths["rates"].write_text("\n".join(rate_lines) + "\n", encoding="utf-8")
        self._write_tf_svg(paths["tf"])
        summary = {
            "status": "completed" if self.success else "failed",
            "phase": 2,
            "seed": self.seed,
            "use_sim_time": True,
            "topics_valid": topics_valid,
            "message_counts_valid": counts_valid,
            "topic_rates_valid": rates_valid,
            "expected_topic_rates_hz": self.expected_rates,
            "camera_info_valid": camera_info_valid,
            "tf_valid": frames_valid,
            "timestamps_monotonic": self.monotonic_stamps,
            "joint_states_valid": joint_valid,
            "imu_valid": imu_valid,
            "forbidden_topics": forbidden,
            "topic_rates": rates,
            "tf_edges": sorted([list(edge) for edge in self.tf_edges]),
            "depth": depth_stats,
            "artifacts": {name: str(path) for name, path in paths.items()},
        }
        paths["summary"].write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self._write_dashboard(paths["dashboard"], summary, paths)
        print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
        self.get_logger().info(f"bridge check {summary['status']}: {paths['dashboard']}")
        self.done = True
        if self.finish_timer is not None:
            self.finish_timer.cancel()

    def _camera_info_valid(self, message: CameraInfo | None, kind: str) -> bool:
        if message is None:
            return False
        expected_frame = self.spec.rgb_optical_frame if kind == "rgb" else self.spec.depth_optical_frame
        return bool(
            message.width == self.spec.width
            and message.height == self.spec.height
            and message.header.frame_id == expected_frame
            and len(message.k) == 9
            and message.k[0] > 0.0
            and message.k[4] > 0.0
        )

    def _paths(self) -> dict[str, Path]:
        prefix = f"ros_bridge_seed-{self.seed}"
        return {
            "topics": self.output_dir / f"{prefix}_topics.txt",
            "rates": self.output_dir / f"{prefix}_topic_rates.txt",
            "tf": self.output_dir / f"{prefix}_tf_frames.svg",
            "rgb": self.output_dir / f"{prefix}_rgb.png",
            "depth": self.output_dir / f"{prefix}_depth.png",
            "dashboard": self.output_dir / f"{prefix}_dashboard.html",
            "summary": self.output_dir / f"{prefix}_summary.json",
        }

    def _write_tf_svg(self, path: Path) -> None:
        edges = sorted(self.tf_edges)
        height = max(160, 60 + len(edges) * 34)
        rows = "".join(
            f'<text x="40" y="{55 + index * 34}" font-size="18">{escape(parent)} → {escape(child)}</text>'
            for index, (parent, child) in enumerate(edges)
        )
        path.write_text(
            f'<svg xmlns="http://www.w3.org/2000/svg" width="800" height="{height}" viewBox="0 0 800 {height}">'
            '<rect width="100%" height="100%" fill="#111827"/><text x="40" y="28" fill="#93c5fd" font-size="22">ROS 2 TF Frames</text>'
            f'<g fill="#e5e7eb">{rows}</g></svg>\n',
            encoding="utf-8",
        )

    @staticmethod
    def _write_dashboard(path: Path, summary: dict, paths: dict[str, Path]) -> None:
        metadata = escape(json.dumps(summary, indent=2, sort_keys=True))
        path.write_text(
            f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><title>ROS Bridge Check</title>
<style>body{{font-family:sans-serif;background:#111827;color:#e5e7eb;margin:1.5rem}}main{{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:1rem}}section,pre{{background:#1f2937;padding:1rem;border-radius:.5rem}}img{{width:100%}}pre{{white-space:pre-wrap;overflow-wrap:anywhere}}</style></head>
<body><h1>ROS 2 MuJoCo Sensor Bridge — {escape(str(summary['status']))}</h1><main>
<section><h2>RGB</h2><img src="{paths['rgb'].name}"></section>
<section><h2>Depth</h2><img src="{paths['depth'].name}"></section>
<section><h2>TF</h2><img src="{paths['tf'].name}"></section></main>
<p><a href="{paths['topics'].name}">Topics</a> · <a href="{paths['rates'].name}">Rates</a></p><pre>{metadata}</pre></body></html>\n""",
            encoding="utf-8",
        )


def main() -> int:
    rclpy.init()
    node = ArtifactCollector()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        success = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1
