from __future__ import annotations

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class DirectScanOdometryTest(unittest.TestCase):
    def test_navigation_launch_uses_direct_scan_odom_for_active_odom(self) -> None:
        launch = ROOT / "ros_ws" / "src" / "g1_nav_bringup" / "launch" / "navigate_d435i.launch.py"
        text = launch.read_text(encoding="utf-8")

        self.assertIn('executable="d435i_scan_odometry"', text)
        self.assertIn('"output_odom_topic":"/odom"', text.replace(" ", ""))
        self.assertIn('"publish_tf":True', text.replace(" ", ""))
        self.assertNotIn('package="robot_localization"', text)
        self.assertNotIn('executable="ekf_node"', text)
        self.assertNotIn("odometry/filtered", text)

    def test_scan_odometry_defaults_to_active_odom_and_tf(self) -> None:
        node = ROOT / "ros_ws" / "src" / "g1_nav_bringup" / "g1_nav_bringup" / "d435i_scan_odometry.py"
        text = node.read_text(encoding="utf-8")

        self.assertIn('("output_odom_topic", "/odom")', text)
        self.assertIn('("publish_tf", True)', text)
        self.assertIn("self.publisher=self.create_publisher(Odometry,self.output_odom_topic,50)", text)

    def test_tuning_script_scores_ground_truth_offline_only(self) -> None:
        script = ROOT / "scripts" / "tune_navigation_odometry.py"
        text = script.read_text(encoding="utf-8")

        self.assertIn('"ground_truth_usage": "offline_scoring_only"', text)
        self.assertIn("NAVIGATE_LAUNCH_ARGS", text)
        self.assertNotIn("ground_truth_navigation_odom:=true", text)


if __name__ == "__main__":
    unittest.main()
