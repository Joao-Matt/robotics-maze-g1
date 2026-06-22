from __future__ import annotations

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class NoGroundTruthNavigationTest(unittest.TestCase):
    def test_ground_truth_odometry_node_is_not_installed(self) -> None:
        setup_py = ROOT / "ros_ws" / "src" / "g1_nav_bringup" / "setup.py"

        self.assertNotIn("ground_truth_odometry =", setup_py.read_text(encoding="utf-8"))

    def test_ground_truth_to_odom_node_is_absent(self) -> None:
        node = ROOT / "ros_ws" / "src" / "g1_nav_bringup" / "g1_nav_bringup" / "ground_truth_odometry.py"

        self.assertFalse(node.exists())

    def test_nav2_launch_forbids_ground_truth_navigation_odom(self) -> None:
        launch = ROOT / "ros_ws" / "src" / "g1_nav_bringup" / "launch" / "navigate_d435i.launch.py"

        self.assertIn('"ground_truth_navigation_odom":False', launch.read_text(encoding="utf-8"))

    def test_bridge_fails_if_ground_truth_navigation_odom_is_requested(self) -> None:
        bridge = ROOT / "ros_ws" / "src" / "g1_mujoco_bridge" / "g1_mujoco_bridge" / "bridge_node.py"
        text = bridge.read_text(encoding="utf-8")

        self.assertIn('self.motion_mode == "nav2_navigation"', text)
        self.assertIn("ground_truth_navigation_odom", text)
        self.assertIn("forbidden for Nav2 navigation", text)


if __name__ == "__main__":
    unittest.main()
