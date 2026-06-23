from __future__ import annotations

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class Nav2ReverseRecoveryConfigTest(unittest.TestCase):
    def test_exploration_dwb_can_sample_reverse_velocity(self) -> None:
        params = ROOT / "ros_ws" / "src" / "g1_nav_bringup" / "config" / "nav2_exploration_params.yaml"
        text = params.read_text(encoding="utf-8")

        self.assertIn("min_vel_x: -0.3", text)
        self.assertNotIn("min_vel_x: 0.0", text)

    def test_exploration_recovery_can_back_up_instead_of_only_waiting(self) -> None:
        params = ROOT / "ros_ws" / "src" / "g1_nav_bringup" / "config" / "nav2_exploration_params.yaml"
        text = params.read_text(encoding="utf-8")

        self.assertIn("behavior_plugins: [backup, wait]", text)
        self.assertIn("backup: {plugin: nav2_behaviors/BackUp}", text)

    def test_no_spin_behavior_trees_include_short_reverse_recovery(self) -> None:
        trees = [
            ROOT / "ros_ws" / "src" / "g1_nav_bringup" / "behavior_trees" / "navigate_to_pose_no_spin.xml",
            ROOT / "ros_ws" / "src" / "g1_nav_bringup" / "behavior_trees" / "navigate_through_poses_no_spin.xml",
        ]

        for tree in trees:
            with self.subTest(tree=tree.name):
                text = tree.read_text(encoding="utf-8")
                self.assertIn('<BackUp backup_dist="0.20" backup_speed="0.08"/>', text)


if __name__ == "__main__":
    unittest.main()
