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
        self.assertIn('"sensor_offset_x_m":ParameterValue(sensor_offset_x,value_type=float)', text.replace(" ", ""))
        self.assertIn('"odometry_translation_scale":ParameterValue(odometry_translation_scale,value_type=float)', text.replace(" ", ""))
        self.assertIn('"scan_yaw_correction_weight":ParameterValue(scan_yaw_correction_weight,value_type=float)', text.replace(" ", ""))
        self.assertIn('"use_imu_orientation_yaw":ParameterValue(use_imu_orientation_yaw,value_type=bool)', text.replace(" ", ""))
        self.assertIn('"imu_orientation_yaw_scale":ParameterValue(imu_orientation_yaw_scale,value_type=float)', text.replace(" ", ""))
        self.assertIn('"command_latency_s":ParameterValue(command_latency,value_type=float)', text.replace(" ", ""))
        self.assertIn('"imu_latency_s":ParameterValue(imu_latency,value_type=float)', text.replace(" ", ""))
        self.assertIn('DeclareLaunchArgument("sensor_offset_x_m",default_value="0.05")', text.replace(" ", ""))
        self.assertIn('DeclareLaunchArgument("use_imu_orientation_yaw",default_value="true")', text.replace(" ", ""))
        self.assertIn('DeclareLaunchArgument("scan_yaw_correction_weight",default_value="0.50")', text.replace(" ", ""))
        self.assertIn('DeclareLaunchArgument("command_latency_s",default_value="0.0")', text.replace(" ", ""))
        self.assertIn('DeclareLaunchArgument("imu_latency_s",default_value="0.0")', text.replace(" ", ""))
        self.assertIn('DeclareLaunchArgument("scan_maximum_points",default_value="260")', text.replace(" ", ""))
        self.assertIn('DeclareLaunchArgument("icp_maximum_correspondence_m",default_value="0.30")', text.replace(" ", ""))
        self.assertIn('"publish_map_to_odom":False', text.replace(" ", ""))
        self.assertIn('"external_navigation_odom":True', text.replace(" ", ""))
        self.assertIn('"ground_truth_navigation_odom":False', text.replace(" ", ""))
        self.assertNotIn("odom_tf_republisher", text)
        self.assertNotIn("pointcloud_imu_odometry", text)
        self.assertNotIn('package="robot_localization"', text)
        self.assertNotIn('executable="ekf_node"', text)
        self.assertNotIn("odometry/filtered", text)

    def test_scan_odometry_defaults_to_active_odom_and_tf(self) -> None:
        node = ROOT / "ros_ws" / "src" / "g1_nav_bringup" / "g1_nav_bringup" / "d435i_scan_odometry.py"
        text = node.read_text(encoding="utf-8")

        self.assertIn('("output_odom_topic", "/odom")', text)
        self.assertIn('("publish_tf", True)', text)
        self.assertIn('("command_timeout_s", 0.35)', text)
        self.assertIn('("imu_timeout_s", 0.35)', text)
        self.assertIn('("odometry_translation_scale", 1.0)', text)
        self.assertIn('("odometry_yaw_scale", 1.0)', text)
        self.assertIn('("scan_yaw_correction_weight", 1.0)', text)
        self.assertIn('("use_imu_orientation_yaw", True)', text)
        self.assertIn("quaternion_yaw", text)
        self.assertIn("sensor_delta_from_base", text)
        self.assertIn("base_delta_from_sensor", text)
        self.assertIn("self.x+=c*dx-s*dy; self.y+=s*dx+c*dy", text)
        self.assertIn("self.publisher=self.create_publisher(Odometry,self.output_odom_topic,50)", text)

    def test_bridge_does_not_publish_active_odom_tf_when_external_odom_is_enabled(self) -> None:
        bridge = ROOT / "ros_ws" / "src" / "g1_mujoco_bridge" / "g1_mujoco_bridge" / "bridge_node.py"
        text = bridge.read_text(encoding="utf-8")

        self.assertIn('raise RuntimeError("MuJoCo ground truth odometry is forbidden for Nav2 navigation', text)
        self.assertIn('if not (isinstance(self.motion_session, Nav2MotionSession) and bool(self.get_parameter("external_navigation_odom").value)):', text)
        self.assertIn('transforms.insert(0, base_transform)', text)
        self.assertIn('message.header.frame_id = "debug_command_odom"', text)
        self.assertIn('message.child_frame_id = "debug_command_base_link"', text)

    def test_tuning_script_scores_ground_truth_offline_only(self) -> None:
        script = ROOT / "scripts" / "tune_navigation_odometry.py"
        text = script.read_text(encoding="utf-8")

        self.assertIn('"ground_truth_usage": "offline_scoring_only"', text)
        self.assertIn("NAVIGATE_LAUNCH_ARGS", text)
        self.assertNotIn("ground_truth_navigation_odom:=true", text)


if __name__ == "__main__":
    unittest.main()
