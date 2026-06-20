from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    seed, config_path, output_dir = LaunchConfiguration("seed"), LaunchConfiguration("config_path"), LaunchConfiguration("output_dir")
    duration, port, with_rviz = LaunchConfiguration("duration_s"), LaunchConfiguration("port"), LaunchConfiguration("with_rviz")
    corridor, lucky_repo, bag_path = LaunchConfiguration("corridor_width_m"), LaunchConfiguration("lucky_g1_repo"), LaunchConfiguration("bag_path")
    zero_timeout = LaunchConfiguration("zero_command_timeout_s")
    package = FindPackageShare("g1_mujoco_bridge")
    bridge_config = PathJoinSubstitution([package, "config", "bridge.yaml"])
    scan_config = PathJoinSubstitution([package, "config", "d435i_depth_to_scan.yaml"])
    slam_config = PathJoinSubstitution([package, "config", "slam_toolbox.yaml"])
    rviz_config = PathJoinSubstitution([package, "rviz", "slam_mapping.rviz"])
    live_dir = PathJoinSubstitution([output_dir, "live_slam"])
    common = {"seed": ParameterValue(seed, value_type=int), "output_dir": ParameterValue(output_dir, value_type=str)}
    bridge = Node(package="g1_mujoco_bridge", executable="bridge_node", name="g1_mujoco_bridge", output="screen",
        parameters=[bridge_config, common, {
            "config_path": ParameterValue(config_path, value_type=str), "motion_mode": "oracle_mapping",
            "motion_duration_s": ParameterValue(duration, value_type=float),
            "corridor_width_m": ParameterValue(corridor, value_type=float),
            "zero_command_timeout_s": ParameterValue(zero_timeout, value_type=float),
            "lucky_g1_repo": ParameterValue(lucky_repo, value_type=str), "publish_map_to_odom": False,
            "live_visual_dir": ParameterValue(live_dir, value_type=str), "live_scan_panel": True, "live_map_panel": True,
            "focused_nav_visuals": True,
        }])
    converter = Node(package="depthimage_to_laserscan", executable="depthimage_to_laserscan_node", name="depthimage_to_laserscan",
        parameters=[scan_config], remappings=[("depth", "/camera/depth/image_rect_raw"), ("depth_camera_info", "/camera/depth/camera_info"), ("scan", "/scan")])
    slam = Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox", output="screen", parameters=[slam_config])
    overlay = Node(package="g1_mujoco_bridge", executable="scan_artifact_collector", name="slam_scan_live_overlay",
        parameters=[common, {"config_path": ParameterValue(config_path, value_type=str), "bounded": False,
            "live_visual_dir": ParameterValue(live_dir, value_type=str), "corridor_width_m": ParameterValue(corridor, value_type=float)}])
    collector = Node(package="g1_mujoco_bridge", executable="slam_artifact_collector", name="slam_artifact_collector", output="screen",
        parameters=[common, {"duration_s": ParameterValue(PythonExpression([duration, " + 30.0"]), value_type=float), "live_visual_dir": ParameterValue(live_dir, value_type=str),
            "config_path": ParameterValue(config_path, value_type=str), "corridor_width_m": ParameterValue(corridor, value_type=float)}])
    web = ExecuteProcess(cmd=["python3", "-m", "http.server", port, "--bind", "0.0.0.0", "--directory", live_dir], output="log")
    bag = ExecuteProcess(cmd=["ros2", "bag", "record", "-o", bag_path, "/scan", "/map", "/tf", "/tf_static", "/imu/data", "/clock", "/mapping/status"], output="screen")
    rviz = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config], condition=IfCondition(with_rviz), output="screen")
    shutdown = RegisterEventHandler(OnProcessExit(target_action=collector, on_exit=[EmitEvent(event=Shutdown(reason="SLAM collection complete"))]))
    return LaunchDescription([
        DeclareLaunchArgument("seed", default_value="123"), DeclareLaunchArgument("config_path", default_value="/workspace/configs/default.yaml"),
        DeclareLaunchArgument("output_dir", default_value="/workspace/runs/visual"), DeclareLaunchArgument("duration_s", default_value="300"),
        DeclareLaunchArgument("port", default_value="8765"), DeclareLaunchArgument("with_rviz", default_value="false"),
        DeclareLaunchArgument("corridor_width_m", default_value="2.0"), DeclareLaunchArgument("lucky_g1_repo", default_value="/workspace/third_party/g1-manipulation-challenge"),
        DeclareLaunchArgument("bag_path", default_value="/workspace/runs/visual/slam_seed-123_bag"),
        DeclareLaunchArgument("zero_command_timeout_s", default_value="20"),
        LogInfo(msg=["Live SLAM dashboard: http://127.0.0.1:", port, "/ros_bridge_live.html"]),
        bridge, converter, slam, overlay, collector, web, bag, rviz, shutdown,
    ])
