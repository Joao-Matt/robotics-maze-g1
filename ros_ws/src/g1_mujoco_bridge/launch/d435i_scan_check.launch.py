from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    seed = LaunchConfiguration("seed")
    config_path = LaunchConfiguration("config_path")
    output_dir = LaunchConfiguration("output_dir")
    duration = LaunchConfiguration("duration_s")
    package = FindPackageShare("g1_mujoco_bridge")
    bridge_config = PathJoinSubstitution([package, "config", "bridge.yaml"])
    scan_config = PathJoinSubstitution([package, "config", "d435i_depth_to_scan.yaml"])
    common = {
        "seed": ParameterValue(seed, value_type=int),
        "config_path": ParameterValue(config_path, value_type=str),
        "output_dir": ParameterValue(output_dir, value_type=str),
    }
    bridge = Node(
        package="g1_mujoco_bridge", executable="bridge_node", name="g1_mujoco_bridge",
        output="screen", parameters=[bridge_config, common],
    )
    converter = Node(
        package="depthimage_to_laserscan", executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan", output="screen", parameters=[scan_config],
        remappings=[
            ("depth", "/camera/depth/image_rect_raw"),
            ("depth_camera_info", "/camera/depth/camera_info"),
            ("scan", "/scan"),
        ],
    )
    collector = Node(
        package="g1_mujoco_bridge", executable="scan_artifact_collector",
        name="d435i_scan_artifact_collector", output="screen",
        parameters=[common, {"duration_s": ParameterValue(duration, value_type=float), "bounded": True}],
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(target_action=collector, on_exit=[EmitEvent(event=Shutdown(reason="scan check complete"))])
    )
    return LaunchDescription([
        DeclareLaunchArgument("seed", default_value="123"),
        DeclareLaunchArgument("config_path", default_value="/workspace/configs/default.yaml"),
        DeclareLaunchArgument("output_dir", default_value="/workspace/runs/visual"),
        DeclareLaunchArgument("duration_s", default_value="8.0"),
        bridge, converter, collector, shutdown,
    ])
