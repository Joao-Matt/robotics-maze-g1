from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler
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
    port = LaunchConfiguration("port")
    package = FindPackageShare("g1_mujoco_bridge")
    bridge_config = PathJoinSubstitution([package, "config", "bridge.yaml"])
    scan_config = PathJoinSubstitution([package, "config", "d435i_depth_to_scan.yaml"])
    rviz_config = PathJoinSubstitution([package, "rviz", "d435i_scan.rviz"])
    live_dir = PathJoinSubstitution([output_dir, "live_scan"])
    common = {
        "seed": ParameterValue(seed, value_type=int),
        "config_path": ParameterValue(config_path, value_type=str),
        "output_dir": ParameterValue(output_dir, value_type=str),
    }
    bridge = Node(
        package="g1_mujoco_bridge", executable="bridge_node", name="g1_mujoco_bridge",
        output="screen", parameters=[bridge_config, common, {
            "live_visual_dir": ParameterValue(live_dir, value_type=str), "live_scan_panel": True,
        }],
    )
    converter = Node(
        package="depthimage_to_laserscan", executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan", output="screen", parameters=[scan_config],
        remappings=[("depth", "/camera/depth/image_rect_raw"),
                    ("depth_camera_info", "/camera/depth/camera_info"), ("scan", "/scan")],
    )
    overlay = Node(
        package="g1_mujoco_bridge", executable="scan_artifact_collector",
        name="d435i_scan_live_overlay", output="screen",
        parameters=[common, {"bounded": False, "live_visual_dir": ParameterValue(live_dir, value_type=str)}],
    )
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz_config], output="screen")
    web = ExecuteProcess(
        cmd=["python3", "-m", "http.server", port, "--bind", "0.0.0.0", "--directory", live_dir],
        output="screen",
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(target_action=bridge, on_exit=[EmitEvent(event=Shutdown(reason="bridge stopped"))])
    )
    return LaunchDescription([
        DeclareLaunchArgument("seed", default_value="123"),
        DeclareLaunchArgument("config_path", default_value="/workspace/configs/default.yaml"),
        DeclareLaunchArgument("output_dir", default_value="/workspace/runs/visual"),
        DeclareLaunchArgument("port", default_value="8765"),
        LogInfo(msg=["Live scan dashboard: http://127.0.0.1:", port, "/ros_bridge_live.html"]),
        bridge, converter, overlay, rviz, web, shutdown,
    ])
