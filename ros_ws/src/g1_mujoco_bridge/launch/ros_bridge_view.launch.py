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
    live_dir = PathJoinSubstitution([output_dir, "live"])
    parameters = PathJoinSubstitution([FindPackageShare("g1_mujoco_bridge"), "config", "bridge.yaml"])

    bridge = Node(
        package="g1_mujoco_bridge",
        executable="bridge_node",
        name="g1_mujoco_bridge",
        output="screen",
        parameters=[parameters, {
            "seed": ParameterValue(seed, value_type=int),
            "config_path": ParameterValue(config_path, value_type=str),
            "output_dir": ParameterValue(output_dir, value_type=str),
            "live_visual_dir": ParameterValue(live_dir, value_type=str),
        }],
    )
    web_server = ExecuteProcess(
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
        LogInfo(msg=["Live viewer: http://127.0.0.1:", port, "/ros_bridge_live.html"]),
        bridge,
        web_server,
        shutdown,
    ])
