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
        }],
    )
    collector = Node(
        package="g1_mujoco_bridge",
        executable="artifact_collector",
        name="ros_bridge_artifact_collector",
        output="screen",
        parameters=[
            parameters,
            {
                "seed": ParameterValue(seed, value_type=int),
                "config_path": ParameterValue(config_path, value_type=str),
                "output_dir": ParameterValue(output_dir, value_type=str),
                "duration_s": ParameterValue(duration, value_type=float),
            },
        ],
    )
    stop_when_collected = RegisterEventHandler(
        OnProcessExit(target_action=collector, on_exit=[EmitEvent(event=Shutdown(reason="bridge check complete"))])
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("seed", default_value="123"),
            DeclareLaunchArgument("config_path", default_value="/workspace/configs/default.yaml"),
            DeclareLaunchArgument("output_dir", default_value="/workspace/runs/visual"),
            DeclareLaunchArgument("duration_s", default_value="8.0"),
            bridge,
            collector,
            stop_when_collected,
        ]
    )
