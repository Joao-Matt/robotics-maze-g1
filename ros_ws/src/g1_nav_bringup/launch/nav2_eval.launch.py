"""Replay a saved SLAM map and compare Nav2 commands with the oracle controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    seed, duration = LaunchConfiguration("seed"), LaunchConfiguration("duration_s")
    output, config, port = LaunchConfiguration("output_dir"), LaunchConfiguration("config_path"), LaunchConfiguration("port")
    rviz_enabled = LaunchConfiguration("with_rviz")
    policy, unitree_repo = LaunchConfiguration("locomotion_policy"), LaunchConfiguration("unitree_rl_gym_repo")
    map_yaml, map_tf = LaunchConfiguration("map_yaml"), LaunchConfiguration("map_to_odom_path")
    goal_tf = LaunchConfiguration("goal_map_to_odom_path")
    zero_timeout, corridor = LaunchConfiguration("zero_command_timeout_s"), LaunchConfiguration("corridor_width_m")
    bridge_share, nav_share = FindPackageShare("g1_mujoco_bridge"), FindPackageShare("g1_nav_bringup")
    live = PathJoinSubstitution([output, "live_nav2"])
    common = {"seed": ParameterValue(seed, value_type=int), "output_dir": ParameterValue(output, value_type=str)}
    bridge = Node(package="g1_mujoco_bridge", executable="bridge_node", name="g1_mujoco_bridge", output="screen",
        parameters=[PathJoinSubstitution([bridge_share,"config","bridge.yaml"]), common, {
            "config_path": ParameterValue(config,value_type=str), "motion_mode":"oracle_mapping",
            "motion_duration_s": ParameterValue(duration,value_type=float), "zero_command_timeout_s": ParameterValue(zero_timeout,value_type=float),
            "corridor_width_m": ParameterValue(corridor,value_type=float),
            "unitree_rl_gym_repo": ParameterValue(unitree_repo,value_type=str), "locomotion_policy": ParameterValue(policy,value_type=str),
            "publish_map_to_odom":False, "live_visual_dir":ParameterValue(live,value_type=str), "focused_nav_visuals":True}])
    scan = Node(package="depthimage_to_laserscan", executable="depthimage_to_laserscan_node", name="depthimage_to_laserscan",
        parameters=[PathJoinSubstitution([bridge_share,"config","d435i_depth_to_scan.yaml"])],
        remappings=[("depth","/camera/depth/image_rect_raw"),("depth_camera_info","/camera/depth/camera_info"),("scan","/scan")])
    saved_tf = Node(package="g1_nav_bringup", executable="saved_map_tf", parameters=[{"transform_path":ParameterValue(map_tf,value_type=str),"use_sim_time":True}])
    map_server = Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
        parameters=[{"yaml_filename":ParameterValue(map_yaml,value_type=str),"use_sim_time":True}])
    params=PathJoinSubstitution([nav_share,"config","nav2_params.yaml"])
    nav_nodes=[
        Node(package="nav2_planner",executable="planner_server",name="planner_server",parameters=[params]),
        Node(package="nav2_controller",executable="controller_server",name="controller_server",parameters=[params]),
        Node(package="nav2_bt_navigator",executable="bt_navigator",name="bt_navigator",parameters=[params]),
        Node(package="nav2_behaviors",executable="behavior_server",name="behavior_server",parameters=[params]),
        Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="lifecycle_manager_navigation",
             parameters=[params,{"node_names":["map_server","planner_server","controller_server","bt_navigator","behavior_server"]}]),
    ]
    probe=Node(package="g1_nav_bringup",executable="nav2_probe",name="nav2_probe",output="screen",parameters=[common,{
        "duration_s":ParameterValue(duration,value_type=float),"live_visual_dir":ParameterValue(live,value_type=str),
        "config_path":ParameterValue(config,value_type=str),"corridor_width_m":ParameterValue(corridor,value_type=float),
        "map_to_odom_path":ParameterValue(map_tf,value_type=str),"goal_map_to_odom_path":ParameterValue(goal_tf,value_type=str),"use_sim_time":True}])
    web=ExecuteProcess(cmd=["python3","-m","http.server",port,"--bind","0.0.0.0","--directory",live],output="log")
    rviz=Node(package="rviz2",executable="rviz2",arguments=["-d",PathJoinSubstitution([nav_share,"rviz","nav2_slam.rviz"])],condition=IfCondition(rviz_enabled))
    shutdown=RegisterEventHandler(OnProcessExit(target_action=probe,on_exit=[EmitEvent(event=Shutdown(reason="Nav2 evaluation complete"))]))
    return LaunchDescription([
        DeclareLaunchArgument("seed",default_value="123"),DeclareLaunchArgument("duration_s",default_value="600"),
        DeclareLaunchArgument("output_dir",default_value="/workspace/runs/visual"),DeclareLaunchArgument("config_path",default_value="/workspace/configs/default.yaml"),
        DeclareLaunchArgument("port",default_value="8765"),DeclareLaunchArgument("with_rviz",default_value="false"),
        DeclareLaunchArgument("unitree_rl_gym_repo",default_value="/workspace/third_party/unitree_rl_gym"),
        DeclareLaunchArgument("locomotion_policy",default_value="unitree_rl_gym_native"),
        DeclareLaunchArgument("map_yaml"),DeclareLaunchArgument("map_to_odom_path"),DeclareLaunchArgument("goal_map_to_odom_path"),DeclareLaunchArgument("zero_command_timeout_s",default_value="20"),
        DeclareLaunchArgument("corridor_width_m",default_value="2.0"),LogInfo(msg=["Stage 2 live evaluation: http://127.0.0.1:",port,"/ros_bridge_live.html"]),
        bridge,scan,saved_tf,map_server,*nav_nodes,probe,web,rviz,shutdown])
