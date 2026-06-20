"""Sensor-derived odometry, Livox Mid-360 SLAM, Nav2, and m-explore control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    seed,duration=LaunchConfiguration("seed"),LaunchConfiguration("duration_s")
    out,config=LaunchConfiguration("output_dir"),LaunchConfiguration("config_path")
    rv,corridor=LaunchConfiguration("with_rviz"),LaunchConfiguration("corridor_width_m")
    unitree,policy=LaunchConfiguration("unitree_rl_gym_repo"),LaunchConfiguration("locomotion_policy")
    mujoco_viewer=LaunchConfiguration("mujoco_viewer")
    bridge_share,nav_share=FindPackageShare("g1_mujoco_bridge"),FindPackageShare("g1_nav_bringup")
    bridge=Node(package="g1_mujoco_bridge",executable="bridge_node",name="g1_mujoco_bridge",output="screen",parameters=[PathJoinSubstitution([bridge_share,"config","bridge.yaml"]),{
        "seed":ParameterValue(seed,value_type=int),"config_path":ParameterValue(config,value_type=str),"output_dir":ParameterValue(out,value_type=str),
        "motion_mode":"nav2_navigation","motion_duration_s":ParameterValue(duration,value_type=float),"corridor_width_m":ParameterValue(corridor,value_type=float),
        "unitree_rl_gym_repo":ParameterValue(unitree,value_type=str),
        "locomotion_policy":ParameterValue(policy,value_type=str),"publish_map_to_odom":False,"external_navigation_odom":True,"ground_truth_navigation_odom":False,
        "mujoco_viewer":ParameterValue(mujoco_viewer,value_type=bool),
        "clock_rate_hz":50.0,"camera_enabled":False,"livox_mid360_enabled":True,"livox_scan_rate_hz":10.0,
        "livox_horizontal_bins":720,"livox_range_min_m":0.10,"livox_range_max_m":40.0,
        "depth_only":True,"live_visual_dir":"","navigation_visuals":False}])
    sensor_odom=Node(package="g1_nav_bringup",executable="d435i_scan_odometry",name="d435i_scan_odometry",output="screen",parameters=[{"use_sim_time":True}])
    exploration_origin=Node(package="tf2_ros",executable="static_transform_publisher",name="exploration_search_origin_tf",arguments=[
        "--x","0.25","--y","0","--z","0","--yaw","0","--pitch","0","--roll","0","--frame-id","base_link","--child-frame-id","exploration_search_origin"])
    slam=Node(package="slam_toolbox",executable="async_slam_toolbox_node",name="slam_toolbox",output="screen",parameters=[PathJoinSubstitution([nav_share,"config","slam_toolbox_livox.yaml"])])
    params=LaunchConfiguration("nav2_params_file")
    no_spin_tree=PathJoinSubstitution([nav_share,"behavior_trees","navigate_to_pose_no_spin.xml"])
    no_spin_through_tree=PathJoinSubstitution([nav_share,"behavior_trees","navigate_through_poses_no_spin.xml"])
    nav_servers=[Node(package="nav2_planner",executable="planner_server",name="planner_server",parameters=[params]),Node(package="nav2_controller",executable="controller_server",name="controller_server",parameters=[params]),Node(package="nav2_bt_navigator",executable="bt_navigator",name="bt_navigator",parameters=[params,{"default_nav_to_pose_bt_xml":ParameterValue(no_spin_tree,value_type=str),"default_nav_through_poses_bt_xml":ParameterValue(no_spin_through_tree,value_type=str)}]),Node(package="nav2_behaviors",executable="behavior_server",name="behavior_server",parameters=[params])]
    lifecycle=Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="lifecycle_manager_navigation",parameters=[params],output="screen")
    explorer=Node(package="explore_lite",executable="explore",name="explore_node",output="screen",parameters=[PathJoinSubstitution([nav_share,"config","m_explore.yaml"])])
    fallback=Node(package="g1_nav_bringup",executable="maze_fallback_goal",name="maze_fallback_goal",output="screen",parameters=[{"seed":ParameterValue(seed,value_type=int),"config_path":ParameterValue(config,value_type=str),"corridor_width_m":ParameterValue(corridor,value_type=float),"use_sim_time":True}])
    reporter=Node(package="g1_nav_bringup",executable="exploration_reporter",name="exploration_reporter",output="screen",parameters=[{"seed":ParameterValue(seed,value_type=int),"duration_s":ParameterValue(duration,value_type=float),"output_dir":ParameterValue(out,value_type=str),"config_path":ParameterValue(config,value_type=str),"live_visual_dir":"","corridor_width_m":ParameterValue(corridor,value_type=float),"m_explore_complete_terminal":False,"use_sim_time":True}])
    rviz=Node(package="rviz2",executable="rviz2",arguments=["-d",PathJoinSubstitution([nav_share,"rviz","nav2_slam.rviz"])],condition=IfCondition(rv))
    shutdown=RegisterEventHandler(OnProcessExit(target_action=reporter,on_exit=[EmitEvent(event=Shutdown(reason="cold-start exploration complete"))]),condition=UnlessCondition(mujoco_viewer))
    viewer_shutdown=RegisterEventHandler(OnProcessExit(target_action=bridge,on_exit=[EmitEvent(event=Shutdown(reason="MuJoCo viewer bridge exited"))]),condition=IfCondition(mujoco_viewer))
    return LaunchDescription([DeclareLaunchArgument("seed",default_value="123"),DeclareLaunchArgument("duration_s",default_value="600"),DeclareLaunchArgument("output_dir",default_value="/workspace/runs"),DeclareLaunchArgument("config_path",default_value="/workspace/configs/default.yaml"),DeclareLaunchArgument("nav2_params_file",default_value=PathJoinSubstitution([nav_share,"config","nav2_exploration_params.yaml"])),DeclareLaunchArgument("with_rviz",default_value="false"),DeclareLaunchArgument("mujoco_viewer",default_value="false"),DeclareLaunchArgument("unitree_rl_gym_repo",default_value="/workspace/third_party/unitree_rl_gym"),DeclareLaunchArgument("locomotion_policy",default_value="unitree_rl_gym_native"),DeclareLaunchArgument("corridor_width_m",default_value="2.0"),bridge,exploration_origin,sensor_odom,TimerAction(period=2.0,actions=[slam]),TimerAction(period=5.0,actions=nav_servers),TimerAction(period=8.0,actions=[lifecycle]),TimerAction(period=12.0,actions=[explorer]),TimerAction(period=13.0,actions=[fallback]),reporter,rviz,shutdown,viewer_shutdown])
