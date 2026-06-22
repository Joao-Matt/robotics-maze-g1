"""Sensor-derived odometry, Livox Mid-360 SLAM, Nav2, and m-explore control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler, TimerAction
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
    dashboard,dashboard_port=LaunchConfiguration("dashboard"),LaunchConfiguration("dashboard_port")
    dashboard_auto_open=LaunchConfiguration("dashboard_auto_open")
    dashboard_rate,dashboard_visual_rate=LaunchConfiguration("dashboard_rate_hz"),LaunchConfiguration("dashboard_visual_rate_hz")
    evaluation_ground_truth_topic=LaunchConfiguration("evaluation_ground_truth_topic")
    scan_maximum_points=LaunchConfiguration("scan_maximum_points")
    icp_maximum_correspondence=LaunchConfiguration("icp_maximum_correspondence_m")
    icp_min_inlier_ratio=LaunchConfiguration("icp_min_inlier_ratio")
    icp_max_translation_error=LaunchConfiguration("icp_max_prediction_translation_error_m")
    icp_max_yaw_error=LaunchConfiguration("icp_max_prediction_yaw_error_rad")
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
    sensor_odom=Node(package="g1_nav_bringup",executable="d435i_scan_odometry",name="d435i_scan_odometry",output="screen",parameters=[{
        "use_sim_time":True,
        "output_odom_topic":"/odom",
        "publish_tf":True,
        "scan_maximum_points":ParameterValue(scan_maximum_points,value_type=int),
        "icp_maximum_correspondence_m":ParameterValue(icp_maximum_correspondence,value_type=float),
        "icp_min_inlier_ratio":ParameterValue(icp_min_inlier_ratio,value_type=float),
        "icp_max_prediction_translation_error_m":ParameterValue(icp_max_translation_error,value_type=float),
        "icp_max_prediction_yaw_error_rad":ParameterValue(icp_max_yaw_error,value_type=float)}])
    exploration_origin=Node(package="tf2_ros",executable="static_transform_publisher",name="exploration_search_origin_tf",arguments=[
        "--x","0.25","--y","0","--z","0","--yaw","0","--pitch","0","--roll","0","--frame-id","base_link","--child-frame-id","exploration_search_origin"])
    slam=Node(package="slam_toolbox",executable="async_slam_toolbox_node",name="slam_toolbox",output="screen",parameters=[PathJoinSubstitution([nav_share,"config","slam_toolbox_livox.yaml"])])
    params=LaunchConfiguration("nav2_params_file")
    no_spin_tree=PathJoinSubstitution([nav_share,"behavior_trees","navigate_to_pose_no_spin.xml"])
    no_spin_through_tree=PathJoinSubstitution([nav_share,"behavior_trees","navigate_through_poses_no_spin.xml"])
    nav_servers=[Node(package="nav2_planner",executable="planner_server",name="planner_server",parameters=[params]),Node(package="nav2_controller",executable="controller_server",name="controller_server",parameters=[params]),Node(package="nav2_bt_navigator",executable="bt_navigator",name="bt_navigator",parameters=[params,{"default_nav_to_pose_bt_xml":ParameterValue(no_spin_tree,value_type=str),"default_nav_through_poses_bt_xml":ParameterValue(no_spin_through_tree,value_type=str)}]),Node(package="nav2_behaviors",executable="behavior_server",name="behavior_server",parameters=[params])]
    lifecycle=Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="lifecycle_manager_navigation",parameters=[params],output="screen")
    explorer=Node(package="explore_lite",executable="explore",name="explore_node",output="screen",parameters=[PathJoinSubstitution([nav_share,"config","m_explore.yaml"])])
    fallback=Node(package="g1_nav_bringup",executable="maze_fallback_goal",name="maze_fallback_goal",output="screen",parameters=[{"m_explore_reset_on_complete":True,"recenter_on_explore_complete":True,"use_sim_time":True}])
    reporter=Node(package="g1_nav_bringup",executable="exploration_reporter",name="exploration_reporter",output="screen",parameters=[{"seed":ParameterValue(seed,value_type=int),"duration_s":ParameterValue(duration,value_type=float),"output_dir":ParameterValue(out,value_type=str),"config_path":ParameterValue(config,value_type=str),"live_visual_dir":"","corridor_width_m":ParameterValue(corridor,value_type=float),"m_explore_complete_terminal":False,"use_sim_time":True}])
    monitor=Node(package="g1_nav_bringup",executable="live_kpi_monitor",name="live_kpi_monitor",output="screen",condition=IfCondition(dashboard),parameters=[{"seed":ParameterValue(seed,value_type=int),"duration_s":ParameterValue(duration,value_type=float),"output_dir":ParameterValue(out,value_type=str),"config_path":ParameterValue(config,value_type=str),"corridor_width_m":ParameterValue(corridor,value_type=float),"dashboard_port":ParameterValue(dashboard_port,value_type=int),"dashboard_auto_open":ParameterValue(dashboard_auto_open,value_type=bool),"dashboard_rate_hz":ParameterValue(dashboard_rate,value_type=float),"dashboard_visual_rate_hz":ParameterValue(dashboard_visual_rate,value_type=float),"evaluation_ground_truth_topic":ParameterValue(evaluation_ground_truth_topic,value_type=str),"use_sim_time":True}])
    rviz=Node(package="rviz2",executable="rviz2",arguments=["-d",PathJoinSubstitution([nav_share,"rviz","nav2_slam.rviz"])],condition=IfCondition(rv))
    shutdown=RegisterEventHandler(OnProcessExit(target_action=reporter,on_exit=[EmitEvent(event=Shutdown(reason="cold-start exploration complete"))]),condition=UnlessCondition(mujoco_viewer))
    viewer_shutdown=RegisterEventHandler(OnProcessExit(target_action=bridge,on_exit=[EmitEvent(event=Shutdown(reason="MuJoCo viewer bridge exited"))]),condition=IfCondition(mujoco_viewer))
    return LaunchDescription([DeclareLaunchArgument("seed",default_value="123"),DeclareLaunchArgument("duration_s",default_value="600"),DeclareLaunchArgument("output_dir",default_value="/workspace/runs"),DeclareLaunchArgument("config_path",default_value="/workspace/configs/default.yaml"),DeclareLaunchArgument("nav2_params_file",default_value=PathJoinSubstitution([nav_share,"config","nav2_exploration_params.yaml"])),DeclareLaunchArgument("scan_maximum_points",default_value="180"),DeclareLaunchArgument("icp_maximum_correspondence_m",default_value="0.35"),DeclareLaunchArgument("icp_min_inlier_ratio",default_value="0.12"),DeclareLaunchArgument("icp_max_prediction_translation_error_m",default_value="0.22"),DeclareLaunchArgument("icp_max_prediction_yaw_error_rad",default_value="0.25"),DeclareLaunchArgument("with_rviz",default_value="false"),DeclareLaunchArgument("mujoco_viewer",default_value="false"),DeclareLaunchArgument("unitree_rl_gym_repo",default_value="/workspace/third_party/unitree_rl_gym"),DeclareLaunchArgument("locomotion_policy",default_value="unitree_rl_gym_native"),DeclareLaunchArgument("corridor_width_m",default_value="2.0"),DeclareLaunchArgument("dashboard",default_value="true"),DeclareLaunchArgument("dashboard_port",default_value="8765"),DeclareLaunchArgument("dashboard_auto_open",default_value="true"),DeclareLaunchArgument("dashboard_rate_hz",default_value="2"),DeclareLaunchArgument("dashboard_visual_rate_hz",default_value="1"),DeclareLaunchArgument("evaluation_ground_truth_topic",default_value="/ground_truth/odom"),LogInfo(msg="Live KPI dashboard node enabled; URL prints after the HTTP server binds.",condition=IfCondition(dashboard)),bridge,exploration_origin,sensor_odom,TimerAction(period=2.0,actions=[slam]),TimerAction(period=5.0,actions=nav_servers),TimerAction(period=8.0,actions=[lifecycle]),TimerAction(period=12.0,actions=[explorer]),TimerAction(period=13.0,actions=[fallback]),reporter,monitor,rviz,shutdown,viewer_shutdown])
