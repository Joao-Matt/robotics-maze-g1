"""Legacy one-stage developer launch.

The supported two-stage workflow is orchestrated by ``make nav2-slam-demo`` or
``make nav2-slam-view``: Phase 4's mapping launch runs first, followed by
``nav2_eval.launch.py`` on the saved map. This file remains for compatibility
with older direct ROS launch invocations.
"""

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
    seed=LaunchConfiguration('seed'); duration=LaunchConfiguration('duration_s'); out=LaunchConfiguration('output_dir'); config=LaunchConfiguration('config_path'); port=LaunchConfiguration('port'); rv=LaunchConfiguration('with_rviz'); lucky=LaunchConfiguration('lucky_g1_repo')
    bridge_share=FindPackageShare('g1_mujoco_bridge'); nav_share=FindPackageShare('g1_nav_bringup'); live=PathJoinSubstitution([out,'live_nav2'])
    bridge=Node(package='g1_mujoco_bridge',executable='bridge_node',name='g1_mujoco_bridge',output='screen',parameters=[PathJoinSubstitution([bridge_share,'config','bridge.yaml']),{'seed':ParameterValue(seed,value_type=int),'config_path':ParameterValue(config,value_type=str),'output_dir':ParameterValue(out,value_type=str),'motion_mode':'oracle_mapping','motion_duration_s':ParameterValue(duration,value_type=float),'corridor_width_m':2.0,'lucky_g1_repo':ParameterValue(lucky,value_type=str),'publish_map_to_odom':False,'live_visual_dir':ParameterValue(live,value_type=str),'live_scan_panel':True,'live_map_panel':True,'live_nav_panel':True}])
    scan=Node(package='depthimage_to_laserscan',executable='depthimage_to_laserscan_node',name='depthimage_to_laserscan',parameters=[PathJoinSubstitution([bridge_share,'config','d435i_depth_to_scan.yaml'])],remappings=[('depth','/camera/depth/image_rect_raw'),('depth_camera_info','/camera/depth/camera_info'),('scan','/scan')])
    slam=Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',parameters=[PathJoinSubstitution([bridge_share,'config','slam_toolbox.yaml'])],output='screen')
    params=PathJoinSubstitution([nav_share,'config','nav2_params.yaml'])
    nodes=[Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[params]),Node(package='nav2_controller',executable='controller_server',name='controller_server',parameters=[params],remappings=[('cmd_vel','/cmd_vel')]),Node(package='nav2_bt_navigator',executable='bt_navigator',name='bt_navigator',parameters=[params]),Node(package='nav2_behaviors',executable='behavior_server',name='behavior_server',parameters=[params]),Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',parameters=[params])]
    scan_overlay=Node(package='g1_mujoco_bridge',executable='scan_artifact_collector',name='nav_scan_overlay',parameters=[{'seed':ParameterValue(seed,value_type=int),'config_path':ParameterValue(config,value_type=str),'bounded':False,'corridor_width_m':2.0,'live_visual_dir':ParameterValue(live,value_type=str)}])
    map_writer=Node(package='g1_mujoco_bridge',executable='slam_artifact_collector',name='nav_map_writer',parameters=[{'seed':ParameterValue(seed,value_type=int),'config_path':ParameterValue(config,value_type=str),'output_dir':ParameterValue(out,value_type=str),'duration_s':ParameterValue(duration,value_type=float),'live_visual_dir':ParameterValue(live,value_type=str),'corridor_width_m':2.0}])
    probe=Node(package='g1_nav_bringup',executable='nav2_probe',name='nav2_probe',output='screen',parameters=[{'seed':ParameterValue(seed,value_type=int),'duration_s':ParameterValue(duration,value_type=float),'output_dir':ParameterValue(out,value_type=str),'live_visual_dir':ParameterValue(live,value_type=str)}])
    web=ExecuteProcess(cmd=['python3','-m','http.server',port,'--bind','0.0.0.0','--directory',live]); rviz=Node(package='rviz2',executable='rviz2',arguments=['-d',PathJoinSubstitution([nav_share,'rviz','nav2_slam.rviz'])],condition=IfCondition(rv))
    shutdown=RegisterEventHandler(OnProcessExit(target_action=probe,on_exit=[EmitEvent(event=Shutdown(reason='Nav2 probe complete'))]))
    return LaunchDescription([DeclareLaunchArgument('seed',default_value='123'),DeclareLaunchArgument('duration_s',default_value='60'),DeclareLaunchArgument('output_dir',default_value='/workspace/runs/visual'),DeclareLaunchArgument('config_path',default_value='/workspace/configs/default.yaml'),DeclareLaunchArgument('port',default_value='8765'),DeclareLaunchArgument('with_rviz',default_value='false'),DeclareLaunchArgument('lucky_g1_repo',default_value='/workspace/third_party/g1-manipulation-challenge'),LogInfo(msg=['Live Nav2: http://127.0.0.1:',port,'/ros_bridge_live.html']),bridge,scan,slam,*nodes,scan_overlay,map_writer,probe,web,rviz,shutdown])
