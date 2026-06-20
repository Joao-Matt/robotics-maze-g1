from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = PROJECT_ROOT / "ros_ws" / "src" / "g1_mujoco_bridge"
NAV_PACKAGE_ROOT = PROJECT_ROOT / "ros_ws" / "src" / "g1_nav_bringup"


def test_ros_bridge_package_and_launch_files_exist():
    for relative in (
        "package.xml",
        "setup.py",
        "config/bridge.yaml",
        "launch/ros_bridge_check.launch.py",
        "launch/ros_bridge_view.launch.py",
        "launch/d435i_scan_check.launch.py",
        "launch/d435i_scan_view.launch.py",
        "config/d435i_depth_to_scan.yaml",
        "rviz/d435i_scan.rviz",
        "launch/slam_map.launch.py",
        "config/slam_toolbox.yaml",
        "rviz/slam_mapping.rviz",
        "g1_mujoco_bridge/slam_artifact_collector.py",
        "g1_mujoco_bridge/bridge_node.py",
        "g1_mujoco_bridge/artifact_collector.py",
    ):
        assert (PACKAGE_ROOT / relative).is_file()


def test_bridge_publishes_phase_2_topics_without_later_phase_scan():
    source = (PACKAGE_ROOT / "g1_mujoco_bridge" / "bridge_node.py").read_text(encoding="utf-8")
    for topic in (
        '"/clock"',
        '"/joint_states"',
        '"/imu/data"',
        '"/odom"',
        '"/camera/color/image_raw"',
        '"/camera/depth/image_rect_raw"',
        '"/camera/depth/camera_info"',
    ):
        assert topic in source
    assert '"/scan"' not in source


def test_makefile_exposes_host_and_container_bridge_check():
    makefile = (PROJECT_ROOT / "Makefile").read_text(encoding="utf-8")

    assert "ros-bridge-check:" in makefile
    assert "ros-bridge-check-inner:" in makefile
    assert "docker/run.sh make ros-bridge-check-inner" in makefile
    assert "ros_bridge_seed-$(SEED)_summary.json" in makefile
    assert "ros-bridge-view:" in makefile
    assert "ros_bridge_view.launch.py" in makefile


def test_depth_to_scan_is_converter_owned_and_later_navigation_is_absent():
    scan_config = (PACKAGE_ROOT / "config" / "d435i_depth_to_scan.yaml").read_text(encoding="utf-8")
    launch = (PACKAGE_ROOT / "launch" / "d435i_scan_check.launch.py").read_text(encoding="utf-8")
    makefile = (PROJECT_ROOT / "Makefile").read_text(encoding="utf-8")

    assert "range_min: 0.45" in scan_config
    assert "range_max: 8.0" in scan_config
    assert "output_frame: d435i_link" in scan_config
    assert "depthimage_to_laserscan_node" in launch
    assert '"/scan"' in launch
    assert "d435i-scan-check:" in makefile
    assert "d435i-scan-view:" in makefile
    assert "slam_toolbox" not in launch
    assert "nav2" not in launch


def test_nav2_slam_monitor_only_bringup_is_complete():
    for relative in (
        "package.xml",
        "setup.py",
        "config/nav2_params.yaml",
        "launch/nav2_slam.launch.py",
        "launch/nav2_eval.launch.py",
        "rviz/nav2_slam.rviz",
        "g1_nav_bringup/nav2_probe.py",
    ):
        assert (NAV_PACKAGE_ROOT / relative).is_file()

    config = (NAV_PACKAGE_ROOT / "config" / "nav2_params.yaml").read_text(encoding="utf-8")
    launch = (NAV_PACKAGE_ROOT / "launch" / "nav2_eval.launch.py").read_text(encoding="utf-8")
    probe = (NAV_PACKAGE_ROOT / "g1_nav_bringup" / "nav2_probe.py").read_text(encoding="utf-8")
    makefile = (PROJECT_ROOT / "Makefile").read_text(encoding="utf-8")

    assert "nav2_navfn_planner/NavfnPlanner" in config
    assert "dwb_core::DWBLocalPlanner" in config
    assert "robot_radius: 0.45" in config
    assert "inflation_radius: 0.55" in config
    assert "max_vel_x: 0.25" in config
    assert "max_vel_theta: 0.5" in config
    compact_launch = launch.replace(" ", "")
    assert '"publish_map_to_odom":False' in compact_launch
    assert '"motion_mode":"oracle_mapping"' in compact_launch
    assert '"cmd_vel_application":"monitor_only"' in probe.replace(" ", "")
    assert "nav2_map_server" in launch
    assert "map_to_odom_path" in launch
    assert '"/oracle_cmd_vel"' in (PACKAGE_ROOT / "g1_mujoco_bridge" / "bridge_node.py").read_text(encoding="utf-8")
    assert "nav2-slam-demo:" in makefile
    assert "nav2-slam-view:" in makefile


def test_focused_nav_visuals_and_rviz_have_requested_panels():
    bridge = (PACKAGE_ROOT / "g1_mujoco_bridge" / "bridge_node.py").read_text(encoding="utf-8")
    rviz = (NAV_PACKAGE_ROOT / "rviz" / "nav2_slam.rviz").read_text(encoding="utf-8")
    for title in ("Robot in Maze", "RGB Camera", "Maze — Bird's-eye View", "Nav2 Commands vs Oracle"):
        assert title in bridge
    assert "maze_overhead.png" in bridge
    assert "command_comparison.svg" in bridge
    assert "/map" in rviz and "/scan" in rviz and "/camera/depth/image_rect_raw" in rviz
    assert "Global Costmap" not in rviz


def test_slam_launch_assigns_tf_ownership_and_excludes_nav2():
    launch = (PACKAGE_ROOT / "launch" / "slam_map.launch.py").read_text(encoding="utf-8")
    config = (PACKAGE_ROOT / "config" / "slam_toolbox.yaml").read_text(encoding="utf-8")
    makefile = (PROJECT_ROOT / "Makefile").read_text(encoding="utf-8")

    assert '"publish_map_to_odom": False' in launch
    assert '"motion_mode": "oracle_mapping"' in launch
    assert "async_slam_toolbox_node" in launch
    assert "map_frame: map" in config
    assert "odom_frame: odom" in config
    assert "base_frame: base_link" in config
    assert "slam-map:" in makefile
    assert "slam-map-view:" in makefile
    assert "nav2" not in launch
