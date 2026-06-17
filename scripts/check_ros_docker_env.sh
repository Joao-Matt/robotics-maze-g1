#!/usr/bin/env bash
set -eo pipefail

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -f /workspace/ros_ws/install/setup.bash ]; then
  source /workspace/ros_ws/install/setup.bash
fi

set -u

failures=0

check_command() {
  local command_name="$1"
  if command -v "${command_name}" >/dev/null 2>&1; then
    echo "ok command ${command_name}: $(command -v "${command_name}")"
  else
    echo "missing command ${command_name}" >&2
    failures=$((failures + 1))
  fi
}

check_pkg() {
  local package_name="$1"
  if ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
    echo "ok ros package ${package_name}: $(ros2 pkg prefix "${package_name}")"
  else
    echo "missing ros package ${package_name}" >&2
    failures=$((failures + 1))
  fi
}

if [ "${ROS_DISTRO:-}" = "humble" ]; then
  echo "ok ROS_DISTRO=humble"
else
  echo "expected ROS_DISTRO=humble, got '${ROS_DISTRO:-unset}'" >&2
  failures=$((failures + 1))
fi

check_command ros2
check_command rviz2
check_command python3
check_command colcon

check_pkg nav2_bringup
check_pkg nav2_bt_navigator
check_pkg slam_toolbox
check_pkg depthimage_to_laserscan
check_pkg robot_localization
check_pkg tf_transformations
check_pkg image_transport
check_pkg cv_bridge
check_pkg vision_msgs

if [ -d /workspace ]; then
  echo "ok workspace path exists: /workspace"
else
  echo "missing workspace path: /workspace" >&2
  failures=$((failures + 1))
fi

if [ "${failures}" -ne 0 ]; then
  echo "ROS Docker environment check failed with ${failures} missing item(s)." >&2
  exit 1
fi

echo "ROS Docker environment check passed."
