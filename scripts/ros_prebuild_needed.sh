#!/usr/bin/env bash
set -euo pipefail

stamp="${1:?usage: ros_prebuild_needed.sh STAMP}"
check_sources="${ROS_PREBUILD_CHECK_SOURCES:-false}"
force="${ROS_FORCE_PREBUILD:-false}"

if [ "$force" = "true" ]; then
  exit 0
fi

required_paths=(
  "ros_ws/install/setup.sh"
  "ros_ws/install/explore_lite_msgs"
  "ros_ws/install/explore_lite"
  "ros_ws/install/g1_mujoco_bridge"
  "ros_ws/install/g1_nav_bringup"
)

if [ ! -f "$stamp" ]; then
  exit 0
fi

for path in "${required_paths[@]}"; do
  if [ ! -e "$path" ]; then
    exit 0
  fi
done

if [ "$check_sources" != "true" ]; then
  exit 1
fi

source_roots=(
  "ros_ws/src/g1_mujoco_bridge"
  "ros_ws/src/g1_nav_bringup"
  "third_party/m-explore-ros2/explore"
  "third_party/m-explore-ros2/explore_lite_msgs"
  "patches"
)

for root in "${source_roots[@]}"; do
  if [ -e "$root" ] && find "$root" -type f \
    \( -name '*.py' -o -name '*.yaml' -o -name '*.xml' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name 'CMakeLists.txt' -o -name 'package.xml' -o -name 'setup.py' \) \
    -newer "$stamp" -print -quit | grep -q .; then
    exit 0
  fi
done

exit 1
