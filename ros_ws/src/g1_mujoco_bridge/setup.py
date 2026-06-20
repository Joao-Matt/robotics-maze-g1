from glob import glob
from setuptools import find_packages, setup

package_name = "g1_mujoco_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Maze G1",
    maintainer_email="robotics@example.com",
    description="ROS 2 bridge for G1 MuJoCo sensors",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge_node = g1_mujoco_bridge.bridge_node:main",
            "artifact_collector = g1_mujoco_bridge.artifact_collector:main",
            "scan_artifact_collector = g1_mujoco_bridge.scan_artifact_collector:main",
            "slam_artifact_collector = g1_mujoco_bridge.slam_artifact_collector:main",
        ]
    },
)
