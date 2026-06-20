from glob import glob
from setuptools import find_packages, setup
setup(name="g1_nav_bringup", version="0.1.0", packages=find_packages(),
 data_files=[("share/ament_index/resource_index/packages", ["resource/g1_nav_bringup"]), ("share/g1_nav_bringup", ["package.xml"]), ("share/g1_nav_bringup/config", glob("config/*.yaml")), ("share/g1_nav_bringup/launch", glob("launch/*.launch.py")), ("share/g1_nav_bringup/rviz", glob("rviz/*.rviz"))],
 install_requires=["setuptools"], zip_safe=True, maintainer="Robotics Maze G1", maintainer_email="robotics@example.com", license="MIT",
 entry_points={"console_scripts": ["nav2_probe = g1_nav_bringup.nav2_probe:main", "saved_map_tf = g1_nav_bringup.saved_map_tf:main"]})
