from setuptools import find_packages, setup
import os
from glob import glob

package_name = "simple_navigator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your_email@example.com",
    description="Simple ROS2 navigation module for holonomic robots",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "navigator = simple_navigator.navigator_node:main",
            "target_goal_editor = simple_navigator.modern_waypoint_editor:main",
            "mock_robot = simple_navigator.mock_robot:main",
        ],
    },
)
