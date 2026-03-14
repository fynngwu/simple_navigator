import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    default_config = os.path.join(
        get_package_share_directory("simple_navigator"),
        "config",
        "target.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to target/navigation configuration file",
            ),
            Node(
                package="simple_navigator",
                executable="navigator",
                name="navigator",
                output="screen",
                parameters=[{"config_file": config_file}],
            ),
        ]
    )
