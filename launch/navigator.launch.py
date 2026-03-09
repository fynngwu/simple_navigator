from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(
                    get_package_share_directory("simple_navigator"),
                    "config",
                    "waypoints.yaml",
                ),
                description="Path to waypoint configuration file",
            ),
            Node(
                package="simple_navigator",
                executable="navigator_node",
                name="navigator_node",
                output="screen",
                parameters=[{"config_file": config_file}],
            ),
        ]
    )
