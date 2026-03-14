from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="simple_navigator",
                executable="target_goal_editor",
                name="target_goal_editor",
                output="screen",
            ),
        ]
    )
