"""Launch file for the Modern Waypoint Editor."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    """Generate launch description for modern waypoint editor demo."""

    # Mock robot node
    mock_robot = Node(
        package='simple_navigator',
        executable='mock_robot',
        name='mock_robot',
        output='screen',
        parameters=[{
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
        }]
    )

    # Navigator node
    navigator = Node(
        package='simple_navigator',
        executable='navigator',
        name='navigator',
        output='screen',
        parameters=[{
            'config_file': os.path.join(
                os.path.dirname(__file__),
                '..', 'config', 'waypoints.yaml'
            )
        }]
    )

    # Modern waypoint editor GUI
    modern_editor = Node(
        package='simple_navigator',
        executable='modern_waypoint_editor',
        name='modern_waypoint_editor',
        output='screen'
    )

    return LaunchDescription([
        mock_robot,
        navigator,
        modern_editor,
    ])
