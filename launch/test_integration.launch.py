"""Test launch file for integration testing.

This launch file starts all three components:
1. mock_robot - Simulates robot with TF and odometry
2. navigator_node - Controls the robot to navigate to waypoints
3. (Optional) waypoint_editor - GUI for visualization

Usage:
    ros2 launch simple_navigator test_integration.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get config file path
    config_file = os.path.join(
        get_package_share_directory("simple_navigator"),
        "config",
        "waypoints.yaml",
    )

    return LaunchDescription(
        [
            # Mock Robot Node (simulates robot)
            Node(
                package="simple_navigator",
                executable="mock_robot",
                name="mock_robot",
                output="screen",
                parameters=[
                    {
                        "base_frame": "base_link",
                        "odom_frame": "odom",
                        "publish_rate": 50.0,
                        "initial_x": 0.0,
                        "initial_y": 0.0,
                        "initial_yaw": 0.0,
                    }
                ],
            ),
            # Navigator Node (controls robot)
            Node(
                package="simple_navigator",
                executable="navigator_node",
                name="navigator_node",
                output="screen",
                parameters=[
                    {
                        "config_file": config_file,
                        "control_frequency": 20.0,
                        "base_frame": "base_link",
                        "odom_frame": "odom",
                        "position_tolerance": 0.1,
                        "yaw_tolerance": 0.1,
                        "default_waypoint": "point_a",
                    }
                ],
            ),
        ]
    )


def generate_test_scenario():
    """Generate commands to run test scenario."""
    commands = [
        # Wait for nodes to initialize
        "sleep 2",
        # Set target waypoint (point_a at x=1.0)
        "ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}' --once",
        # Send go signal
        "sleep 1",
        "ros2 topic pub /go std_msgs/msg/Bool '{data: true}' --once",
        # Monitor for goal reached
        "ros2 topic echo /goal_reached",
    ]

    return "\n".join(commands)
