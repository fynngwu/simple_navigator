#!/usr/bin/env python3
"""Mock Robot Node for simulation and testing.

This node simulates a holonomic robot by:
1. Subscribing to cmd_vel velocity commands
2. Simulating robot movement based on velocity
3. Publishing TF transform (odom -> base_link)
4. Publishing odometry messages

Use this for testing the navigator without real hardware.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d.euler as tf_transformations
import math
import time


class MockRobotNode(Node):
    """Mock holonomic robot for simulation/testing."""

    def __init__(self):
        super().__init__("mock_robot")

        # Declare parameters
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("publish_rate", 50.0)  # Hz
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.initial_x = self.get_parameter("initial_x").value
        self.initial_y = self.get_parameter("initial_y").value
        self.initial_yaw = self.get_parameter("initial_yaw").value

        # Robot state
        self.x = self.initial_x
        self.y = self.initial_y
        self.yaw = self.initial_yaw

        # Current velocity (from cmd_vel)
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

        # Last update time
        self.last_time = self.get_clock().now()

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for state updates
        update_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(update_period, self.update)

        self.get_logger().info(
            f"Mock robot initialized at ({self.x}, {self.y}, yaw={self.yaw})"
        )
        self.get_logger().info(f"Publishing {self.publish_rate} Hz")

    def cmd_vel_callback(self, msg: Twist):
        """Receive velocity command."""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vyaw = msg.angular.z
        self.get_logger().debug(
            f"Received cmd_vel: vx={self.vx:.3f}, vy={self.vy:.3f}, vyaw={self.vyaw:.3f}"
        )

    def update(self):
        """Update robot state and publish transforms."""
        # Calculate time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Update position (holonomic robot)
        # x += vx * dt, y += vy * dt, yaw += vyaw * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.yaw += self.vyaw * dt

        # Normalize yaw to [-pi, pi]
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi

        # Publish odometry
        self.publish_odom(current_time)

        # Publish TF
        self.publish_tf(current_time)

    def publish_odom(self, timestamp):
        """Publish odometry message."""
        odom = Odometry()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.header.stamp = timestamp.to_msg()

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        q = tf_transformations.euler2quat(0, 0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Velocity (in child frame)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vyaw

        self.odom_pub.publish(odom)

    def publish_tf(self, timestamp):
        """Publish odom -> base_link transform."""
        t = TransformStamped()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.header.stamp = timestamp.to_msg()

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        q = tf_transformations.euler2quat(0, 0, self.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = MockRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
