import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .math_utils import normalize_angle, quaternion_from_yaw


class MockRobotNode(Node):
    def __init__(self):
        super().__init__("mock_robot")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("debug_logging", True)
        self.declare_parameter("debug_log_period", 1.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)
        self.debug_logging = bool(self.get_parameter("debug_logging").value)
        self.debug_log_period = float(self.get_parameter("debug_log_period").value)

        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.last_time = self.get_clock().now()
        self.last_debug_log_time = None

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.update)
        self.get_logger().info(
            "Mock robot ready "
            f"(x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}, rate={self.publish_rate:.1f}Hz)"
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vyaw = msg.angular.z
        self.get_logger().debug(
            f"Received cmd_vel vx={self.vx:.2f}, vy={self.vy:.2f}, wz={self.vyaw:.2f}"
        )

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        self.x += (c * self.vx - s * self.vy) * dt
        self.y += (s * self.vx + c * self.vy) * dt
        self.yaw = normalize_angle(self.yaw + self.vyaw * dt)

        self.publish_odom(now)
        self.publish_tf(now)
        self.maybe_log_debug(now)

    def maybe_log_debug(self, now) -> None:
        if not self.debug_logging:
            return
        if self.last_debug_log_time is not None:
            delta = (now - self.last_debug_log_time).nanoseconds / 1e9
            if delta < self.debug_log_period:
                return
        self.last_debug_log_time = now
        self.get_logger().debug(
            f"state x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}, "
            f"vx={self.vx:.2f}, vy={self.vy:.2f}, wz={self.vyaw:.2f}"
        )

    def publish_odom(self, stamp) -> None:
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vyaw
        self.odom_pub.publish(odom)

    def publish_tf(self, stamp) -> None:
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp.to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockRobotNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
