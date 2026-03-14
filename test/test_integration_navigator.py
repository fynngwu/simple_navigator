#!/usr/bin/env python3

import math
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool
    from tf2_ros import Buffer, TransformListener
except ImportError:  # pragma: no cover
    rclpy = None

from simple_navigator.math_utils import quaternion_from_yaw


@unittest.skipIf(rclpy is None, "ROS2 Python packages are not available")
class TestNavigatorIntegration(unittest.TestCase):
    class Collector(Node):
        def __init__(self):
            super().__init__("integration_collector")
            self.cmd_vel_msgs = []
            self.goal_reached_msgs = []
            self.received_goal_reached = threading.Event()

            self.create_subscription(Twist, "cmd_vel", self._cmd_cb, 10)
            self.create_subscription(Bool, "goal_reached", self._goal_cb, 10)

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        def _cmd_cb(self, msg: Twist):
            self.cmd_vel_msgs.append(msg)

        def _goal_cb(self, msg: Bool):
            self.goal_reached_msgs.append(msg)
            if msg.data:
                self.received_goal_reached.set()

        def clear(self):
            self.cmd_vel_msgs.clear()
            self.goal_reached_msgs.clear()
            self.received_goal_reached.clear()

        def pose(self):
            transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
                yaw,
            )

    @classmethod
    def setUpClass(cls):
        from simple_navigator.mock_robot import MockRobotNode
        from simple_navigator.navigator_node import NavigatorNode

        try:
            rclpy.init()
            cls.executor = MultiThreadedExecutor()
            cls.mock_robot = MockRobotNode()
            cls.navigator = NavigatorNode()
            cls.collector = cls.Collector()
        except Exception as exc:  # pragma: no cover
            try:
                rclpy.shutdown()
            except Exception:
                pass
            raise unittest.SkipTest(f"ROS integration unavailable in this environment: {exc}")

        cls.executor.add_node(cls.mock_robot)
        cls.executor.add_node(cls.navigator)
        cls.executor.add_node(cls.collector)

        cls.executor_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.executor_thread.start()
        time.sleep(0.5)

    @classmethod
    def tearDownClass(cls):
        cls.navigator.destroy_node()
        cls.mock_robot.destroy_node()
        cls.collector.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.collector.clear()
        self.navigator.target_state = None
        self.navigator.active_trajectory = None
        self.navigator.trajectory_start_time = None
        self.navigator.goal_reached_sent = False
        self.navigator.last_stop_sent = False
        self.navigator.stop_robot()
        self.mock_robot.x = 0.0
        self.mock_robot.y = 0.0
        self.mock_robot.yaw = 0.0
        self.mock_robot.vx = 0.0
        self.mock_robot.vy = 0.0
        self.mock_robot.vyaw = 0.0
        time.sleep(0.3)

    def _publish_target(self, x: float, y: float, yaw: float) -> None:
        pub = self.collector.create_publisher(PoseStamped, "goal_pose", 10)
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.pose.position.x = x
        msg.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        pub.publish(msg)
        time.sleep(0.3)

    def test_goal_pose_creates_trajectory(self):
        self._publish_target(1.0, 0.5, 0.7)
        self.assertIsNotNone(self.navigator.target_state)
        self.assertIsNotNone(self.navigator.active_trajectory)
        self.assertAlmostEqual(self.navigator.target_state.x, 1.0, delta=0.02)
        self.assertAlmostEqual(self.navigator.target_state.y, 0.5, delta=0.02)

    def test_robot_reaches_target_pose(self):
        self._publish_target(0.6, 0.0, 0.6)
        deadline = time.time() + 8.0
        last_pose = None
        while time.time() < deadline:
            try:
                last_pose = self.collector.pose()
            except Exception:
                time.sleep(0.05)
                continue

            x, y, yaw = last_pose
            position_error = math.hypot(x - 0.6, y)
            yaw_error = abs(math.atan2(math.sin(yaw - 0.6), math.cos(yaw - 0.6)))
            if position_error < 0.1 and yaw_error < 0.15:
                break
            time.sleep(0.05)

        self.assertIsNotNone(last_pose)
        x, y, yaw = last_pose
        self.assertLess(math.hypot(x - 0.6, y), 0.1)
        self.assertLess(abs(math.atan2(math.sin(yaw - 0.6), math.cos(yaw - 0.6))), 0.15)
        if self.collector.cmd_vel_msgs:
            time.sleep(0.5)
            final_cmd = self.collector.cmd_vel_msgs[-1]
            self.assertLess(math.hypot(final_cmd.linear.x, final_cmd.linear.y), 0.12)

    def test_translation_and_rotation_happen_together(self):
        self._publish_target(0.8, 0.0, math.pi / 2)
        deadline = time.time() + 2.0
        saw_combined_motion = False
        while time.time() < deadline:
            if self.collector.cmd_vel_msgs:
                cmd = self.collector.cmd_vel_msgs[-1]
                if abs(cmd.linear.x) > 0.02 and abs(cmd.angular.z) > 0.02:
                    saw_combined_motion = True
                    break
            time.sleep(0.05)
        self.assertTrue(saw_combined_motion)


if __name__ == "__main__":
    unittest.main()
