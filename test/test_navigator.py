#!/usr/bin/env python3

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simple_navigator.config_loader import load_navigator_config
from simple_navigator.controller import RobotState, TrajectoryTracker, VelocityCommand
from simple_navigator.math_utils import normalize_angle, quaternion_from_yaw, smoothstep5
from simple_navigator.navigator_node import NavigatorNode
from simple_navigator.trajectory import SE2Trajectory, TrajectoryPoint

try:
    from simple_navigator.modern_waypoint_editor import build_target_pose
except Exception:  # pragma: no cover
    build_target_pose = None


class TestMathUtils(unittest.TestCase):
    def test_normalize_angle(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi, places=5)
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), -math.pi, places=5)

    def test_quaternion_from_yaw(self):
        qx, qy, qz, qw = quaternion_from_yaw(math.pi / 2)
        self.assertAlmostEqual(qx, 0.0, places=6)
        self.assertAlmostEqual(qy, 0.0, places=6)
        self.assertAlmostEqual(qz, math.sqrt(0.5), places=6)
        self.assertAlmostEqual(qw, math.sqrt(0.5), places=6)

    def test_smoothstep5(self):
        self.assertAlmostEqual(smoothstep5(0.0), 0.0, places=6)
        self.assertAlmostEqual(smoothstep5(0.5), 0.5, places=6)
        self.assertAlmostEqual(smoothstep5(1.0), 1.0, places=6)


class TestConfigLoader(unittest.TestCase):
    def test_load_target_config(self):
        config = load_navigator_config(
            os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "config",
                "target.yaml",
            )
        )
        self.assertIsNotNone(config.target)
        self.assertAlmostEqual(config.target.x, 1.0, places=6)
        self.assertAlmostEqual(config.target.yaw, 1.57, places=6)
        self.assertIn("kp_x", config.controller)
        self.assertTrue(config.trajectory["debug_logging"])

    def test_load_all_example_target_configs(self):
        config_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config",
        )
        for filename in ("target.yaml", "target_origin.yaml", "target_diag.yaml", "target_spin.yaml"):
            with self.subTest(filename=filename):
                config = load_navigator_config(os.path.join(config_dir, filename))
                self.assertIsNotNone(config.target)


class TestNavigatorConfigApplication(unittest.TestCase):
    def _make_navigator_stub(self):
        node = NavigatorNode.__new__(NavigatorNode)
        node.max_vx = 0.4
        node.max_vy = 0.4
        node.max_vyaw = 1.0
        node.controller = type(
            "ControllerStub",
            (),
            {"max_linear_velocity": 0.4, "max_angular_velocity": 1.0},
        )()
        return node

    def test_controller_linear_limit_feeds_navigation_limits(self):
        node = self._make_navigator_stub()
        node._apply_controller_limits(
            {
                "kp_x": 2.0,
                "kp_y": 2.0,
                "kp_yaw": 3.0,
                "max_linear_velocity": 1.2,
                "max_angular_velocity": 2.4,
            }
        )
        self.assertAlmostEqual(node.max_vx, 1.2, places=6)
        self.assertAlmostEqual(node.max_vy, 1.2, places=6)
        self.assertAlmostEqual(node.max_vyaw, 2.4, places=6)
        self.assertAlmostEqual(node.controller.max_linear_velocity, 1.2, places=6)
        self.assertAlmostEqual(node.controller.max_angular_velocity, 2.4, places=6)

    def test_axis_specific_limits_override_shared_linear_limit(self):
        node = self._make_navigator_stub()
        node._apply_controller_limits(
            {
                "max_linear_velocity": 1.2,
                "max_vx": 1.8,
                "max_vy": 0.9,
                "max_angular_velocity": 2.1,
            }
        )
        self.assertAlmostEqual(node.max_vx, 1.8, places=6)
        self.assertAlmostEqual(node.max_vy, 0.9, places=6)
        self.assertAlmostEqual(node.max_vyaw, 2.1, places=6)
        self.assertAlmostEqual(node.controller.max_linear_velocity, 0.9, places=6)


class TestControllerPrimitives(unittest.TestCase):
    def test_robot_state_distance(self):
        state = RobotState(x=0.0, y=0.0, yaw=0.0)
        self.assertAlmostEqual(state.distance_to(3.0, 4.0), 5.0, places=5)

    def test_robot_state_angle_wrapping(self):
        state = RobotState(x=0.0, y=0.0, yaw=math.pi - 0.1)
        self.assertAlmostEqual(abs(state.angle_to(-math.pi + 0.1)), 0.2, places=3)

    def test_velocity_command_tuple(self):
        self.assertEqual(VelocityCommand(1.0, 2.0, 0.5).to_tuple(), (1.0, 2.0, 0.5))


class TestTrajectory(unittest.TestCase):
    def test_final_sample_marks_done(self):
        traj = SE2Trajectory(0.0, 0.0, 0.0, 1.0, 2.0, 1.0, duration=2.0)
        point = traj.sample(2.0)
        self.assertTrue(point.done)
        self.assertAlmostEqual(point.x, 1.0, places=6)
        self.assertAlmostEqual(point.y, 2.0, places=6)
        self.assertAlmostEqual(point.yaw, 1.0, places=6)

    def test_midpoint_stays_on_translation_line(self):
        traj = SE2Trajectory(0.0, 0.0, 0.0, 2.0, 1.0, math.pi / 2, duration=4.0)
        point = traj.sample(2.0)
        self.assertAlmostEqual(point.y / point.x, 0.5, places=3)
        self.assertGreater(point.vx, 0.0)
        self.assertGreater(point.vy, 0.0)
        self.assertGreater(point.wz, 0.0)

    def test_duration_from_states(self):
        start = RobotState(0.0, 0.0, 0.0)
        goal = RobotState(1.0, 0.0, math.pi / 2)
        traj = SE2Trajectory.from_states(
            start=start,
            goal=goal,
            max_trans_vel=0.5,
            max_rot_vel=1.0,
            duration_scale=1.2,
            min_duration=0.3,
        )
        self.assertAlmostEqual(traj.duration, 2.4, places=3)


class TestTrajectoryTracker(unittest.TestCase):
    def setUp(self):
        self.tracker = TrajectoryTracker(
            kx=1.0,
            ky=1.0,
            kyaw=1.0,
            max_linear_velocity=0.5,
            max_angular_velocity=1.0,
        )

    def test_compute_command_uses_feedforward(self):
        state = RobotState(x=0.0, y=0.0, yaw=0.0)
        ref = TrajectoryPoint(x=0.0, y=0.0, yaw=0.0, vx=0.2, vy=0.1, wz=0.3)
        cmd = self.tracker.compute_command(state, ref)
        self.assertAlmostEqual(cmd.vx, 0.2, places=4)
        self.assertAlmostEqual(cmd.vy, 0.1, places=4)
        self.assertAlmostEqual(cmd.vyaw, 0.3, places=4)

    def test_compute_command_body_frame_error(self):
        state = RobotState(x=0.0, y=0.0, yaw=math.pi / 2)
        ref = TrajectoryPoint(x=1.0, y=0.0, yaw=math.pi / 2, vx=0.0, vy=0.0, wz=0.0)
        cmd = self.tracker.compute_command(state, ref)
        self.assertAlmostEqual(cmd.vx, 0.0, places=3)
        self.assertLess(cmd.vy, -0.4)

    def test_linear_velocity_is_norm_limited(self):
        state = RobotState(x=0.0, y=0.0, yaw=0.0)
        ref = TrajectoryPoint(x=10.0, y=10.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0)
        cmd = self.tracker.compute_command(state, ref)
        self.assertLessEqual(math.hypot(cmd.vx, cmd.vy), 0.5 + 1e-6)

    def test_goal_check(self):
        current = RobotState(x=0.01, y=-0.01, yaw=0.02)
        target = RobotState(x=0.0, y=0.0, yaw=0.0)
        self.assertTrue(self.tracker.is_goal_reached(current, target, 0.05, 0.05))


@unittest.skipIf(build_target_pose is None, "Qt or ROS dependencies unavailable")
class TestTargetGoalEditorHelper(unittest.TestCase):
    def test_build_target_pose(self):
        msg = build_target_pose(1.2, -0.5, 0.3)
        self.assertEqual(msg.header.frame_id, "odom")
        self.assertAlmostEqual(msg.pose.position.x, 1.2, places=6)
        self.assertAlmostEqual(msg.pose.position.y, -0.5, places=6)

    def test_build_target_pose_with_frame(self):
        msg = build_target_pose(0.2, 0.4, -0.1, frame_id="map")
        self.assertEqual(msg.header.frame_id, "map")


if __name__ == "__main__":
    unittest.main()
