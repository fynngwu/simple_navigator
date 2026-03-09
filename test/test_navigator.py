#!/usr/bin/env python3
"""Integration tests for simple_navigator package.

Tests the following components:
1. WaypointManager - waypoint CRUD operations
2. Controller - velocity computation and goal detection
3. SimpleController - PID controller logic

Run with:
    ros2 run simple_navigator test_integration
    python3 test/test_navigator.py
"""

import sys
import os
import math
import unittest
import tempfile

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simple_navigator.waypoint_manager import WaypointManager, Waypoint
from simple_navigator.controller import Controller, RobotState, VelocityCommand
from simple_navigator.simple_controller import SimpleController


class TestWaypointManager(unittest.TestCase):
    """Test waypoint manager functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.manager = WaypointManager()

    def test_add_waypoint(self):
        """Test adding a waypoint."""
        self.manager.add_waypoint("test_point", x=1.0, y=2.0, yaw=0.5)
        waypoint = self.manager.get_waypoint("test_point")

        self.assertIsNotNone(waypoint)
        self.assertEqual(waypoint.x, 1.0)
        self.assertEqual(waypoint.y, 2.0)
        self.assertEqual(waypoint.yaw, 0.5)

    def test_get_waypoint_not_found(self):
        """Test getting non-existent waypoint."""
        waypoint = self.manager.get_waypoint("nonexistent")
        self.assertIsNone(waypoint)

    def test_list_waypoints(self):
        """Test listing all waypoints."""
        self.manager.add_waypoint("point1", 0.0, 0.0, 0.0)
        self.manager.add_waypoint("point2", 1.0, 1.0, 1.0)

        waypoints = self.manager.list_waypoints()
        self.assertEqual(len(waypoints), 2)
        self.assertIn("point1", waypoints)
        self.assertIn("point2", waypoints)

    def test_remove_waypoint(self):
        """Test removing a waypoint."""
        self.manager.add_waypoint("temp", 0.0, 0.0, 0.0)
        self.assertTrue(self.manager.has_waypoint("temp"))

        self.manager.remove_waypoint("temp")
        self.assertFalse(self.manager.has_waypoint("temp"))

    def test_temporary_waypoint(self):
        """Test temporary waypoint functionality."""
        self.manager.add_waypoint("permanent", 1.0, 1.0, 0.0)
        self.manager.set_temporary_waypoint("temp", 2.0, 2.0, 0.5)

        # Both should exist
        self.assertTrue(self.manager.has_waypoint("permanent"))
        self.assertTrue(self.manager.has_waypoint("temp"))

        # Clear temporary
        self.manager.clear_temporary_waypoints()
        self.assertTrue(self.manager.has_waypoint("permanent"))
        self.assertFalse(self.manager.has_waypoint("temp"))


class TestController(unittest.TestCase):
    """Test controller base class."""

    def test_robot_state_distance(self):
        """Test distance calculation."""
        state = RobotState(x=0.0, y=0.0, yaw=0.0)
        distance = state.distance_to(3.0, 4.0)
        self.assertAlmostEqual(distance, 5.0, places=5)

    def test_robot_state_angle_normalization(self):
        """Test angle difference normalization."""
        state = RobotState(x=0.0, y=0.0, yaw=0.0)

        # Positive angle difference
        diff = state.angle_to(math.pi)  # pi - 0
        self.assertAlmostEqual(diff, math.pi, places=5)

        # Negative angle difference
        diff = state.angle_to(-math.pi)
        self.assertAlmostEqual(diff, -math.pi, places=5)

    def test_robot_state_angle_wrapping(self):
        """Test angle wrapping around pi."""
        state = RobotState(x=0.0, y=0.0, yaw=math.pi - 0.1)

        # Should wrap from 2*pi - 0.1 to -0.1
        diff = state.angle_to(-math.pi + 0.1)
        self.assertAlmostEqual(abs(diff), 0.2, places=1)

    def test_velocity_command_tuple(self):
        """Test velocity command conversion to tuple."""
        cmd = VelocityCommand(vx=1.0, vy=2.0, vyaw=0.5)
        t = cmd.to_tuple()
        self.assertEqual(t, (1.0, 2.0, 0.5))

    def test_is_goal_reached(self):
        """Test goal reached detection."""
        controller = SimpleController()

        # Robot at origin, target at origin
        state = RobotState(x=0.0, y=0.0, yaw=0.0)
        reached = controller.is_goal_reached(
            state, 0.0, 0.0, 0.0, position_tolerance=0.05, yaw_tolerance=0.05
        )
        self.assertTrue(reached)

        # Robot far from target
        state = RobotState(x=0.0, y=0.0, yaw=0.0)
        reached = controller.is_goal_reached(
            state, 10.0, 10.0, 0.0, position_tolerance=0.05, yaw_tolerance=0.05
        )
        self.assertFalse(reached)


class TestSimpleController(unittest.TestCase):
    """Test simple PID controller."""

    def setUp(self):
        """Set up test fixtures."""
        self.controller = SimpleController(
            kp_x=1.0,
            kp_y=1.0,
            kp_yaw=1.0,
            max_linear_velocity=0.5,
            max_angular_velocity=1.0,
        )

    def test_compute_velocity_straight_line(self):
        """Test velocity computation for straight line movement."""
        state = RobotState(x=0.0, y=0.0, yaw=0.0)

        # Target 1m ahead
        cmd = self.controller.compute_velocity(state, 1.0, 0.0, 0.0)

        self.assertAlmostEqual(cmd.vx, 0.5, places=1)  # limited by max_linear_velocity
        self.assertAlmostEqual(cmd.vy, 0.0, places=1)
        self.assertAlmostEqual(cmd.vyaw, 0.0, places=1)

    def test_compute_velocity_lateral(self):
        """Test velocity computation for lateral movement."""
        state = RobotState(x=0.0, y=0.0, yaw=0.0)

        # Target 1m to the right
        cmd = self.controller.compute_velocity(state, 0.0, 1.0, 0.0)

        self.assertAlmostEqual(cmd.vx, 0.0, places=1)
        self.assertAlmostEqual(cmd.vy, 0.5, places=1)

    def test_compute_velocity_rotation(self):
        """Test velocity computation for rotation."""
        # Create controller with higher angular velocity limit
        controller = SimpleController(
            kp_x=1.0,
            kp_y=1.0,
            kp_yaw=1.0,
            max_linear_velocity=0.5,
            max_angular_velocity=2.0,
        )
        state = RobotState(x=0.0, y=0.0, yaw=0.0)

        # Target 90 degrees rotation
        cmd = controller.compute_velocity(state, 0.0, 0.0, math.pi / 2)

        self.assertAlmostEqual(cmd.vx, 0.0, places=1)
        self.assertAlmostEqual(cmd.vy, 0.0, places=1)
        self.assertAlmostEqual(cmd.vyaw, 1.57, places=1)

    def test_velocity_limits(self):
        """Test velocity saturation."""
        # Reset controller
        self.controller.reset()

        # Create controller with very low limits
        controller = SimpleController(
            kp_x=10.0,  # High gain
            max_linear_velocity=0.2,
            max_angular_velocity=0.3,
        )

        state = RobotState(x=0.0, y=0.0, yaw=0.0)

        # Target far away - should be clamped
        cmd = controller.compute_velocity(state, 100.0, 100.0, 100.0)

        # Check velocity limits
        self.assertLessEqual(abs(cmd.vx), 0.2 + 0.001)
        self.assertLessEqual(abs(cmd.vy), 0.2 + 0.001)
        self.assertLessEqual(abs(cmd.vyaw), 0.3 + 0.001)

    def test_reset(self):
        """Test controller reset."""
        state = RobotState(x=0.0, y=0.0, yaw=0.0)

        # Compute some velocity to trigger integral term
        self.controller._integral_x = 10.0
        self.controller._integral_y = 10.0
        self.controller._prev_error_x = 5.0

        # Reset
        self.controller.reset()

        # Check integrals are cleared
        self.assertEqual(self.controller._integral_x, 0.0)
        self.assertEqual(self.controller._integral_y, 0.0)
        self.assertEqual(self.controller._prev_error_x, 0.0)

    def test_set_parameters(self):
        """Test dynamic parameter update."""
        original_kp = self.controller.kp_x

        self.controller.set_parameters(kp_x=5.0)

        self.assertEqual(self.controller.kp_x, 5.0)
        self.assertEqual(self.controller.kp_y, original_kp)  # Unchanged

    def test_from_config(self):
        """Test controller creation from config."""
        config = {
            "controller": {
                "kp_x": 2.0,
                "kp_y": 3.0,
                "kp_yaw": 1.5,
                "max_linear_velocity": 1.0,
                "max_angular_velocity": 2.0,
            }
        }

        controller = SimpleController.from_config(config)

        self.assertEqual(controller.kp_x, 2.0)
        self.assertEqual(controller.kp_y, 3.0)
        self.assertEqual(controller.kp_yaw, 1.5)
        self.assertEqual(controller.max_linear_velocity, 1.0)
        self.assertEqual(controller.max_angular_velocity, 2.0)

    def test_deadband(self):
        """Test deadband for small errors."""
        # Create controller with deadband
        controller = SimpleController(
            kp_x=1.0,
            linear_deadband=0.05,
        )

        # Small error within deadband
        state = RobotState(x=0.0, y=0.0, yaw=0.0)
        cmd = controller.compute_velocity(state, 0.03, 0.0, 0.0)

        # Should be zero due to deadband
        self.assertAlmostEqual(cmd.vx, 0.0, places=2)

    def test_navigation_simulation(self):
        """Simulate a complete navigation scenario."""
        # Reset controller
        self.controller.reset()

        # Robot starts at origin
        state = RobotState(x=0.0, y=0.0, yaw=0.0)

        # Target at (0.5, 0.5)
        target_x, target_y, target_yaw = 0.5, 0.5, 0.0

        iterations = 0
        max_iterations = 100

        while iterations < max_iterations:
            # Check if goal reached
            if self.controller.is_goal_reached(
                state,
                target_x,
                target_y,
                target_yaw,
                position_tolerance=0.05,
                yaw_tolerance=0.05,
            ):
                break

            # Compute velocity
            cmd = self.controller.compute_velocity(
                state, target_x, target_y, target_yaw
            )

            # Simulate movement (dt = 0.1s)
            dt = 0.1
            state.x += cmd.vx * dt
            state.y += cmd.vy * dt
            state.yaw += cmd.vyaw * dt

            iterations += 1

        # Should have reached goal
        self.assertTrue(
            self.controller.is_goal_reached(
                state,
                target_x,
                target_y,
                target_yaw,
                position_tolerance=0.05,
                yaw_tolerance=0.05,
            ),
            f"Failed to reach goal in {iterations} iterations",
        )


def run_tests():
    """Run all tests."""
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Add test cases
    suite.addTests(loader.loadTestsFromTestCase(TestWaypointManager))
    suite.addTests(loader.loadTestsFromTestCase(TestController))
    suite.addTests(loader.loadTestsFromTestCase(TestSimpleController))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(run_tests())
