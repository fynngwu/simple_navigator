"""Abstract Controller Interface for navigation control.

This module defines the abstract base class for navigation controllers,
allowing for future extensions like MPC, Pure Pursuit, etc.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple
import math


@dataclass
class RobotState:
    """Current state of the robot."""

    x: float
    y: float
    yaw: float

    def distance_to(self, target_x: float, target_y: float) -> float:
        """Calculate distance to target position."""
        return math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2)

    def angle_to(self, target_yaw: float) -> float:
        """Calculate angle difference to target yaw."""
        diff = target_yaw - self.yaw
        # Normalize to [-pi, pi]
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


@dataclass
class VelocityCommand:
    """Velocity command for holonomic robot."""

    vx: float = 0.0  # Linear velocity in x (m/s)
    vy: float = 0.0  # Linear velocity in y (m/s)
    vyaw: float = 0.0  # Angular velocity (rad/s)

    def to_tuple(self) -> Tuple[float, float, float]:
        """Convert to tuple (vx, vy, vyaw)."""
        return (self.vx, self.vy, self.vyaw)


class Controller(ABC):
    """Abstract base class for navigation controllers.

    This interface allows for different control strategies:
    - Simple PID controller
    - Model Predictive Control (MPC)
    - Pure Pursuit
    - Stanley controller
    - Custom controllers
    """

    @abstractmethod
    def compute_velocity(
        self,
        current_state: RobotState,
        target_x: float,
        target_y: float,
        target_yaw: float,
    ) -> VelocityCommand:
        """Compute velocity command to reach target waypoint.

        Args:
            current_state: Current robot state (x, y, yaw)
            target_x: Target x position (meters)
            target_y: Target y position (meters)
            target_yaw: Target orientation (radians)

        Returns:
            VelocityCommand with vx, vy, vyaw for holonomic robot
        """
        pass

    @abstractmethod
    def reset(self) -> None:
        """Reset controller state (useful for switching targets)."""
        pass

    @abstractmethod
    def set_parameters(self, **kwargs) -> None:
        """Update controller parameters.

        Allows dynamic parameter adjustment without recreating controller.

        Args:
            **kwargs: Parameter names and values (e.g., kp=2.0, max_velocity=1.0)
        """
        pass

    def is_goal_reached(
        self,
        current_state: RobotState,
        target_x: float,
        target_y: float,
        target_yaw: float,
        position_tolerance: float = 0.05,
        yaw_tolerance: float = 0.05,
    ) -> bool:
        """Check if robot has reached target waypoint.

        Args:
            current_state: Current robot state
            target_x: Target x position
            target_y: Target y position
            target_yaw: Target orientation
            position_tolerance: Position tolerance (meters)
            yaw_tolerance: Yaw tolerance (radians)

        Returns:
            True if goal is reached within tolerance
        """
        distance_error = current_state.distance_to(target_x, target_y)
        yaw_error = abs(current_state.angle_to(target_yaw))

        return distance_error < position_tolerance and yaw_error < yaw_tolerance
