"""Simple PID Controller for holonomic robot navigation.

This module implements a basic PID controller for holonomic robots,
suitable for direct waypoint navigation without path planning.
"""

from typing import Dict, Any
from .controller import Controller, RobotState, VelocityCommand


class SimpleController(Controller):
    """Simple PID-based controller for holonomic robots.

    Features:
    - Independent PID control for x, y, and yaw
    - Velocity saturation
    - Deadband for small errors
    - Extensible parameter interface
    """

    def __init__(
        self,
        kp_x: float = 1.0,
        kp_y: float = 1.0,
        kp_yaw: float = 1.0,
        ki_x: float = 0.0,
        ki_y: float = 0.0,
        ki_yaw: float = 0.0,
        kd_x: float = 0.0,
        kd_y: float = 0.0,
        kd_yaw: float = 0.0,
        max_linear_velocity: float = 0.5,
        max_angular_velocity: float = 1.0,
        linear_deadband: float = 0.01,
        angular_deadband: float = 0.01,
    ):
        """Initialize the PID controller.

        Args:
            kp_x, kp_y, kp_yaw: Proportional gains
            ki_x, ki_y, ki_yaw: Integral gains
            kd_x, kd_y, kd_yaw: Derivative gains
            max_linear_velocity: Maximum linear velocity (m/s)
            max_angular_velocity: Maximum angular velocity (rad/s)
            linear_deadband: Deadband for linear errors (meters)
            angular_deadband: Deadband for angular errors (radians)
        """
        # Gains
        self.kp_x = kp_x
        self.kp_y = kp_y
        self.kp_yaw = kp_yaw
        self.ki_x = ki_x
        self.ki_y = ki_y
        self.ki_yaw = ki_yaw
        self.kd_x = kd_x
        self.kd_y = kd_y
        self.kd_yaw = kd_yaw

        # Limits
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.linear_deadband = linear_deadband
        self.angular_deadband = angular_deadband

        # State
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._integral_yaw = 0.0
        self._prev_error_x = 0.0
        self._prev_error_y = 0.0
        self._prev_error_yaw = 0.0

    def compute_velocity(
        self,
        current_state: RobotState,
        target_x: float,
        target_y: float,
        target_yaw: float,
    ) -> VelocityCommand:
        """Compute velocity command using PID control.

        Args:
            current_state: Current robot state
            target_x: Target x position
            target_y: Target y position
            target_yaw: Target orientation

        Returns:
            VelocityCommand with computed velocities
        """
        # Calculate errors
        error_x = target_x - current_state.x
        error_y = target_y - current_state.y
        error_yaw = current_state.angle_to(target_yaw)

        # Apply deadband
        if abs(error_x) < self.linear_deadband:
            error_x = 0.0
        if abs(error_y) < self.linear_deadband:
            error_y = 0.0
        if abs(error_yaw) < self.angular_deadband:
            error_yaw = 0.0

        # Update integrals
        self._integral_x += error_x
        self._integral_y += error_y
        self._integral_yaw += error_yaw

        # Calculate derivatives
        derivative_x = error_x - self._prev_error_x
        derivative_y = error_y - self._prev_error_y
        derivative_yaw = error_yaw - self._prev_error_yaw

        # Store previous errors
        self._prev_error_x = error_x
        self._prev_error_y = error_y
        self._prev_error_yaw = error_yaw

        # PID output
        vx = (
            self.kp_x * error_x
            + self.ki_x * self._integral_x
            + self.kd_x * derivative_x
        )
        vy = (
            self.kp_y * error_y
            + self.ki_y * self._integral_y
            + self.kd_y * derivative_y
        )
        vyaw = (
            self.kp_yaw * error_yaw
            + self.ki_yaw * self._integral_yaw
            + self.kd_yaw * derivative_yaw
        )

        # Apply velocity saturation
        vx = self._clamp(vx, -self.max_linear_velocity, self.max_linear_velocity)
        vy = self._clamp(vy, -self.max_linear_velocity, self.max_linear_velocity)
        vyaw = self._clamp(vyaw, -self.max_angular_velocity, self.max_angular_velocity)

        return VelocityCommand(vx=vx, vy=vy, vyaw=vyaw)

    def reset(self) -> None:
        """Reset controller state."""
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._integral_yaw = 0.0
        self._prev_error_x = 0.0
        self._prev_error_y = 0.0
        self._prev_error_yaw = 0.0

    def set_parameters(self, **kwargs) -> None:
        """Update controller parameters.

        Args:
            **kwargs: Parameter names and values
        """
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """Clamp value between min and max."""
        return max(min_val, min(max_val, value))

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "SimpleController":
        """Create controller from configuration dictionary.

        Args:
            config: Configuration dictionary with controller parameters

        Returns:
            Configured SimpleController instance
        """
        controller_config = config.get("controller", {})

        return cls(
            kp_x=controller_config.get("kp_x", 1.0),
            kp_y=controller_config.get("kp_y", 1.0),
            kp_yaw=controller_config.get("kp_yaw", 1.0),
            max_linear_velocity=controller_config.get("max_linear_velocity", 0.5),
            max_angular_velocity=controller_config.get("max_angular_velocity", 1.0),
            linear_deadband=controller_config.get("linear_deadband", 0.01),
            angular_deadband=controller_config.get("angular_deadband", 0.01),
        )
