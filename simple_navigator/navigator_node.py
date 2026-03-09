#!/usr/bin/env python3
"""Main Navigator Node for ROS2 navigation.

This module implements the main navigation node that:
- Listens for 'go' signals
- Gets current position from TF
- Navigates to waypoints using velocity commands
- Provides services for waypoint management
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from typing import Optional
import yaml
import os
import math

from .waypoint_manager import WaypointManager, Waypoint
from .controller import Controller, RobotState
from .simple_controller import SimpleController

# Import for TF transformations
import transforms3d.euler as tf_transformations


class NavigatorNode(Node):
    """Main navigation node for holonomic robots.

    This node implements a simple navigation system that:
    1. Waits for a 'go' signal
    2. Navigates to the specified waypoint
    3. Stops and waits for next 'go' signal

    Features:
    - TF-based localization
    - Holonomic velocity control
    - Configurable waypoints via YAML
    - Service interface for temporary waypoints
    - Extensible controller architecture
    """

    def __init__(self):
        super().__init__("navigator_node")

        self.declare_parameter("config_file", "")
        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("position_tolerance", 0.05)
        self.declare_parameter("yaw_tolerance", 0.05)
        self.declare_parameter("default_waypoint", "home")

        config_file = self.get_parameter("config_file").value
        self.control_frequency = self.get_parameter("control_frequency").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.position_tolerance = self.get_parameter("position_tolerance").value
        self.yaw_tolerance = self.get_parameter("yaw_tolerance").value
        self.default_waypoint = self.get_parameter("default_waypoint").value

        self.waypoint_manager = WaypointManager()

        if config_file and os.path.exists(config_file):
            self._load_config(config_file)
        else:
            default_config = os.path.join(
                os.path.dirname(__file__), "..", "config", "waypoints.yaml"
            )
            if os.path.exists(default_config):
                self._load_config(default_config)
                self.get_logger().info(f"Loaded default config from {default_config}")

        # Initialize controller with config if available
        self.controller: Optional[Controller] = None
        if config_file and os.path.exists(config_file):
            with open(config_file, "r") as f:
                config = yaml.safe_load(f)
            self.controller = SimpleController.from_config(config)
        else:
            self.controller = SimpleController()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_state: Optional[RobotState] = None
        self.target_waypoint: Optional[Waypoint] = None
        self.is_navigating = False
        self.go_received = False

        self.callback_group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.goal_reached_pub = self.create_publisher(Bool, "goal_reached", 10)

        self.go_sub = self.create_subscription(Bool, "go", self.go_callback, 10)

        self.target_sub = self.create_subscription(
            PoseStamped, "target_pose", self.target_pose_callback, 10
        )

        self.set_waypoint_srv = self.create_service(
            SetBool,
            "set_temp_waypoint",
            self.set_temp_waypoint_callback,
            callback_group=self.callback_group,
        )

        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_loop
        )

        self.get_logger().info("Navigator node initialized")
        self.get_logger().info(
            f"Available waypoints: {list(self.waypoint_manager.list_waypoints().keys())}"
        )

    def _load_config(self, config_file: str) -> None:
        """Load configuration from YAML file.

        Args:
            config_file: Path to YAML configuration file
        """
        try:
            with open(config_file, "r") as f:
                config = yaml.safe_load(f)

            if "waypoints" in config:
                for name, data in config["waypoints"].items():
                    self.waypoint_manager.add_waypoint(
                        name=name,
                        x=float(data.get("x", 0.0)),
                        y=float(data.get("y", 0.0)),
                        yaw=math.radians(float(data.get("yaw", 0.0))),
                    )

            if "controller" in config and self.controller:
                self.controller.set_parameters(**config["controller"])

            if "navigation" in config:
                nav_config = config["navigation"]
                if "position_tolerance" in nav_config:
                    self.position_tolerance = nav_config["position_tolerance"]
                if "yaw_tolerance" in nav_config:
                    self.yaw_tolerance = nav_config["yaw_tolerance"]
                if "control_frequency" in nav_config:
                    self.control_frequency = nav_config["control_frequency"]

            self.get_logger().info(f"Loaded config from {config_file}")

        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")

    def get_current_state(self) -> Optional[RobotState]:
        """Get current robot state from TF.

        Returns:
            Current robot state or None if TF not available
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation
            yaw = tf_transformations.quat2euler([q.x, q.y, q.z, q.w])[2]

            return RobotState(x=x, y=y, yaw=yaw)

        except Exception as e:
            self.get_logger().warn(
                f"Failed to get current state: {e}", throttle_duration_sec=1.0
            )
            return None

    def go_callback(self, msg: Bool) -> None:
        """Callback for 'go' signal.

        Args:
            msg: Bool message (True = start navigation)
        """
        if msg.data:
            if self.target_waypoint is None:
                self.target_waypoint = self.waypoint_manager.get_waypoint(
                    self.default_waypoint
                )

            if self.target_waypoint:
                self.get_logger().info(
                    f"Go signal received, navigating to {self.target_waypoint.name}"
                )
                self.is_navigating = True
                self.go_received = True
                self.controller.reset()
            else:
                self.get_logger().warn("No target waypoint set!")

    def target_pose_callback(self, msg: PoseStamped) -> None:
        """Callback for target pose message.

        Args:
            msg: Target pose as PoseStamped
        """
        x = msg.pose.position.x
        y = msg.pose.position.y

        q = msg.pose.orientation
        yaw = tf_transformations.quat2euler([q.x, q.y, q.z, q.w])[2]

        self.waypoint_manager.set_temporary_waypoint("target", x, y, yaw)
        self.target_waypoint = self.waypoint_manager.get_waypoint("target")

        self.get_logger().info(f"Target pose set: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

    def set_temp_waypoint_callback(self, request, response):
        """Service callback to set temporary waypoint.

        This is a placeholder - in practice, you'd create a custom service
        definition that accepts x, y, yaw parameters.

        Args:
            request: Service request
            response: Service response

        Returns:
            Service response
        """
        response.success = True
        response.message = "Use target_pose topic to set waypoints"
        return response

    def control_loop(self) -> None:
        """Main control loop."""
        self.current_state = self.get_current_state()

        if self.current_state is None:
            self.stop_robot()
            return

        if not self.is_navigating or self.target_waypoint is None:
            self.stop_robot()
            return

        if self.controller.is_goal_reached(
            self.current_state,
            self.target_waypoint.x,
            self.target_waypoint.y,
            self.target_waypoint.yaw,
            self.position_tolerance,
            self.yaw_tolerance,
        ):
            self.get_logger().info(f"Reached waypoint: {self.target_waypoint.name}")
            self.is_navigating = False
            self.go_received = False
            self.stop_robot()

            goal_reached_msg = Bool()
            goal_reached_msg.data = True
            self.goal_reached_pub.publish(goal_reached_msg)
            return

        cmd = self.controller.compute_velocity(
            self.current_state,
            self.target_waypoint.x,
            self.target_waypoint.y,
            self.target_waypoint.yaw,
        )

        self.publish_velocity(cmd.vx, cmd.vy, cmd.vyaw)

    def publish_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        """Publish velocity command.

        Args:
            vx: Linear velocity in x (m/s)
            vy: Linear velocity in y (m/s)
            vyaw: Angular velocity (rad/s)
        """
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = vyaw

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self) -> None:
        """Stop the robot by publishing zero velocity."""
        self.publish_velocity(0.0, 0.0, 0.0)

    def set_target_waypoint(self, name: str) -> bool:
        """Set target waypoint by name.

        Args:
            name: Waypoint name

        Returns:
            True if waypoint exists, False otherwise
        """
        waypoint = self.waypoint_manager.get_waypoint(name)
        if waypoint:
            self.target_waypoint = waypoint
            self.get_logger().info(f"Target waypoint set: {name}")
            return True
        else:
            self.get_logger().warn(f"Waypoint not found: {name}")
            return False

    def add_waypoint(
        self, name: str, x: float, y: float, yaw: float, temporary: bool = False
    ) -> None:
        """Add a new waypoint.

        Args:
            name: Waypoint name
            x: X position
            y: Y position
            yaw: Orientation
            temporary: If True, waypoint is temporary
        """
        self.waypoint_manager.add_waypoint(name, x, y, yaw, temporary)
        self.get_logger().info(f"Added waypoint: {name}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = NavigatorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
