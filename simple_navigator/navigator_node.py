import math
import os
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from .config_loader import load_navigator_config
from .controller import RobotState, TrajectoryTracker
from .math_utils import clamp, normalize_angle, yaw_from_quaternion
from .trajectory import SE2Trajectory


class NavigatorNode(Node):
    def __init__(self):
        super().__init__("navigator")

        self.declare_parameter("config_file", "")
        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("position_tolerance", 0.05)
        self.declare_parameter("yaw_tolerance", 0.05)
        self.declare_parameter("max_vx", 0.4)
        self.declare_parameter("max_vy", 0.4)
        self.declare_parameter("max_vyaw", 1.0)
        self.declare_parameter("trajectory_duration_scale", 1.2)
        self.declare_parameter("trajectory_min_duration", 0.3)
        self.declare_parameter("debug_logging", True)
        self.declare_parameter("debug_log_period", 0.5)

        self.config_file = self.get_parameter("config_file").value
        self.control_frequency = float(self.get_parameter("control_frequency").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_vyaw = float(self.get_parameter("max_vyaw").value)
        self.trajectory_duration_scale = float(
            self.get_parameter("trajectory_duration_scale").value
        )
        self.trajectory_min_duration = float(
            self.get_parameter("trajectory_min_duration").value
        )
        self.debug_logging = bool(self.get_parameter("debug_logging").value)
        self.debug_log_period = float(self.get_parameter("debug_log_period").value)

        self.controller = TrajectoryTracker(
            max_linear_velocity=min(self.max_vx, self.max_vy),
            max_angular_velocity=self.max_vyaw,
        )
        self.target_state: Optional[RobotState] = None
        self.active_trajectory: Optional[SE2Trajectory] = None
        self.trajectory_start_time = None
        self.goal_reached_sent = False
        self.last_stop_sent = False
        self.last_debug_log_time = None

        self._load_config(self.config_file)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.goal_reached_pub = self.create_publisher(Bool, "goal_reached", 10)
        self.target_sub = self.create_subscription(
            PoseStamped, "goal_pose", self.target_pose_callback, 10
        )
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_loop
        )

        self.get_logger().info(
            "Minimal trajectory navigator ready "
            f"(odom_frame={self.odom_frame}, base_frame={self.base_frame}, "
            f"max_v=({self.max_vx:.2f}, {self.max_vy:.2f}), max_w={self.max_vyaw:.2f})"
        )

    def _apply_controller_limits(self, controller_config: dict) -> None:
        max_linear_velocity = controller_config.get("max_linear_velocity")
        max_angular_velocity = controller_config.get("max_angular_velocity")
        max_vx = controller_config.get("max_vx")
        max_vy = controller_config.get("max_vy")

        if max_linear_velocity is not None:
            linear_limit = float(max_linear_velocity)
            self.max_vx = linear_limit
            self.max_vy = linear_limit

        if max_vx is not None:
            self.max_vx = float(max_vx)

        if max_vy is not None:
            self.max_vy = float(max_vy)

        if max_angular_velocity is not None:
            self.max_vyaw = float(max_angular_velocity)

        self.controller.max_linear_velocity = min(self.max_vx, self.max_vy)
        self.controller.max_angular_velocity = self.max_vyaw

    def _default_config_path(self) -> str:
        return os.path.join(os.path.dirname(__file__), "..", "config", "target.yaml")

    def _load_config(self, config_file: str) -> None:
        path = config_file if config_file and os.path.exists(config_file) else None
        if path is None:
            default_path = self._default_config_path()
            if os.path.exists(default_path):
                path = default_path
        if path is None:
            return

        config = load_navigator_config(path)
        self._apply_controller_limits(config.controller)
        self.controller.set_parameters(**config.controller)

        navigation = config.navigation
        self.position_tolerance = float(
            navigation.get("position_tolerance", self.position_tolerance)
        )
        self.yaw_tolerance = float(navigation.get("yaw_tolerance", self.yaw_tolerance))
        self.control_frequency = float(
            navigation.get("control_frequency", self.control_frequency)
        )

        frames = config.frames
        self.base_frame = frames.get("base_frame", self.base_frame)
        self.odom_frame = frames.get("odom_frame", self.odom_frame)

        trajectory = config.trajectory
        self.trajectory_duration_scale = float(
            trajectory.get("duration_scale", self.trajectory_duration_scale)
        )
        self.trajectory_min_duration = float(
            trajectory.get("min_duration", self.trajectory_min_duration)
        )
        self.debug_logging = bool(
            trajectory.get("debug_logging", self.debug_logging)
        )
        self.debug_log_period = float(
            trajectory.get("debug_log_period", self.debug_log_period)
        )

        if config.target is not None:
            self.target_state = RobotState(
                x=config.target.x,
                y=config.target.y,
                yaw=normalize_angle(config.target.yaw),
            )

        self.get_logger().info(
            f"Loaded config from {path}"
            + (
                f" with target=({self.target_state.x:.2f}, {self.target_state.y:.2f}, {self.target_state.yaw:.2f})"
                if self.target_state is not None
                else " without initial target"
            )
            + f", limits=({self.max_vx:.2f}, {self.max_vy:.2f}, {self.max_vyaw:.2f})"
        )

    def get_current_state(self) -> Optional[RobotState]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
            q = transform.transform.rotation
            return RobotState(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                yaw=normalize_angle(yaw_from_quaternion(q.x, q.y, q.z, q.w)),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to get robot pose: {exc}",
                throttle_duration_sec=1.0,
            )
            return None

    def transform_target_to_odom(
        self, msg: PoseStamped
    ) -> Optional[RobotState]:
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            q = msg.pose.orientation
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            source_frame = msg.header.frame_id or self.odom_frame

            if source_frame == self.odom_frame:
                return RobotState(x=x, y=y, yaw=normalize_angle(yaw))

            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                source_frame,
                rclpy.time.Time(),
            )
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tq = transform.transform.rotation
            tyaw = yaw_from_quaternion(tq.x, tq.y, tq.z, tq.w)
            c = math.cos(tyaw)
            s = math.sin(tyaw)

            return RobotState(
                x=tx + c * x - s * y,
                y=ty + s * x + c * y,
                yaw=normalize_angle(yaw + tyaw),
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to transform target pose: {exc}")
            return None

    def _start_trajectory(self, current_state: RobotState, target_state: RobotState) -> None:
        self.active_trajectory = SE2Trajectory.from_states(
            start=current_state,
            goal=target_state,
            max_trans_vel=max(min(self.max_vx, self.max_vy), 1e-3),
            max_rot_vel=max(self.max_vyaw, 1e-3),
            duration_scale=self.trajectory_duration_scale,
            min_duration=self.trajectory_min_duration,
        )
        self.trajectory_start_time = self.get_clock().now()
        self.goal_reached_sent = False
        self.last_debug_log_time = None
        self.get_logger().info(
            "Started trajectory "
            f"from ({current_state.x:.2f}, {current_state.y:.2f}, {current_state.yaw:.2f}) "
            f"to ({target_state.x:.2f}, {target_state.y:.2f}, {target_state.yaw:.2f}) "
            f"duration={self.active_trajectory.duration:.2f}s"
        )

    def target_pose_callback(self, msg: PoseStamped) -> None:
        target_state = self.transform_target_to_odom(msg)
        if target_state is None:
            return

        self.target_state = target_state
        self.active_trajectory = None
        self.trajectory_start_time = None
        self.goal_reached_sent = False

        current_state = self.get_current_state()
        if current_state is not None:
            self._start_trajectory(current_state, self.target_state)

        self.get_logger().info(
            f"New target: x={target_state.x:.2f}, y={target_state.y:.2f}, yaw={target_state.yaw:.2f}"
        )

    def publish_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = vyaw
        self.cmd_vel_pub.publish(msg)
        self.last_stop_sent = False

    def stop_robot(self) -> None:
        if self.last_stop_sent:
            return
        self.publish_velocity(0.0, 0.0, 0.0)
        self.last_stop_sent = True

    def publish_goal_reached(self) -> None:
        msg = Bool()
        msg.data = True
        self.goal_reached_pub.publish(msg)
        if self.target_state is not None:
            self.get_logger().info(
                f"Goal reached at x={self.target_state.x:.2f}, y={self.target_state.y:.2f}, yaw={self.target_state.yaw:.2f}"
            )

    def maybe_log_debug(
        self,
        current_state: RobotState,
        reference,
        cmd: Twist | object,
        elapsed: float,
    ) -> None:
        if not self.debug_logging:
            return
        now = self.get_clock().now()
        if self.last_debug_log_time is not None:
            delta = (now - self.last_debug_log_time).nanoseconds / 1e9
            if delta < self.debug_log_period:
                return
        self.last_debug_log_time = now
        self.get_logger().debug(
            "track "
            f"t={elapsed:.2f}s "
            f"pose=({current_state.x:.2f}, {current_state.y:.2f}, {current_state.yaw:.2f}) "
            f"ref=({reference.x:.2f}, {reference.y:.2f}, {reference.yaw:.2f}) "
            f"cmd=({cmd.vx:.2f}, {cmd.vy:.2f}, {cmd.vyaw:.2f})"
        )

    def control_loop(self) -> None:
        if self.target_state is None:
            self.stop_robot()
            return

        current_state = self.get_current_state()
        if current_state is None:
            self.stop_robot()
            return

        if self.controller.is_goal_reached(
            current_state,
            self.target_state,
            self.position_tolerance,
            self.yaw_tolerance,
        ):
            if not self.goal_reached_sent:
                self.publish_goal_reached()
                self.goal_reached_sent = True
            self.active_trajectory = None
            self.trajectory_start_time = None
            self.stop_robot()
            return

        if self.active_trajectory is None:
            self._start_trajectory(current_state, self.target_state)

        elapsed = (
            (self.get_clock().now() - self.trajectory_start_time).nanoseconds / 1e9
            if self.trajectory_start_time is not None
            else 0.0
        )
        reference = self.active_trajectory.sample(elapsed)
        cmd = self.controller.compute_command(current_state, reference)
        self.maybe_log_debug(current_state, reference, cmd, elapsed)

        self.publish_velocity(
            clamp(cmd.vx, -self.max_vx, self.max_vx),
            clamp(cmd.vy, -self.max_vy, self.max_vy),
            clamp(cmd.vyaw, -self.max_vyaw, self.max_vyaw),
        )

    def destroy_node(self):
        self.stop_robot()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
