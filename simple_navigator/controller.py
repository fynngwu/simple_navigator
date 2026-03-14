from dataclasses import dataclass
import math

from .math_utils import clamp, normalize_angle
from .trajectory import TrajectoryPoint


@dataclass
class RobotState:
    x: float
    y: float
    yaw: float

    def distance_to(self, target_x: float, target_y: float) -> float:
        return math.hypot(target_x - self.x, target_y - self.y)

    def angle_to(self, target_yaw: float) -> float:
        return normalize_angle(target_yaw - self.yaw)


@dataclass
class VelocityCommand:
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0

    def to_tuple(self) -> tuple[float, float, float]:
        return (self.vx, self.vy, self.vyaw)


class TrajectoryTracker:
    def __init__(
        self,
        kx: float = 1.5,
        ky: float = 1.5,
        kyaw: float = 2.0,
        max_linear_velocity: float = 0.8,
        max_angular_velocity: float = 1.5,
    ) -> None:
        self.kx = kx
        self.ky = ky
        self.kyaw = kyaw
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity

    def set_parameters(self, **kwargs) -> None:
        aliases = {
            "kx": "kx",
            "kp_x": "kx",
            "ky": "ky",
            "kp_y": "ky",
            "kyaw": "kyaw",
            "kp_yaw": "kyaw",
            "max_linear_velocity": "max_linear_velocity",
            "max_angular_velocity": "max_angular_velocity",
        }
        for key, value in kwargs.items():
            attr = aliases.get(key)
            if attr is not None:
                setattr(self, attr, float(value))

    def compute_command(
        self,
        current_state: RobotState,
        reference: TrajectoryPoint,
    ) -> VelocityCommand:
        dx = reference.x - current_state.x
        dy = reference.y - current_state.y
        dyaw = normalize_angle(reference.yaw - current_state.yaw)

        c = math.cos(current_state.yaw)
        s = math.sin(current_state.yaw)

        error_x_body = c * dx + s * dy
        error_y_body = -s * dx + c * dy

        vx_ref_body = c * reference.vx + s * reference.vy
        vy_ref_body = -s * reference.vx + c * reference.vy

        vx = vx_ref_body + self.kx * error_x_body
        vy = vy_ref_body + self.ky * error_y_body
        vyaw = reference.wz + self.kyaw * dyaw

        speed = math.hypot(vx, vy)
        if self.max_linear_velocity > 0.0 and speed > self.max_linear_velocity:
            scale = self.max_linear_velocity / speed
            vx *= scale
            vy *= scale

        vyaw = clamp(vyaw, -self.max_angular_velocity, self.max_angular_velocity)
        return VelocityCommand(vx=vx, vy=vy, vyaw=vyaw)

    def is_goal_reached(
        self,
        current_state: RobotState,
        target_state: RobotState,
        position_tolerance: float = 0.05,
        yaw_tolerance: float = 0.05,
    ) -> bool:
        return (
            current_state.distance_to(target_state.x, target_state.y) < position_tolerance
            and abs(current_state.angle_to(target_state.yaw)) < yaw_tolerance
        )
