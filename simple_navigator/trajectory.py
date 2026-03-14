from dataclasses import dataclass
import math

from .math_utils import normalize_angle, smoothstep5, smoothstep5_derivative


@dataclass
class TrajectoryPoint:
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    done: bool = False


class SE2Trajectory:
    def __init__(
        self,
        x0: float,
        y0: float,
        yaw0: float,
        x1: float,
        y1: float,
        yaw1: float,
        duration: float,
    ) -> None:
        self.x0 = x0
        self.y0 = y0
        self.yaw0 = normalize_angle(yaw0)
        self.x1 = x1
        self.y1 = y1
        self.yaw1 = normalize_angle(yaw1)
        self.duration = max(duration, 0.1)

        self.dx = self.x1 - self.x0
        self.dy = self.y1 - self.y0
        self.dyaw = normalize_angle(self.yaw1 - self.yaw0)

    def sample(self, t: float) -> TrajectoryPoint:
        if t >= self.duration:
            return TrajectoryPoint(
                x=self.x1,
                y=self.y1,
                yaw=self.yaw1,
                vx=0.0,
                vy=0.0,
                wz=0.0,
                done=True,
            )

        s = max(0.0, t / self.duration)
        alpha = smoothstep5(s)
        alpha_dot = smoothstep5_derivative(s) / self.duration

        return TrajectoryPoint(
            x=self.x0 + alpha * self.dx,
            y=self.y0 + alpha * self.dy,
            yaw=normalize_angle(self.yaw0 + alpha * self.dyaw),
            vx=alpha_dot * self.dx,
            vy=alpha_dot * self.dy,
            wz=alpha_dot * self.dyaw,
        )

    @classmethod
    def from_states(
        cls,
        start,
        goal,
        max_trans_vel: float,
        max_rot_vel: float,
        duration_scale: float = 1.2,
        min_duration: float = 0.3,
    ) -> "SE2Trajectory":
        distance = math.hypot(goal.x - start.x, goal.y - start.y)
        dyaw = abs(normalize_angle(goal.yaw - start.yaw))

        duration = duration_scale * max(
            distance / max(max_trans_vel, 1e-6),
            dyaw / max(max_rot_vel, 1e-6),
            min_duration,
        )

        return cls(
            x0=start.x,
            y0=start.y,
            yaw0=start.yaw,
            x1=goal.x,
            y1=goal.y,
            yaw1=goal.yaw,
            duration=duration,
        )
