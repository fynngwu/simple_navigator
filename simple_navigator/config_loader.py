from dataclasses import dataclass, field
from typing import Any
import os

import yaml


@dataclass
class TargetPose:
    x: float
    y: float
    yaw: float


@dataclass
class NavigatorConfig:
    target: TargetPose | None = None
    controller: dict[str, Any] = field(default_factory=dict)
    trajectory: dict[str, Any] = field(default_factory=dict)
    navigation: dict[str, Any] = field(default_factory=dict)
    frames: dict[str, Any] = field(default_factory=dict)


def load_navigator_config(config_file: str | None) -> NavigatorConfig:
    if not config_file or not os.path.exists(config_file):
        return NavigatorConfig()

    with open(config_file, "r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}

    target = None
    if "target" in raw:
        data = raw["target"] or {}
        target = TargetPose(
            x=float(data.get("x", 0.0)),
            y=float(data.get("y", 0.0)),
            yaw=float(data.get("yaw", 0.0)),
        )

    return NavigatorConfig(
        target=target,
        controller=dict(raw.get("controller", {})),
        trajectory=dict(raw.get("trajectory", {})),
        navigation=dict(raw.get("navigation", {})),
        frames=dict(raw.get("frames", {})),
    )
