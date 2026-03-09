"""Waypoint Manager for navigation waypoints.

This module manages waypoint configuration and provides an interface
for setting and retrieving waypoints.
"""

from typing import Dict, Optional, Any
from dataclasses import dataclass
import yaml
import os


@dataclass
class Waypoint:
    """A navigation waypoint with position and orientation."""

    name: str
    x: float
    y: float
    yaw: float

    def __repr__(self) -> str:
        return (
            f"Waypoint({self.name}: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f})"
        )


class WaypointManager:
    """Manages navigation waypoints.

    Features:
    - Load waypoints from YAML configuration file
    - Add/remove waypoints dynamically
    - Set temporary waypoints
    - Retrieve waypoints by name
    """

    def __init__(self, config_path: Optional[str] = None):
        """Initialize waypoint manager.

        Args:
            config_path: Path to YAML configuration file
        """
        self._waypoints: Dict[str, Waypoint] = {}
        self._temporary_waypoints: Dict[str, Waypoint] = {}

        if config_path:
            self.load_from_file(config_path)

    def load_from_file(self, config_path: str) -> None:
        """Load waypoints from YAML configuration file.

        Args:
            config_path: Path to YAML file

        Raises:
            FileNotFoundError: If config file doesn't exist
            yaml.YAMLError: If YAML parsing fails
        """
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        if config and "waypoints" in config:
            for name, data in config["waypoints"].items():
                self.add_waypoint(
                    name=name,
                    x=float(data.get("x", 0.0)),
                    y=float(data.get("y", 0.0)),
                    yaw=float(data.get("yaw", 0.0)),
                )

    def add_waypoint(
        self, name: str, x: float, y: float, yaw: float, temporary: bool = False
    ) -> None:
        """Add a new waypoint.

        Args:
            name: Waypoint name/identifier
            x: X position (meters)
            y: Y position (meters)
            yaw: Orientation (radians)
            temporary: If True, waypoint is temporary (can be cleared)
        """
        waypoint = Waypoint(name=name, x=x, y=y, yaw=yaw)

        if temporary:
            self._temporary_waypoints[name] = waypoint
        else:
            self._waypoints[name] = waypoint

    def get_waypoint(self, name: str) -> Optional[Waypoint]:
        """Get waypoint by name.

        Args:
            name: Waypoint name

        Returns:
            Waypoint if found, None otherwise
        """
        # Check temporary waypoints first
        if name in self._temporary_waypoints:
            return self._temporary_waypoints[name]

        # Then check permanent waypoints
        return self._waypoints.get(name)

    def set_temporary_waypoint(self, name: str, x: float, y: float, yaw: float) -> None:
        """Set a temporary waypoint (overwrites existing with same name).

        Args:
            name: Waypoint name
            x: X position
            y: Y position
            yaw: Orientation
        """
        self._temporary_waypoints[name] = Waypoint(name=name, x=x, y=y, yaw=yaw)

    def remove_waypoint(self, name: str) -> bool:
        """Remove a waypoint.

        Args:
            name: Waypoint name

        Returns:
            True if waypoint was removed, False if not found
        """
        if name in self._temporary_waypoints:
            del self._temporary_waypoints[name]
            return True

        if name in self._waypoints:
            del self._waypoints[name]
            return True

        return False

    def clear_temporary_waypoints(self) -> None:
        """Clear all temporary waypoints."""
        self._temporary_waypoints.clear()

    def list_waypoints(self) -> Dict[str, Waypoint]:
        """List all waypoints (permanent + temporary).

        Returns:
            Dictionary of all waypoints
        """
        all_waypoints = {}
        all_waypoints.update(self._waypoints)
        all_waypoints.update(self._temporary_waypoints)
        return all_waypoints

    def has_waypoint(self, name: str) -> bool:
        """Check if waypoint exists.

        Args:
            name: Waypoint name

        Returns:
            True if waypoint exists
        """
        return name in self._waypoints or name in self._temporary_waypoints
