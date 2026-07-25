"""Circular orbit movement in a selected plane."""

import numpy as np

from RobotControl.navigation_patterns.base import NavigationPattern


class CirclePattern(NavigationPattern):
    """Orbit a centre point at a fixed radius."""

    def __init__(
        self,
        center=(0.0, 0.0, 1.0),
        radius=0.5,
        angular_speed=0.5,
        plane="xy",
        fixed_coordinate=None,
    ):
        """
        Args:
            center: Orbit centre [x, y, z].
            radius: Orbit radius in metres.
            angular_speed: Angular speed in rad/s.
            plane: Orbit plane, one of "xy", "xz", or "yz".
            fixed_coordinate: Value held constant on the axis orthogonal to the plane.
                Defaults to the corresponding centre coordinate.
        """
        self.center = np.asarray(center, dtype=float)
        self.radius = float(radius)
        self.angular_speed = float(angular_speed)
        self.plane = plane.lower()
        if self.plane not in {"xy", "xz", "yz"}:
            raise ValueError('plane must be "xy", "xz", or "yz"')

        axis_index = {"xy": 2, "xz": 1, "yz": 0}[self.plane]
        self.fixed_coordinate = (
            float(fixed_coordinate)
            if fixed_coordinate is not None
            else float(self.center[axis_index])
        )

    def reset(self, start_position: np.ndarray) -> None:
        pass

    def compute_movement(
        self,
        current_position: np.ndarray,
        current_velocity: np.ndarray,
        current_acceleration: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        current_position = np.asarray(current_position, dtype=float)
        target = current_position.copy()

        if self.plane == "xy":
            rel = current_position[:2] - self.center[:2]
            angle = np.arctan2(rel[1], rel[0]) if np.linalg.norm(rel) > 1e-6 else 0.0
            new_angle = angle + self.angular_speed * dt
            target[:2] = self.center[:2] + self.radius * np.array(
                [np.cos(new_angle), np.sin(new_angle)]
            )
            target[2] = self.fixed_coordinate
        elif self.plane == "xz":
            rel = np.array([current_position[0] - self.center[0], current_position[2] - self.center[2]])
            angle = np.arctan2(rel[1], rel[0]) if np.linalg.norm(rel) > 1e-6 else 0.0
            new_angle = angle + self.angular_speed * dt
            target[0] = self.center[0] + self.radius * np.cos(new_angle)
            target[2] = self.center[2] + self.radius * np.sin(new_angle)
            target[1] = self.fixed_coordinate
        else:
            rel = np.array([current_position[1] - self.center[1], current_position[2] - self.center[2]])
            angle = np.arctan2(rel[1], rel[0]) if np.linalg.norm(rel) > 1e-6 else 0.0
            new_angle = angle + self.angular_speed * dt
            target[1] = self.center[1] + self.radius * np.cos(new_angle)
            target[2] = self.center[2] + self.radius * np.sin(new_angle)
            target[0] = self.fixed_coordinate

        return target - current_position
