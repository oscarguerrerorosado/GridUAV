"""Constant-direction straight-line movement."""

import numpy as np

from RobotControl.navigation_patterns.base import NavigationPattern


class StraightLinePattern(NavigationPattern):
    """Move along a fixed direction at a constant step size."""

    def __init__(self, direction=(1.0, 0.0, 0.0), step_size=0.05):
        direction = np.asarray(direction, dtype=float)
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            raise ValueError("direction must be non-zero")
        self.direction = direction / norm
        self.step_size = float(step_size)

    def reset(self, start_position: np.ndarray) -> None:
        pass

    def compute_movement(
        self,
        current_position: np.ndarray,
        current_velocity: np.ndarray,
        current_acceleration: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        return self.direction * self.step_size
