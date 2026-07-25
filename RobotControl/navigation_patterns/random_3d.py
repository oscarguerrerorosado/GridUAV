"""Fully three-dimensional random exploration."""

import numpy as np

from RobotControl.navigation_patterns.base import NavigationPattern
from RobotControl.navigation_patterns.utils import update_direction_3d


class RandomExploration3D(NavigationPattern):
    """Random walk on the unit sphere with smooth direction changes."""

    def __init__(self, step_size=0.05, angular_std=0.25):
        """
        Args:
            step_size: Displacement length per update in metres.
            angular_std: Standard deviation of angular changes (fraction of pi).
        """
        self.step_size = float(step_size)
        self.angular_std = float(angular_std)
        self._direction = np.array([1.0, 0.0, 0.0])

    def reset(self, start_position: np.ndarray) -> None:
        self._direction = np.array([1.0, 0.0, 0.0])

    def compute_movement(
        self,
        current_position: np.ndarray,
        current_velocity: np.ndarray,
        current_acceleration: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        self._direction = update_direction_3d(
            self._direction,
            self.step_size,
            dt,
            self.angular_std,
        )
        return self._direction.copy()
