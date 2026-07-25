"""Horizontal random exploration with fixed altitude."""

import numpy as np

from RobotControl.navigation_patterns.base import NavigationPattern
from RobotControl.navigation_patterns.utils import update_direction_2d


class RandomExploration2D(NavigationPattern):
    """Random walk confined to the horizontal plane."""

    def __init__(
        self,
        step_size=0.05,
        angular_std=0.25,
        fixed_altitude=None,
    ):
        """
        Args:
            step_size: Displacement length per update in metres.
            angular_std: Standard deviation of heading changes (fraction of pi).
            fixed_altitude: Target altitude in metres. When None, altitude is held
                at the value observed on the first update after reset().
        """
        self.step_size = float(step_size)
        self.angular_std = float(angular_std)
        self.fixed_altitude = fixed_altitude
        self._direction = np.array([1.0, 0.0, 0.0])
        self._locked_altitude = None

    def reset(self, start_position: np.ndarray) -> None:
        self._direction = np.array([1.0, 0.0, 0.0])
        self._locked_altitude = (
            float(self.fixed_altitude)
            if self.fixed_altitude is not None
            else float(start_position[2])
        )

    def compute_movement(
        self,
        current_position: np.ndarray,
        current_velocity: np.ndarray,
        current_acceleration: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        if self._locked_altitude is None:
            self._locked_altitude = float(current_position[2])

        self._direction = update_direction_2d(
            self._direction,
            self.step_size,
            dt,
            self.angular_std,
        )
        movement = self._direction.copy()
        movement[2] = self._locked_altitude - current_position[2]
        return movement
