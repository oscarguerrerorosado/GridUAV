"""Abstract base class for UAV movement patterns."""

from abc import ABC, abstractmethod

import numpy as np


class NavigationPattern(ABC):
    """Compute a proposed displacement vector for one navigation step."""

    @abstractmethod
    def reset(self, start_position: np.ndarray) -> None:
        """Reset internal state when a new trajectory starts."""

    @abstractmethod
    def compute_movement(
        self,
        current_position: np.ndarray,
        current_velocity: np.ndarray,
        current_acceleration: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """
        Propose a displacement vector for the current timestep.

        Args:
            current_position: Current [x, y, z] in metres.
            current_velocity: Current [vx, vy, vz] in m/s.
            current_acceleration: Current [ax, ay, az] in m/s².
            dt: Timestep in seconds.

        Returns:
            Proposed displacement [dx, dy, dz] in metres.
        """
