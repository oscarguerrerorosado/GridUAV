"""
Navigation class for the UAV.

Defines the navigational path for the UAV. Called by the VelNoise controller to
generate the target position each control step.

Configuration parameters:
    - relative_position: If True, the returned target is a displacement from the
      current position; if False, the target is an absolute coordinate.
    - pattern: Movement pattern name or NavigationPattern instance.
    - boundaries: Arena limits as shape (3, 2) array of [lower, upper] per axis.
    - obstacle_avoidance: Whether to repel movement near obstacles.
    - avoidance_distance: Clearance kept from obstacle surfaces.

Inputs (update):
    - current_position: [x, y, z]
    - current_velocity: [vx, vy, vz]
    - current_acceleration: [ax, ay, az]
    - obstacle_positions: shape (N, 3) or None
    - obstacle_sizes: shape (N,) or None
    - dt: optional timestep override in seconds

Outputs:
    - target_position: [x_target, y_target, z_target]
"""

import numpy as np

from RobotControl.navigation_patterns import PATTERN_REGISTRY, NavigationPattern


DEFAULT_BOUNDARIES = np.array([
    [-2.8, 2.8],
    [-2.8, 2.8],
    [-2.8, 2.8],
])

class Navigation:
    """Facade that delegates movement generation to a swappable pattern."""

    def __init__(
        self,
        pattern="random_3d",
        relative_position=False,
        boundaries=None,
        obstacle_avoidance=True,
        avoidance_distance=0.05,
        default_dt=0.032,
        **pattern_kwargs,
    ):
        """
        Args:
            pattern: Registered pattern name or a NavigationPattern instance.
            relative_position: Return displacement when True, absolute target when False.
            boundaries: Arena limits, shape (3, 2). Defaults to the SquareBox arena.
            obstacle_avoidance: Apply simple repulsion near obstacles.
            avoidance_distance: Minimum clearance from obstacle surfaces in metres.
            default_dt: Timestep used when update() is called without dt.
            **pattern_kwargs: Constructor arguments forwarded to the selected pattern.
        """
        self.relative_position = relative_position
        self.boundaries = np.asarray(
            boundaries if boundaries is not None else DEFAULT_BOUNDARIES,
            dtype=float,
        )
        self.obstacle_avoidance = obstacle_avoidance
        self.avoidance_distance = float(avoidance_distance)
        self.default_dt = float(default_dt)
        self._last_time = None
        self._pattern = self._create_pattern(pattern, pattern_kwargs)

    @property
    def pattern(self) -> NavigationPattern:
        return self._pattern

    @property
    def pattern_name(self) -> str:
        for name, pattern_cls in PATTERN_REGISTRY.items():
            if isinstance(self._pattern, pattern_cls):
                return name
        return self._pattern.__class__.__name__

    def set_pattern(self, pattern, **pattern_kwargs) -> None:
        """Replace the active movement pattern."""
        self._pattern = self._create_pattern(pattern, pattern_kwargs)

    def reset(self, start_position=None) -> None:
        """Reset pattern state, optionally from a known starting position."""
        if start_position is None:
            start_position = np.zeros(3)
        self._last_time = None
        self._pattern.reset(np.asarray(start_position, dtype=float))

    def update(
        self,
        current_position,
        current_velocity,
        current_acceleration,
        obstacle_positions=None,
        obstacle_sizes=None,
        dt=None,
    ):
        """
        Compute the next target position for the UAV.

        Returns:
            np.ndarray: Target position as [x, y, z].
        """
        current_position = np.asarray(current_position, dtype=float)
        current_velocity = np.asarray(current_velocity, dtype=float)
        current_acceleration = np.asarray(current_acceleration, dtype=float)
        step_dt = self._resolve_dt(dt)

        movement = self._pattern.compute_movement(
            current_position,
            current_velocity,
            current_acceleration,
            step_dt,
        )
        movement = self._apply_constraints(
            current_position,
            movement,
            obstacle_positions,
            obstacle_sizes,
        )

        if self.relative_position:
            return movement
        return current_position + movement

    def _create_pattern(self, pattern, pattern_kwargs):
        if isinstance(pattern, NavigationPattern):
            return pattern
        if pattern not in PATTERN_REGISTRY:
            available = ", ".join(sorted(PATTERN_REGISTRY))
            raise ValueError(f'Unknown pattern "{pattern}". Available: {available}')
        return PATTERN_REGISTRY[pattern](**pattern_kwargs)

    def _resolve_dt(self, dt):
        if dt is not None:
            return float(dt)
        return self.default_dt

    def _apply_constraints(
        self,
        current_position,
        movement,
        obstacle_positions,
        obstacle_sizes,
    ):
        from RobotControl.navigation_patterns.utils import (
            adjust_for_boundaries,
            adjust_for_obstacles,
        )

        movement = adjust_for_boundaries(self.boundaries, current_position, movement)

        if self.obstacle_avoidance and obstacle_positions is not None:
            movement = adjust_for_obstacles(
                current_position,
                movement,
                obstacle_positions,
                obstacle_sizes,
                self.avoidance_distance,
            )
            movement = adjust_for_boundaries(self.boundaries, current_position, movement)

        return movement
