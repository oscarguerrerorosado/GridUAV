"""Shared helpers for navigation movement patterns."""

import numpy as np


def adjust_for_boundaries(
    boundaries: np.ndarray,
    position: np.ndarray,
    movement_vector: np.ndarray,
) -> np.ndarray:
    """
    Clip a movement vector so the resulting position stays inside the arena.

    Args:
        boundaries: Shape (3, 2) with [lower, upper] bounds for x, y, and z.
        position: Current [x, y, z] position.
        movement_vector: Proposed [dx, dy, dz] displacement.

    Returns:
        Displacement clipped to remain within boundaries.
    """
    new_position = position + movement_vector
    adjusted_vector = movement_vector.copy()
    lower_bounds, upper_bounds = boundaries[:, 0], boundaries[:, 1]

    for axis in range(3):
        if new_position[axis] < lower_bounds[axis]:
            adjusted_vector[axis] = lower_bounds[axis] - position[axis]
        elif new_position[axis] > upper_bounds[axis]:
            adjusted_vector[axis] = upper_bounds[axis] - position[axis]

    return adjusted_vector


def adjust_for_obstacles(
    position: np.ndarray,
    movement_vector: np.ndarray,
    obstacle_positions: np.ndarray,
    obstacle_sizes: np.ndarray,
    avoidance_distance: float,
) -> np.ndarray:
    """
    Repel the movement vector away from nearby obstacles.

    Args:
        position: Current [x, y, z] position.
        movement_vector: Proposed [dx, dy, dz] displacement.
        obstacle_positions: Array of shape (N, 3) or empty.
        obstacle_sizes: Array of shape (N,) with obstacle radii or half-extents.
        avoidance_distance: Minimum clearance kept from obstacle surfaces.

    Returns:
        Adjusted displacement with repulsion applied near obstacles.
    """
    if obstacle_positions is None or len(obstacle_positions) == 0:
        return movement_vector

    obstacle_positions = np.asarray(obstacle_positions, dtype=float).reshape(-1, 3)
    obstacle_sizes = np.asarray(obstacle_sizes, dtype=float).reshape(-1)
    adjusted_vector = movement_vector.copy()

    for obstacle_position, obstacle_size in zip(obstacle_positions, obstacle_sizes):
        offset = position - obstacle_position
        distance = np.linalg.norm(offset)
        clearance = obstacle_size + avoidance_distance

        if distance < 1e-6:
            adjusted_vector += np.random.randn(3)
            continue

        if distance < clearance:
            repulsion = (clearance - distance) / clearance
            adjusted_vector += repulsion * (offset / distance)

    return adjusted_vector


def update_direction_3d(
    current_direction: np.ndarray,
    magnitude: float,
    dt: float,
    angular_std: float = 0.25,
) -> np.ndarray:
    """
    Perturb a 3D direction with a small random rotation (random walk on the sphere).

    Args:
        current_direction: Current movement direction vector.
        magnitude: Desired length of the returned vector.
        dt: Timestep in seconds.
        angular_std: Standard deviation of angular change as a fraction of pi.

    Returns:
        Updated direction vector with the requested magnitude.
    """
    norm_curr = np.linalg.norm(current_direction)
    if norm_curr < 1e-6:
        random_vec = np.random.randn(3)
        unit_current = random_vec / np.linalg.norm(random_vec)
    else:
        unit_current = current_direction / norm_curr

    d_angle = np.random.normal(0, angular_std * np.pi * np.sqrt(dt))
    if np.abs(d_angle) < 1e-8:
        return unit_current * magnitude

    random_vector = np.random.randn(3)
    perp = random_vector - np.dot(random_vector, unit_current) * unit_current
    perp_norm = np.linalg.norm(perp)
    if perp_norm < 1e-6:
        if np.abs(unit_current[0]) < 0.9:
            perp = np.cross(unit_current, np.array([1.0, 0.0, 0.0]))
        else:
            perp = np.cross(unit_current, np.array([0.0, 1.0, 0.0]))
        perp_norm = np.linalg.norm(perp)
    axis = perp / perp_norm

    new_direction = np.cos(d_angle) * unit_current + np.sin(d_angle) * np.cross(axis, unit_current)
    return new_direction * magnitude


def update_direction_2d(
    current_direction: np.ndarray,
    magnitude: float,
    dt: float,
    angular_std: float = 0.25,
) -> np.ndarray:
    """
    Perturb a horizontal direction with a small random yaw change.

    Args:
        current_direction: Current movement direction vector (xy components used).
        magnitude: Desired length of the returned vector.
        dt: Timestep in seconds.
        angular_std: Standard deviation of angular change as a fraction of pi.

    Returns:
        Updated horizontal direction vector with zero z component.
    """
    heading = np.arctan2(current_direction[1], current_direction[0])
    d_heading = np.random.normal(0, angular_std * np.pi * np.sqrt(dt))
    heading += d_heading
    direction = np.array([np.cos(heading), np.sin(heading), 0.0])
    return direction * magnitude
