"""
Coordinate transformation between Cartesian (inertial/world) and body-centered frames.

The body frame is related to the Cartesian frame by rotations about roll (x),
pitch (y), and yaw (z) using the ZYX Euler angle convention.
"""

import numpy as np

class CoordinateTransform:
    """Transform vectors between Cartesian and body-centered coordinates."""

    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0):
        self.rotation_matrix = np.eye(3)
        self._update_rotation_matrix(roll, pitch, yaw)

    def _update_rotation_matrix(self, roll, pitch, yaw):
        """Build body-to-Cartesian rotation matrix: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        self.rotation_matrix = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])

    def body_centered_to_cartesian(self, x, y, z):
        """Transform a vector from body frame to Cartesian frame."""
        v_body = np.array([x, y, z], dtype=float)
        v_cart = self.rotation_matrix @ v_body
        return float(v_cart[0]), float(v_cart[1]), float(v_cart[2])

    def body_centered_to_cartesian_vector(self, vector):
        """Transform a vector from body frame to Cartesian frame."""
        return self.rotation_matrix @ vector

    def cartesian_to_body_centered(self, x, y, z):
        """Transform a vector from Cartesian frame to body frame."""
        v_cart = np.array([x, y, z], dtype=float)
        v_body = self.rotation_matrix.T @ v_cart
        return float(v_body[0]), float(v_body[1]), float(v_body[2])

    def cartesian_to_body_centered_vector(self, vector):
        """Transform a vector from Cartesian frame to body frame."""
        return self.rotation_matrix.T @ vector