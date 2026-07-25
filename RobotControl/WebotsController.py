"""
Webots controller class for the UAV.
This class is responsible for the communication with the Webots simulator.

It communicates with the following classes:
- the VelNoise class as orchestration layer.
- the UAVControl class to get the current state of the UAV and to set the target position.
- the Navigation class to get the target position.

It owns the Webots Robot/Supervisor instance and the simulation time loop.
It processes the UAV state and the lidar scan from the UAVControl class, to detect possible collisions.
It transfers this information to the Navigation class to generate the next step in the movement path and returns this to the UAVControl class.

VelNoise orchestrates flights via:

    wc = WebotsController(sim_time=self.sim_time)
    wc.run()
"""

from __future__ import annotations
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import numpy as np
from tqdm import tqdm
from controller import Supervisor
from RobotControl.Navigation import Navigation
from RobotControl.UAVControl import UAVControl

# Obstacle hit points use a small fixed radius for Navigation repulsion.
_OBSTACLE_RADIUS = 0.05
# Keep only lidar hits closer than this (metres) when forming obstacles.
_MAX_OBSTACLE_RANGE = 2.0
# Angular downsampling: keep one closest hit per coarse (layer, azimuth) bin.
_AZIMUTH_BINS = 36
_LAYER_STRIDE = 2


class WebotsController:
    """Own the Webots robot and time loop; bridge UAVControl and Navigation."""

    def __init__(
        self,
        sim_time=10.0,
        relative_position=False,
        navigation=None,
        **nav_kwargs,
    ):
        """
        Args:
            sim_time: Default flight duration in seconds for run().
            relative_position: Shared absolute/relative target mode for UAV and Navigation.
            navigation: Optional pre-built Navigation instance.
            **nav_kwargs: Forwarded to Navigation when navigation is None.
        """
        # Supervisor subclasses Robot; one instance owns the extern controller link.
        self.robot = Supervisor()
        self.robot_node = self.robot.getFromDef("Crazyflie")
        self.trans_field = (
            self.robot_node.getField("translation") if self.robot_node is not None else None
        )

        self.timestep = int(self.robot.getBasicTimeStep())
        self.sim_time = float(sim_time)
        self.relative_position = relative_position

        self.uav = UAVControl(self.robot, relative_position=relative_position)
        if navigation is None:
            self.navigation = Navigation(
                relative_position=relative_position,
                default_dt=self.uav.dt,
                **nav_kwargs,
            )
        else:
            self.navigation = navigation

        self._sensor_data = None

    def reset(self, start_position=None) -> None:
        """Reset navigation state from a given or current GPS position."""
        if start_position is None:
            if self._sensor_data is not None:
                start_position = self._sensor_data["position"]
            else:
                start_position = np.zeros(3)
        self.navigation.reset(start_position)

    def takeoff(
        self,
        hover_target=None,
        position_tol=0.1,
        speed_tol=0.1,
        hold_time=1.0,
        timeout=30.0,
    ) -> bool:
        """
        Climb to a stable hover before the experiment loop starts.

        Bypasses Navigation and commands a fixed hover target until the UAV
        holds position and speed within tolerance for hold_time seconds.

        Args:
            hover_target: Absolute hover position [x, y, z]. Defaults to room centre.
            position_tol: Max position error (m) to count as stable.
            speed_tol: Max GPS speed (m/s) to count as stable.
            hold_time: Continuous stable duration required (s).
            timeout: Max sim time for takeoff before proceeding with a warning (s).

        Returns:
            True if stable hover was reached, False if the timeout expired or
            the simulation ended early.
        """
        if hover_target is None:
            hover_target = np.zeros(3)
        else:
            hover_target = np.asarray(hover_target, dtype=float)

        takeoff_start = self.robot.getTime()
        stable_since = None
        reached = False

        dt_s = self.timestep / 1000.0
        estimated_steps = max(int(timeout / dt_s), 1)

        with tqdm(total=estimated_steps, desc="Takeoff / hover", unit="step") as progress:
            while True:
                if self.robot.step(self.timestep) == -1:
                    return False

                elapsed = self.robot.getTime() - takeoff_start
                if elapsed >= timeout:
                    print(
                        f"WARNING: takeoff timed out after {timeout:.1f}s "
                        f"without reaching stable hover; continuing anyway."
                    )
                    break

                self._sensor_data = self.uav.sense()
                position = np.asarray(self._sensor_data["position"], dtype=float)
                velocity = np.asarray(self._sensor_data["velocity_gps"], dtype=float)

                if self.relative_position:
                    target = hover_target - position
                else:
                    target = hover_target

                self.uav.act(target)

                pos_error = np.linalg.norm(position - hover_target)
                speed = np.linalg.norm(velocity)
                if pos_error < position_tol and speed < speed_tol:
                    if stable_since is None:
                        stable_since = self.robot.getTime()
                    elif self.robot.getTime() - stable_since >= hold_time:
                        reached = True
                        break
                else:
                    stable_since = None

                progress.update(1)

        return reached

    def step(self) -> dict | None:
        """
        Advance the simulation by one timestep and run one control cycle.

        Returns:
            Sensor dict from UAVControl, or None if the simulation has ended.
        """
        if self.robot.step(self.timestep) == -1:
            return None

        self._sensor_data = self.uav.sense()

        lidar_scan = self.uav.get_lidar_scan()
        obstacle_positions, obstacle_sizes = self._lidar_to_obstacles(
            self._sensor_data,
            lidar_scan,
        )
        target = self.navigation.update(
            self._sensor_data["position"],
            self._sensor_data["velocity_gps"],
            self._sensor_data["acceleration_linear"],
            obstacle_positions=obstacle_positions,
            obstacle_sizes=obstacle_sizes,
            dt=self.uav.dt,
        )
        self.uav.act(target)
        return self._sensor_data

    def run(self, sim_time=None) -> None:
        """
        Take off to a stable hover, then fly for sim_time seconds.

        The takeoff grace period does not count toward sim_time.

        Args:
            sim_time: Override constructor sim_time when provided.
        """
        duration = float(self.sim_time if sim_time is None else sim_time)
        dt_s = self.timestep / 1000.0
        estimated_steps = max(int(duration / dt_s), 1)

        self.takeoff()
        if self._sensor_data is not None:
            self.reset(self._sensor_data["position"])
            pos = self._sensor_data["position"]
            print(
                f"Takeoff complete; hovering near "
                f"[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]. "
                f"Starting experiment for {duration:.1f}s."
            )
        else:
            self.reset(np.zeros(3))
            print(f"Takeoff ended without sensor data; starting experiment for {duration:.1f}s.")

        start_time = self.robot.getTime()
        with tqdm(total=estimated_steps, desc="Webots flight", unit="step") as progress:
            while True:
                if self.robot.getTime() - start_time >= duration:
                    break
                sensor_data = self.step()
                if sensor_data is None:
                    break
                progress.update(1)

    def _lidar_to_obstacles(self, sensor_data, lidar_scan):
        """
        Convert a lidar range image into sparse world-frame obstacle points.

        Returns:
            tuple: (obstacle_positions shape (N, 3) or None, obstacle_sizes shape (N,) or None)
        """
        ranges = lidar_scan["ranges"]
        if ranges.size == 0:
            return None, None

        h_res = max(lidar_scan["horizontal_resolution"], 1)
        n_layers = max(lidar_scan["number_of_layers"], 1)
        expected = h_res * n_layers
        if ranges.size < expected:
            return None, None

        ranges = ranges[:expected].reshape(n_layers, h_res)
        fov = lidar_scan["fov"]
        v_fov = lidar_scan["vertical_fov"]
        min_range = lidar_scan["min_range"]
        max_range = min(lidar_scan["max_range"], _MAX_OBSTACLE_RANGE)

        # Closest hit per coarse angular bin: (layer_bin, azimuth_bin) -> (range, body_point)
        bins = {}
        for layer in range(0, n_layers, _LAYER_STRIDE):
            if n_layers == 1:
                elev = 0.0
            else:
                elev = -0.5 * v_fov + layer * (v_fov / (n_layers - 1))

            for col in range(h_res):
                r = float(ranges[layer, col])
                if not np.isfinite(r) or r < min_range or r >= max_range:
                    continue

                if h_res == 1:
                    azimuth = 0.0
                else:
                    azimuth = -0.5 * fov + col * (fov / (h_res - 1))

                cos_e = np.cos(elev)
                body_point = np.array(
                    [
                        r * cos_e * np.cos(azimuth),
                        r * cos_e * np.sin(azimuth),
                        r * np.sin(elev),
                    ],
                    dtype=float,
                )

                az_bin = int(((azimuth + 0.5 * fov) / max(fov, 1e-6)) * _AZIMUTH_BINS)
                az_bin = min(max(az_bin, 0), _AZIMUTH_BINS - 1)
                layer_bin = layer // _LAYER_STRIDE
                key = (layer_bin, az_bin)
                prev = bins.get(key)
                if prev is None or r < prev[0]:
                    bins[key] = (r, body_point)

        if not bins:
            return None, None

        position = np.asarray(sensor_data["position"], dtype=float)
        transform = self.uav.coordinate_transform
        obstacle_positions = np.array(
            [
                position + transform.body_centered_to_cartesian_vector(body_point)
                for _, body_point in bins.values()
            ],
            dtype=float,
        )
        obstacle_sizes = np.full(len(obstacle_positions), _OBSTACLE_RADIUS, dtype=float)
        return obstacle_positions, obstacle_sizes
