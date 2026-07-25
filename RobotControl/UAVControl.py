"""
UAV control for a Crazyflie quadcopter in Webots.
Reads sensors, converts cartesian targets to motor commands via PID control.
"""

import numpy as np
from Helpers.CoordinateTransform import CoordinateTransform
from controller import Robot


class UAVControl:
    """
    Control a quadcopter from cartesian position targets.

    Maintains two velocity estimates each timestep: GPS differentiation (sim ground
    truth) and accelerometer integration (for GPS-free real-world use).
    """

    def __init__(self, robot, relative_position=False, acc_z_bias=6.3e-6):
        """
        Initialise the UAV controller, motors, sensors, and PID state.

        Args:
            robot: Webots Robot instance.
            relative_position: If True, update() targets are offsets from the
                current position; if False, targets are absolute coordinates.
            acc_z_bias: Empirical body-frame z accelerometer offset (m/s²).
        """
        self.robot = robot
        self.relative_position = relative_position
        self.acc_z_bias = acc_z_bias

        timestep = int(self.robot.getBasicTimeStep())
        self.dt = timestep / 1000.0

        self._init_motors()
        self._init_sensors(timestep)
        self.coordinate_transform = CoordinateTransform()

        self.pos_global = np.zeros(3)
        self.past_pos = np.zeros(3)
        self.velocity_integrated = np.zeros(3)
        self.linear_acceleration = np.zeros(3)
        self.past_time = 0.0

        self._init_pid_state()
        self._init_pid_gains()

    def update(self, target):
        """
        Process a cartesian target position and drive the motors.

        Convenience wrapper that reads sensors then acts. Prefer sense() + act()
        when the caller already needs sensor data before computing a target.

        Args:
            target: [x, y] or [x, y, z] in the global frame.
                Interpreted as relative or absolute per relative_position.

        Returns:
            dict: Current UAV sensor readings from get_sensor_data().
        """
        self.sense()
        return self.act(target)

    def sense(self):
        """
        Read sensors without commanding the motors.

        Useful for bootstrapping state before the first navigation update.

        Returns:
            dict: Current UAV sensor readings from get_sensor_data().
        """
        self._read_sensors()
        return self.get_sensor_data()

    def act(self, target):
        """
        Drive the motors from a target using the latest sense() readings.

        Must be called after sense() in the same simulation step so that
        sensors are read exactly once (avoids dt=0 and exploded D-terms).

        Args:
            target: [x, y] or [x, y, z] in the global frame.
                Interpreted as relative or absolute per relative_position.

        Returns:
            dict: Current UAV sensor readings from get_sensor_data().
        """
        target_pos = self._resolve_target(target)
        motor_speeds = self._compute_motor_speeds(target_pos)
        self._set_motors(motor_speeds)
        return self.get_sensor_data()

    def get_sensor_data(self):
        """
        Return the latest sensor readings from the most recent update cycle.

        Returns:
            dict: Keys include position, velocity_gps, velocity_integrated,
                altitude, roll, pitch, yaw, yaw_rate, acceleration_body,
                and acceleration_linear.
        """
        return {
            "position": self.pos_global.copy(),
            "velocity_gps": self._gps_velocity(),
            "velocity_integrated": self.velocity_integrated.copy(),
            "altitude": self.altitude,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "yaw_rate": self.yaw_rate,
            "acceleration_body": np.array([self.ax, self.ay, self.az]),
            "acceleration_linear": self.linear_acceleration.copy(),
        }

    def get_lidar_scan(self):
        """
        Return the latest lidar range image and sensor metadata.

        Returns:
            dict: Keys include ranges, horizontal_resolution, number_of_layers,
                fov, vertical_fov, min_range, and max_range. ranges is empty
                when no image is available yet.
        """
        range_image = self.lidar.getRangeImage()
        ranges = (
            np.asarray(range_image, dtype=float)
            if range_image is not None
            else np.array([], dtype=float)
        )
        return {
            "ranges": ranges,
            "horizontal_resolution": int(self.lidar.getHorizontalResolution()),
            "number_of_layers": int(self.lidar.getNumberOfLayers()),
            "fov": float(self.lidar.getFov()),
            "vertical_fov": float(self.lidar.getVerticalFov()),
            "min_range": float(self.lidar.getMinRange()),
            "max_range": float(self.lidar.getMaxRange()),
        }

    def _init_motors(self):
        """Configure the four rotor motors for velocity control."""
        self.motors = []
        for i in range(4):
            motor = self.robot.getDevice(f"m{i + 1}_motor")
            motor.setPosition(float("inf"))
            motor.setVelocity(-1)
            self.motors.append(motor)

    def _init_sensors(self, timestep):
        """
        Acquire and enable all onboard sensors at the simulation timestep.

        Args:
            timestep: Webots basic time step in milliseconds.
        """
        self.imu = self.robot.getDevice("inertial_unit")
        self.imu.enable(timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(timestep)
        self.acc = self.robot.getDevice("acc")
        self.acc.enable(timestep)
        self.lidar = self.robot.getDevice("lidar")
        self.lidar.enable(timestep)
        if hasattr(self.lidar, "enablePointCloud"):
            self.lidar.enablePointCloud()

    def _init_pid_state(self):
        """Reset PID error accumulators and integrators to their initial values."""
        self.prev_vx_error = 0.0
        self.prev_vy_error = 0.0
        self.prev_alt_error = 0.0
        self.prev_pitch_error = 0.0
        self.prev_roll_error = 0.0
        self.prev_yaw_error = 0.0
        self.prev_vz_error = 0.0
        self.altitude_integrator_vz = 0.0
        self.yaw_integrator = 0.0
        self.prev_altitude = None
        self.hover_speed = 55.35
        self._pid_seeded = False

    def _init_pid_gains(self):
        """Set default PID gain values for velocity, attitude, and altitude control."""
        self.gains = {
            "kp_pos": 0.8,
            "kp_att_y": 1.0,
            "kd_att_y": 0.5,
            "kp_att_rp": 0.5,
            "kd_att_rp": 0.1,
            "kp_vel_xy": 2.0,
            "kd_vel_xy": 0.5,
            "k_alt": 1.0,
            "kp_vz": 1.0,
            "kd_vz": 0.5,
            "ki_vz": 0.01,
            "Kp_yaw": 2.0,
            "Ki_yaw": 0.0,
            "Kd_yaw": 0.5,
        }
        self.clip_bound = 15

    def _read_sensors(self):
        """
        Read all sensors, update pose state, and integrate velocity.

        Updates position, attitude, raw acceleration, simulation timestep,
        and triggers accelerometer-based velocity integration.
        """
        self.yaw_rate = self.gyro.getValues()[2]
        self.gps_values = self.gps.getValues()
        self.altitude = self.gps_values[2]
        self.ax, self.ay, self.az = self.acc.getValues()
        self.roll, self.pitch, self.yaw = self.imu.getRollPitchYaw()
        self.coordinate_transform._update_rotation_matrix(self.roll, self.pitch, self.yaw)

        self.past_pos = self.pos_global.copy()
        self.pos_global = np.array(self.gps_values, dtype=float)

        current_time = self.robot.getTime()
        if self.past_time > 0.0:
            self.dt = current_time - self.past_time
        self.past_time = current_time

        self._integrate_velocity()

    def _integrate_velocity(self):
        """
        Integrate linear acceleration into cartesian velocity.

        The accelerometer reports specific force in the body frame, including
        gravity. The empirical z bias and gravity are removed before transforming
        to the cartesian frame and integrating with Euler's method.
        """
        acc_body = np.array([self.ax, self.ay, self.az - self.acc_z_bias])
        gravity_body = np.array(self.coordinate_transform.cartesian_to_body_centered(0, 0, 9.81))
        acc_linear_body = acc_body - gravity_body
        self.linear_acceleration = self.coordinate_transform.body_centered_to_cartesian_vector(acc_linear_body)

        if self.dt > 0.0:
            self.velocity_integrated += self.linear_acceleration * self.dt

    def _resolve_target(self, target):
        """
        Convert input coordinates to an absolute target position.

        Args:
            target: [x, y] or [x, y, z] in the global frame.

        Returns:
            np.ndarray: Absolute [x, y, z] target position.
        """
        target = np.asarray(target, dtype=float).reshape(-1)
        if target.size == 2:
            target = np.array([target[0], target[1], self.altitude])
        if self.relative_position:
            return self.pos_global + target
        return target

    def _gps_velocity(self):
        """
        Estimate velocity by differentiating consecutive GPS positions.

        Returns:
            np.ndarray: Cartesian velocity [vx, vy, vz] in m/s.
        """
        if self.dt <= 0.0:
            return np.zeros(3)
        return (self.pos_global - self.past_pos) / self.dt

    def _target_to_setpoints(self, target_pos):
        """
        Derive body-frame velocity and altitude setpoints from a target position.

        Args:
            target_pos: Absolute [x, y, z] target in the global frame.

        Returns:
            tuple: (desired_vx, desired_vy, desired_yaw, desired_altitude).
                Horizontal velocities are in the body frame; yaw is in radians.
        """
        pos_error = target_pos - self.pos_global
        desired_v_global = self.gains["kp_pos"] * pos_error
        desired_v_body = self.coordinate_transform.cartesian_to_body_centered_vector(desired_v_global)

        desired_yaw = self.yaw
        if np.linalg.norm(pos_error[:2]) > 1e-6:
            desired_yaw = np.arctan2(pos_error[1], pos_error[0])

        return desired_v_body[0], desired_v_body[1], desired_yaw, target_pos[2]

    def _compute_motor_speeds(self, target_pos):
        """
        Run the full PID pipeline to compute per-motor velocity commands.

        Args:
            target_pos: Absolute [x, y, z] target in the global frame.

        Returns:
            list: Motor speeds [m1, m2, m3, m4].
        """
        desired_vx, desired_vy, desired_yaw, desired_altitude = self._target_to_setpoints(target_pos)

        gps_velocity = self._gps_velocity()
        v_body = self.coordinate_transform.cartesian_to_body_centered_vector(gps_velocity)

        roll_cmd, pitch_cmd, yaw_cmd = self._attitude_commands(
            desired_vx, desired_vy, desired_yaw, v_body[0], v_body[1]
        )
        alt_cmd = self._altitude_command(desired_altitude)
        return self._mix_motors(alt_cmd, roll_cmd, pitch_cmd, yaw_cmd)

    def _attitude_commands(self, desired_vx, desired_vy, desired_yaw, actual_vx, actual_vy):
        """
        Compute roll, pitch, and yaw motor mixing commands from velocity and yaw errors.

        Args:
            desired_vx: Target forward velocity in the body frame (m/s).
            desired_vy: Target lateral velocity in the body frame (m/s).
            desired_yaw: Target heading in radians.
            actual_vx: Measured forward velocity in the body frame (m/s).
            actual_vy: Measured lateral velocity in the body frame (m/s).

        Returns:
            tuple: (roll_command, pitch_command, yaw_command) for motor mixing.
        """
        dt = max(self.dt, 1e-6)

        vx_error = desired_vx - actual_vx
        vy_error = desired_vy - actual_vy
        yaw_error = (desired_yaw - self.yaw + np.pi) % (2 * np.pi) - np.pi

        if not self._pid_seeded:
            # First cycle: seed previous errors so derivative terms start at zero.
            self.prev_vx_error = vx_error
            self.prev_vy_error = vy_error
            self.prev_yaw_error = yaw_error

        vx_deriv = (vx_error - self.prev_vx_error) / dt
        vy_deriv = (vy_error - self.prev_vy_error) / dt

        desired_pitch = (
            self.gains["kp_vel_xy"] * np.clip(vx_error, -self.clip_bound, self.clip_bound)
            + self.gains["kd_vel_xy"] * vx_deriv
        )
        desired_roll = (
            -self.gains["kp_vel_xy"] * np.clip(vy_error, -self.clip_bound, self.clip_bound)
            - self.gains["kd_vel_xy"] * vy_deriv
        )

        yaw_deriv = (yaw_error - self.prev_yaw_error) / dt
        self.yaw_integrator += yaw_error * dt
        yaw_rate_command = (
            self.gains["Kp_yaw"] * yaw_error
            + self.gains["Ki_yaw"] * self.yaw_integrator
            + self.gains["Kd_yaw"] * yaw_deriv
        )

        self.prev_vx_error = vx_error
        self.prev_vy_error = vy_error
        self.prev_yaw_error = yaw_error

        pitch_error = desired_pitch - self.pitch
        roll_error = desired_roll - self.roll

        if not self._pid_seeded:
            self.prev_pitch_error = pitch_error
            self.prev_roll_error = roll_error

        pitch_deriv = (pitch_error - self.prev_pitch_error) / dt
        roll_deriv = (roll_error - self.prev_roll_error) / dt
        yaw_rate_error = yaw_rate_command - self.yaw_rate

        roll_command = (
            self.gains["kp_att_rp"] * np.clip(roll_error, -self.clip_bound, self.clip_bound)
            + self.gains["kd_att_rp"] * roll_deriv
        )
        pitch_command = (
            -self.gains["kp_att_rp"] * np.clip(pitch_error, -self.clip_bound, self.clip_bound)
            - self.gains["kd_att_rp"] * pitch_deriv
        )
        yaw_command = self.gains["kp_att_y"] * np.clip(yaw_rate_error, -self.clip_bound, self.clip_bound)

        self.prev_pitch_error = pitch_error
        self.prev_roll_error = roll_error

        return roll_command, pitch_command, yaw_command

    def _altitude_command(self, desired_altitude):
        """
        Compute the base throttle command from vertical velocity PID control.

        Args:
            desired_altitude: Target altitude in metres.

        Returns:
            float: Base motor speed before attitude mixing.
        """
        dt = max(self.dt, 1e-6)

        if self.prev_altitude is None:
            vertical_velocity = 0.0
        else:
            vertical_velocity = (self.altitude - self.prev_altitude) / dt
        self.prev_altitude = self.altitude

        alt_error = desired_altitude - self.altitude
        desired_vz = self.gains["k_alt"] * alt_error
        vz_error = desired_vz - vertical_velocity

        if not self._pid_seeded:
            self.prev_vz_error = vz_error
            self._pid_seeded = True

        vz_deriv = (vz_error - self.prev_vz_error) / dt
        self.altitude_integrator_vz += vz_error * dt
        self.prev_vz_error = vz_error

        vz_command = (
            self.gains["kp_vz"] * vz_error
            + self.gains["kd_vz"] * vz_deriv
            + self.gains["ki_vz"] * self.altitude_integrator_vz
        )
        return self.hover_speed + 2 * vz_command

    def _mix_motors(self, alt_command, roll_command, pitch_command, yaw_command):
        """
        Combine altitude and attitude commands into individual motor speeds.

        Args:
            alt_command: Base throttle shared by all motors.
            roll_command: Differential roll contribution.
            pitch_command: Differential pitch contribution.
            yaw_command: Differential yaw contribution.

        Returns:
            list: Clipped motor speeds [m1, m2, m3, m4] in the range [0, 600].
        """
        m1 = alt_command - roll_command + pitch_command + yaw_command
        m2 = alt_command - roll_command - pitch_command - yaw_command
        m3 = alt_command + roll_command - pitch_command + yaw_command
        m4 = alt_command + roll_command + pitch_command - yaw_command
        return [np.clip(speed, 0, 600) for speed in (m1, m2, m3, m4)]

    def _set_motors(self, motor_speeds):
        """
        Apply computed speeds to the four rotors.

        Motors 1 and 3 are sign-inverted to match the Crazyflie rotor convention
        in the Webots model.

        Args:
            motor_speeds: List of four motor velocity commands.
        """
        motor_speeds[0] *= -1
        motor_speeds[2] *= -1

        for motor, speed in zip(self.motors, motor_speeds):
            motor.setVelocity(speed)
