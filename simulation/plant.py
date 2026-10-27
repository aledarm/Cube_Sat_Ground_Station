"""Rigid-body + three reaction-wheel simulation used only by the desktop SIM.

The team's C branch provides controller logic, not rigid-body dynamics. This module
therefore keeps plant physics separate: diagonal inertia, wheel angular momentum,
Euler gyroscopic coupling, quaternion attitude integration, actuator saturation,
and small deterministic disturbance torques. The latter make an uncontrolled body
drift/coast instead of visually freezing in space.
"""
from __future__ import annotations
import numpy as np


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ], dtype=float)


def _quat_from_euler_deg(a: np.ndarray) -> np.ndarray:
    r, p, y = np.deg2rad(np.asarray(a, dtype=float)) * 0.5
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    q = np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])
    return q / np.linalg.norm(q)


def _euler_deg_from_quat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    sinr = 2.0 * (w*x + y*z)
    cosr = 1.0 - 2.0 * (x*x + y*y)
    roll = np.arctan2(sinr, cosr)
    sinp = np.clip(2.0 * (w*y - z*x), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    siny = 2.0 * (w*z + x*y)
    cosy = 1.0 - 2.0 * (y*y + z*z)
    yaw = np.arctan2(siny, cosy)
    out = np.rad2deg([roll, pitch, yaw])
    return (out + 180.0) % 360.0 - 180.0


class ReactionWheelPlant:
    def __init__(self) -> None:
        # Initial engineering estimates for the desktop model; replace with measured CAD/test values.
        self.body_inertia = np.array([0.024, 0.026, 0.030])  # kg*m^2
        self.wheel_inertia = np.array([0.00035, 0.00035, 0.00040])  # kg*m^2
        self.torque_limit = 0.20  # N*m, matches current team's C safety limit
        self.speed_limit_rpm = 3000.0
        self.damping = 2.0e-5  # legacy/public knob: zero also disables ambient disturbance for conservation tests
        self.sim_time = 0.0
        self.reset()

    @property
    def attitude(self) -> np.ndarray:
        return _euler_deg_from_quat(self.quaternion)

    def set_attitude(self, attitude_deg: np.ndarray) -> None:
        self.quaternion = _quat_from_euler_deg(np.asarray(attitude_deg, dtype=float))

    def reset(self, attitude: np.ndarray | None = None) -> None:
        initial = np.array([8., -6., 25.]) if attitude is None else np.asarray(attitude, dtype=float).copy()
        self.quaternion = _quat_from_euler_deg(initial)
        self.rate = np.zeros(3)  # body angular rate, rad/s
        self.wheel_relative_rate = np.zeros(3)  # wheel rate relative to body, rad/s
        self.wheel_angle = np.zeros(3)
        self.motor_torque = np.zeros(3)
        self.external_torque = np.zeros(3)
        self.saturated = np.zeros(3, dtype=bool)
        self.sim_time = 0.0

    @property
    def wheel_rpm(self) -> np.ndarray:
        return self.wheel_relative_rate * 60.0 / (2.0 * np.pi)

    def _disturbance_torque(self) -> np.ndarray:
        """Small repeatable environmental/bench disturbances, N*m.

        These are intentionally small and deterministic: they reveal that reaction
        wheels actively stabilize the body while keeping tests reproducible.
        """
        if self.damping <= 0.0:
            return np.zeros(3)
        t = self.sim_time
        return np.array([
            7.0e-6 + 5.0e-6*np.sin(0.37*t),
            -5.0e-6 + 4.0e-6*np.sin(0.23*t + 0.8),
            6.0e-6*np.sin(0.31*t + 1.4),
        ])

    def step(self, requested_motor_torque: np.ndarray, dt: float) -> None:
        requested = np.asarray(requested_motor_torque, dtype=float)
        torque = np.clip(requested, -self.torque_limit, self.torque_limit)
        self.saturated = np.abs(requested) >= self.torque_limit - 1e-12
        outward = ((np.abs(self.wheel_rpm) >= self.speed_limit_rpm)
                   & (torque * self.wheel_relative_rate > 0.0))
        self.saturated |= outward
        torque[outward] = 0.0
        self.motor_torque = torque.copy()

        # Wheel angular momentum contributes to the rigid-body gyroscopic term.
        body_momentum = self.body_inertia * self.rate
        wheel_momentum = self.wheel_inertia * (self.rate + self.wheel_relative_rate)
        total_momentum = body_momentum + wheel_momentum
        self.external_torque = self._disturbance_torque() - self.damping * self.rate

        # Euler rigid-body dynamics with reaction torque from the motors. The
        # gyroscopic term uses TOTAL body-frame angular momentum, including the
        # rotor's absolute spin (body rate + relative wheel rate).
        gyro = np.cross(self.rate, total_momentum)
        body_accel = (self.external_torque - torque - gyro) / self.body_inertia

        # Wheel rate is relative to the body. The body acceleration is subtracted
        # along each wheel axis to preserve the intended relative-rate definition.
        relative_accel = torque / self.wheel_inertia - body_accel
        self.rate += body_accel * dt
        self.wheel_relative_rate += relative_accel * dt

        # Quaternion integration avoids accumulating Euler-angle singularity errors.
        omega_q = np.array([0.0, *self.rate])
        qdot = 0.5 * _quat_mul(self.quaternion, omega_q)
        self.quaternion = self.quaternion + qdot * dt
        self.quaternion /= np.linalg.norm(self.quaternion)

        self.wheel_angle = (self.wheel_angle + np.rad2deg(self.wheel_relative_rate) * dt) % 360.0
        self.sim_time += dt


class PID:
    """Derivative on measurement, filtered; simple conditional anti-windup."""
    def __init__(self, gains: np.ndarray) -> None:
        self.gains = gains.copy()
        self.reset()

    def reset(self) -> None:
        self.integral = np.zeros(3)
        self.previous_measurement: np.ndarray | None = None
        self.derivative = np.zeros(3)

    def update(self, error: np.ndarray, measurement: np.ndarray,
               dt: float, limit: float | np.ndarray) -> np.ndarray:
        if self.previous_measurement is not None:
            raw = (measurement - self.previous_measurement + 180.) % 360. - 180.
            if getattr(self, 'linear_measurement', False):
                raw = measurement - self.previous_measurement
            self.derivative += (dt / (0.04 + dt)) * (raw / dt - self.derivative)
        self.previous_measurement = measurement.copy()
        candidate_i = self.integral + error * dt
        kp, ki, kd = self.gains.T
        raw_out = kp * error + ki * candidate_i - kd * self.derivative
        bounded = np.clip(raw_out, -limit, limit)
        integrate = np.isclose(raw_out, bounded) | (error * raw_out < 0.)
        self.integral = np.where(integrate, candidate_i, self.integral)
        return np.clip(kp * error + ki * self.integral - kd * self.derivative, -limit, limit)
