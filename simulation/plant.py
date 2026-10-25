"""Illustrative three independent rotational axes; NOT a validated spacecraft.

The body/rotor momentum exchange is modelled per axis. Cross-axis rigid-body
coupling, gimbal gravity, true IMU fusion, electrical commutation and calibrated
hardware parameters are deliberately outside this first GUI demonstrator.
"""
from __future__ import annotations
import numpy as np


class ReactionWheelPlant:
    def __init__(self) -> None:
        self.body_inertia = np.array([0.024, 0.026, 0.030])  # kg*m^2; assumed
        self.wheel_inertia = np.array([0.00035, 0.00035, 0.00040])
        self.damping = 0.0003  # N*m*s/rad; assumed external drag
        self.torque_limit = 0.035  # N*m; demonstration only
        self.speed_limit_rpm = 3000.0  # demonstration only
        self.reset()

    def reset(self, attitude: np.ndarray | None = None) -> None:
        self.attitude = np.array([8., -6., 25.]) if attitude is None else attitude.copy()
        self.rate = np.zeros(3)  # rad/s
        self.wheel_relative_rate = np.zeros(3)  # rad/s relative to body
        self.wheel_angle = np.zeros(3)
        self.motor_torque = np.zeros(3)
        self.saturated = np.zeros(3, dtype=bool)

    @property
    def wheel_rpm(self) -> np.ndarray:
        return self.wheel_relative_rate * 60.0 / (2.0 * np.pi)

    def step(self, requested_motor_torque: np.ndarray, dt: float) -> None:
        torque = np.clip(requested_motor_torque, -self.torque_limit, self.torque_limit)
        self.saturated = np.abs(requested_motor_torque) >= self.torque_limit - 1e-12
        outward = ((np.abs(self.wheel_rpm) >= self.speed_limit_rpm)
                   & (torque * self.wheel_relative_rate > 0.0))
        self.saturated |= outward
        torque[outward] = 0.0
        self.motor_torque = torque.copy()
        # Rotor torque reacts oppositely on the body. Wheel speed is RELATIVE.
        body_accel = (-torque - self.damping * self.rate) / self.body_inertia
        relative_accel = torque / self.wheel_inertia - body_accel
        self.rate += body_accel * dt
        self.wheel_relative_rate += relative_accel * dt
        # Independent-axis, near-equilibrium attitude approximation.
        self.attitude = (self.attitude + np.rad2deg(self.rate) * dt + 180.) % 360. - 180.
        self.wheel_angle = (self.wheel_angle + np.rad2deg(self.wheel_relative_rate) * dt) % 360.


class PID:
    """Derivative on measurement, filtered; simple conditional anti-windup."""
    def __init__(self, gains: np.ndarray) -> None:
        self.gains = gains.copy()  # three rows (axes), columns Kp, Ki, Kd
        self.reset()

    def reset(self) -> None:
        self.integral = np.zeros(3)
        self.previous_measurement: np.ndarray | None = None
        self.derivative = np.zeros(3)

    def update(self, error: np.ndarray, measurement: np.ndarray,
               dt: float, limit: float | np.ndarray) -> np.ndarray:
        if self.previous_measurement is not None:
            raw = (measurement - self.previous_measurement + 180.) % 360. - 180.
            # For angular rate / rpm, ordinary difference is passed below by flag.
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
