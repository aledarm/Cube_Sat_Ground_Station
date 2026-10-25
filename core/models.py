"""Application data, independent of the GUI and the future wire protocol.

All displayed angles are degrees; rates are degrees/second. A real adapter must
convert firmware units explicitly. These are NOT the team's TMTC packet fields.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Any
from uuid import uuid4
import numpy as np

AXES = ('Roll / X', 'Pitch / Y', 'Yaw / Z')


class CommandKind(str, Enum):
    ARM = 'arm'
    DISARM = 'disarm'
    ATTITUDE = 'attitude_setpoint'
    HOLD = 'hold_current_attitude'
    TRACK = 'track_target'
    SIM_TARGET = 'simulation_target'
    WHEEL_SPEED = 'wheel_speed_test'
    GAINS = 'set_gains'
    ESTOP = 'emergency_stop'
    CLEAR_ESTOP = 'clear_emergency_latch'


@dataclass(frozen=True)
class Command:
    kind: CommandKind
    payload: dict[str, Any] = field(default_factory=dict)
    command_id: str = field(default_factory=lambda: uuid4().hex[:12])


@dataclass(frozen=True)
class CommandResult:
    command_id: str
    accepted: bool
    applied: bool
    message: str
    source: str = 'LOCAL SIMULATOR'


@dataclass
class Telemetry:
    sequence: int
    sim_time_s: float
    attitude_deg: np.ndarray
    reference_deg: np.ndarray
    rate_deg_s: np.ndarray
    wheel_rpm: np.ndarray
    wheel_angle_deg: np.ndarray
    motor_torque_nm: np.ndarray
    gyro_deg_s: np.ndarray
    accel_m_s2: np.ndarray
    mag_ut: np.ndarray
    battery_v: float | None
    armed: bool
    estop: bool
    mode: str
    saturated: np.ndarray
    target_detected: bool
    target_uv: tuple[float, float] | None
    visual_error_deg: tuple[float, float] | None
    frame: np.ndarray | None
    truth_attitude_deg: np.ndarray
    truth_error_deg: np.ndarray
    source: str = 'LOCAL SIMULATOR'


def angular_error(target: np.ndarray, actual: np.ndarray) -> np.ndarray:
    """Shortest signed angular difference, in degrees, for this demo model."""
    return (np.asarray(target) - np.asarray(actual) + 180.0) % 360.0 - 180.0


def vector3(value: Any, name: str, limit: float | None = None) -> np.ndarray:
    """Reject missing, non-finite or incorrectly shaped command values."""
    x = np.asarray(value, dtype=float)
    if x.shape != (3,) or not np.isfinite(x).all():
        raise ValueError(f'{name}: provide three finite numbers.')
    if limit is not None and (np.abs(x) > limit).any():
        raise ValueError(f'{name}: absolute value must not exceed {limit:g}.')
    return x.copy()
