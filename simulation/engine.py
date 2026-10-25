"""Virtual onboard controller. The GUI never calculates the actuator correction.

This class lives in the PC simulator now. It is NOT firmware and does not replace
Mauricio/Ana's controller. Real operation will use the ESP32 instead of this class.
"""
from __future__ import annotations
import numpy as np
from core.models import Command, CommandKind, CommandResult, Telemetry, angular_error, vector3
from simulation.plant import ReactionWheelPlant, PID
from simulation.camera import VirtualCamera, rotation_matrix

LOOPS = ('Attitude -> angular rate', 'Angular rate -> body torque', 'Wheel speed bench test')
DEFAULT_GAINS = {
    LOOPS[0]: np.array([[2., 0., 0.], [2., 0., 0.], [2., 0., 0.]]),
    LOOPS[1]: np.array([[0.0035, 0., 0.], [0.0035, 0., 0.], [0.0035, 0., 0.]]),
    LOOPS[2]: np.array([[0.000035, 0., 0.], [0.000035, 0., 0.], [0.000035, 0., 0.]])
}


class SimulationEngine:
    def __init__(self, seed: int = 7) -> None:
        self.rng = np.random.default_rng(seed)
        self.plant = ReactionWheelPlant()
        self.camera = VirtualCamera()
        self.controllers = {key: PID(value) for key, value in DEFAULT_GAINS.items()}
        self.controllers[LOOPS[1]].linear_measurement = True
        self.controllers[LOOPS[2]].linear_measurement = True
        self.estop = False
        self.noise_deg = 0.04
        self.time = 0.
        self.sequence = 0
        self.armed = False
        self.mode = 'ATTITUDE HOLD'
        self.reference = np.zeros(3)
        self.rate_limits = np.full(3, 15.)
        self.wheel_setpoint = np.zeros(3)
        self.target = np.zeros(3)
        self.desired_visual_offset_deg = np.zeros(2)
        self.tracking_profile = 'yaw_only'
        self.moving_target = False
        self.occluded_until = 0.
        self.dropout_until = 0.
        self.target_uv = None
        self.image = None
        self.last_seen = 0.
        self.tracking_roll = 0.
        self._seen_ids: dict[str, CommandResult] = {}
        self._refresh_camera()

    def reset_controller_memory(self) -> None:
        for pid in self.controllers.values():
            pid.reset()

    def command(self, command: Command) -> CommandResult:
        # Locally idempotent: replaying a command ID does not reapply an operation.
        if command.command_id in self._seen_ids:
            return self._seen_ids[command.command_id]
        try:
            message = self._apply(command)
            result = CommandResult(command.command_id, True, True, message)
        except (ValueError, KeyError, TypeError) as exc:
            result = CommandResult(command.command_id, False, False, str(exc))
        self._seen_ids[command.command_id] = result
        if len(self._seen_ids) > 256:
            del self._seen_ids[next(iter(self._seen_ids))]
        return result

    def _apply(self, command: Command) -> str:
        kind, p = command.kind, command.payload
        if kind == CommandKind.ESTOP:
            self.estop, self.armed = True, False
            self.plant.motor_torque[:] = 0.
            self.reset_controller_memory()
            return 'E-STOP LATCHED. SIM torque off; body and wheels may coast.'
        if kind == CommandKind.DISARM:
            self.armed = False
            self.plant.motor_torque[:] = 0.
            self.reset_controller_memory()
            return 'SIM disarmed. Motor torque disabled.'
        if kind == CommandKind.CLEAR_ESTOP:
            self.estop, self.armed = False, False
            self.reset_controller_memory()
            return 'Latch cleared. Still DISARMED; explicit Arm SIM is required.'
        if self.estop:
            raise ValueError('Blocked: clear the E-STOP latch first. No automatic re-arm.')
        if kind == CommandKind.ARM:
            self.armed = True
            self.reset_controller_memory()
            return 'Virtual onboard controller ARMED. Simulation only.'
        if kind == CommandKind.GAINS:
            if self.armed:
                raise ValueError('Disarm SIM before changing gains.')
            loop = p.get('loop')
            if loop not in self.controllers:
                raise ValueError('Unknown control loop.')
            gains = np.asarray(p.get('gains'), dtype=float)
            maxima = np.array([20., 5., 2.]) if loop == LOOPS[0] else np.array([1., 1., 1.])
            if gains.shape != (3, 3) or not np.isfinite(gains).all() or (gains < 0).any() or (gains > maxima).any():
                raise ValueError('Invalid gains: use finite non-negative values within the SIM limits.')
            self.controllers[loop].gains = gains.copy()
            self.controllers[loop].reset()
            return f'Gains applied in SIM: {loop}. Not hardware-approved gains.'
        if kind == CommandKind.ATTITUDE:
            reference = vector3(p['attitude_deg'], 'Attitude', 180.)
            limits = vector3(p['max_rate_deg_s'], 'Rate limits', 60.)
            if (limits <= 0).any():
                raise ValueError('Rate limits must be greater than zero.')
            if abs(reference[1]) > 75.:
                raise ValueError('This small-angle demo limits pitch targets to +/-75 deg.')
            self.reference, self.rate_limits = reference, limits
            self.mode = 'ATTITUDE HOLD'
            self.reset_controller_memory()
            return 'Roll/Pitch/Yaw target and slew-rate limits applied in SIM.'
        if kind == CommandKind.HOLD:
            self.reference = self.plant.attitude.copy()
            self.mode = 'ATTITUDE HOLD'
            self.reset_controller_memory()
            return 'Current attitude copied to the reference in SIM.'
        if kind == CommandKind.SIM_TARGET:
            pitch = float(p.get('target_pitch_deg', 0.))
            yaw = float(p.get('target_yaw_deg', 0.))
            offset_x = float(p.get('desired_x_deg', 0.))
            offset_y = float(p.get('desired_y_deg', 0.))
            values = np.array([pitch, yaw, offset_x, offset_y], dtype=float)
            if not np.isfinite(values).all():
                raise ValueError('Simulation target values must be finite.')
            if abs(pitch) > 75. or abs(yaw) > 180.:
                raise ValueError('Simulation target must stay within Pitch +/-75 deg and Yaw +/-180 deg.')
            if abs(offset_x) > 25. or abs(offset_y) > 25.:
                raise ValueError('Desired camera offset is limited to +/-25 deg in this simulator.')
            self.target[:] = [0., pitch, yaw]
            self.desired_visual_offset_deg[:] = [offset_x, offset_y]
            self.moving_target = bool(p.get('moving', False))
            profile = str(p.get('tracking_profile', 'yaw_only'))
            if profile not in ('yaw_only', 'three_axis'):
                raise ValueError('Unknown simulation tracking profile.')
            self.tracking_profile = profile
            self._refresh_camera()
            return (f'SIM target placed at Pitch {pitch:+.1f} deg, Yaw {yaw:+.1f} deg; '
                    f'desired image offset X {offset_x:+.1f} deg, Y {offset_y:+.1f} deg.')
        if kind == CommandKind.TRACK:
            self.mode = 'TARGET TRACKING'
            self.tracking_roll = 0.
            self.last_seen = self.time
            self.reset_controller_memory()
            self._refresh_camera()
            return 'Tracking enabled in SIM. Camera points Pitch/Yaw; inertial Roll reference is 0 deg.'
        if kind == CommandKind.WHEEL_SPEED:
            self.wheel_setpoint = vector3(p['wheel_rpm'], 'Wheel speeds', 2000.)
            self.mode = 'WHEEL BENCH TEST'
            self.reset_controller_memory()
            return 'Wheel-speed test selected in SIM; attitude hold is NOT active.'
        raise ValueError(f'Unsupported command: {kind}')

    def _refresh_camera(self) -> None:
        self.image = self.camera.render(self.plant.attitude, self.target,
                                        self.time >= self.occluded_until, self.rng)
        self.target_uv = self.camera.detect(self.image)
        if self.mode == 'TARGET TRACKING' and self.target_uv is not None:
            # Image + idealized onboard attitude estimate, not target truth, drives control.
            visual_reference = self.camera.pointing_reference(
                self.target_uv, self.plant.attitude, self.tracking_roll,
                tuple(self.desired_visual_offset_deg))
            if self.tracking_profile == 'yaw_only':
                # Original assignment: camera closes only Yaw; Roll/Pitch are
                # damped toward zero rather than used as free pointing axes.
                self.reference = np.array([0., 0., visual_reference[2]])
            else:
                # Team extension for the confirmed 3-wheel hardware.
                self.reference = visual_reference
            self.last_seen = self.time

    def place_target_from_image_uv(self, uv: tuple[float, float]) -> tuple[float, float]:
        """Place the synthetic target so it appears at a clicked image location.

        The click defines where the point is *currently* seen in the camera. The
        simulator converts that image ray and the current body attitude into a
        new world target direction. Desired image X/Y remains a separate command:
        after switching to tracking, the controller will move the point toward
        that desired image location.
        """
        u, v = map(float, uv)
        if not np.isfinite([u, v]).all():
            raise ValueError('Clicked image coordinates must be finite.')
        if not (0.0 <= u <= self.camera.width - 1 and 0.0 <= v <= self.camera.height - 1):
            raise ValueError('Clicked image coordinates are outside the camera frame.')
        ray = np.array([1., (u - self.camera.cx) / self.camera.focal, -(v - self.camera.cy) / self.camera.focal])
        world = rotation_matrix(self.plant.attitude) @ ray
        pitch = float(np.rad2deg(np.arctan2(-world[2], np.hypot(world[0], world[1]))))
        yaw = float(np.rad2deg(np.arctan2(world[1], world[0])))
        self.target[:] = [0., pitch, yaw]
        self._refresh_camera()
        return pitch, yaw

    def advance(self, duration: float = .05) -> None:
        if not np.isfinite(duration) or duration <= 0 or duration > 1.:
            raise ValueError('Simulation advance must be in (0, 1] seconds.')
        steps = max(1, int(np.ceil(duration / .01)))
        dt = duration / steps
        for _ in range(steps):
            if self.moving_target:
                self.target[2] = 12. * np.sin(self.time * .20)
                self.target[1] = 5. * np.sin(self.time * .12)
            motor_torque = np.zeros(3)
            if self.armed and not self.estop:
                if self.mode == 'TARGET TRACKING' and self.time - self.last_seen > 3.:
                    # Slow exploratory scan after a longer target loss. A 2 s occlusion
                    # holds the last reference instead; it does not depend on ground commands.
                    self.reference[2] = 45. * np.sin((self.time - self.last_seen - 3.) * .25)
                if self.mode == 'WHEEL BENCH TEST':
                    motor_torque = self.controllers[LOOPS[2]].update(
                        self.wheel_setpoint - self.plant.wheel_rpm,
                        self.plant.wheel_rpm, dt, self.plant.torque_limit)
                else:
                    error = angular_error(self.reference, self.plant.attitude)
                    rate_ref = self.controllers[LOOPS[0]].update(
                        error, self.plant.attitude, dt, self.rate_limits)
                    rates = np.rad2deg(self.plant.rate)
                    body_torque = self.controllers[LOOPS[1]].update(
                        rate_ref - rates, rates, dt, self.plant.torque_limit)
                    motor_torque = -body_torque
            self.plant.step(motor_torque, dt)
            self.time += dt
        self._refresh_camera()

    def snapshot(self) -> Telemetry:
        self.sequence += 1
        truth = self.plant.attitude.copy()
        attitude = truth + self.rng.normal(0, self.noise_deg, 3)
        rate = np.rad2deg(self.plant.rate)
        rot = rotation_matrix(truth)
        truth_reference = self.target.copy() if self.mode == 'TARGET TRACKING' else self.reference.copy()
        if self.mode == 'TARGET TRACKING':
            truth_reference[0] = self.tracking_roll
        return Telemetry(
            sequence=self.sequence, sim_time_s=self.time,
            attitude_deg=attitude, reference_deg=self.reference.copy(),
            rate_deg_s=rate.copy(), wheel_rpm=self.plant.wheel_rpm.copy(),
            wheel_angle_deg=self.plant.wheel_angle.copy(), motor_torque_nm=self.plant.motor_torque.copy(),
            gyro_deg_s=rate + self.rng.normal(0, self.noise_deg * 2, 3),
            accel_m_s2=rot.T @ np.array([0., 0., 9.80665]) + self.rng.normal(0, .015, 3),
            mag_ut=rot.T @ np.array([20., 0., 42.]),
            battery_v=max(22., 24.4 - self.time * .0003),
            armed=self.armed, estop=self.estop, mode=self.mode,
            saturated=self.plant.saturated.copy(), target_detected=self.target_uv is not None,
            target_uv=self.target_uv,
            visual_error_deg=self.camera.visual_error(self.target_uv) if self.target_uv is not None else None,
            frame=self.image.copy() if self.image is not None else None,
            truth_attitude_deg=truth, truth_error_deg=angular_error(truth_reference, truth))

    def nudge(self, axis: int, degrees: float = 20.) -> None:
        if axis not in (0, 1, 2) or not np.isfinite(degrees):
            raise ValueError('Invalid disturbance.')
        delta = np.zeros(3)
        delta[axis] = float(degrees)
        self.nudge_vector(delta)

    def nudge_vector(self, delta_deg: np.ndarray) -> None:
        """Apply a user disturbance to the virtual body orientation.

        This is intentionally a kinematic shove for the demonstrator, not a
        calibrated force/impulse model. The controller then has to recover from
        the displaced attitude using the normal simulated telemetry path.
        """
        delta = vector3(delta_deg, 'Disturbance', 45.)
        self.plant.attitude[:] = (self.plant.attitude + delta + 180.) % 360. - 180.
        self._refresh_camera()

    def load_initial_state(self, attitude: np.ndarray) -> None:
        if self.estop or self.armed:
            raise ValueError('Disarm and clear the E-STOP latch before loading a scenario.')
        self.plant.reset(vector3(attitude, 'Initial attitude', 180.))
        self.time, self.sequence = 0., 0
        self.reference = np.zeros(3)
        self.mode = 'ATTITUDE HOLD'
        self.target = np.zeros(3)
        self.desired_visual_offset_deg = np.zeros(2)
        self.tracking_profile = 'yaw_only'
        self.moving_target = False
        self.occluded_until = self.dropout_until = 0.
        self.last_seen = 0.
        self.reset_controller_memory()
        self._refresh_camera()
