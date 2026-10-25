"""Repeatable demonstration scenarios. Scores use simulation time and truth.

These tests exercise an assumed model, not the real firmware or acceptance of
hardware. A 2 deg capture band for acquisition/recovery is our explicit choice;
the project only gives those manoeuvres a time bound. Moving-target and occlusion
have no complete numerical rubric, so they finish with REVIEW, not an invented PASS.
"""
from __future__ import annotations
import numpy as np
from core.models import Command, CommandKind
from simulation.engine import SimulationEngine

SCENARIOS = (
    'Free exploration - three axes',
    'Acquisition - 30 deg / under 6 s',
    'Hold - +/-2 deg for 60 s',
    'Disturbance - 20 deg / under 4 s',
    'Moving target - slow Pitch/Yaw motion',
    'Occlusion - hide target for 2 s',
)


class ScenarioRunner:
    def __init__(self, engine: SimulationEngine) -> None:
        self.engine = engine
        self.selected = SCENARIOS[0]
        self.status = 'MANUAL'
        self.detail = 'Choose a scenario, load it, then Arm SIM.'
        self.start_time: float | None = None
        self.event_sent = False
        self.elapsed = 0.
        self.max_error = 0.
        self.in_band = 0.
        self._last_time = 0.

    def load(self, name: str) -> None:
        if name not in SCENARIOS:
            raise ValueError('Unknown scenario.')
        initial = np.array([8., -6., 25.]) if name == SCENARIOS[0] else np.zeros(3)
        if name == SCENARIOS[1]:
            initial[2] = 30.
        self.engine.load_initial_state(initial)
        self.selected = name
        self.status = 'READY' if name != SCENARIOS[0] else 'MANUAL'
        self.detail = 'Loaded, DISARMED. Press Arm SIM to begin.'
        self.start_time = None
        self.event_sent = False
        self.elapsed = self.max_error = self.in_band = self._last_time = 0.
        if name != SCENARIOS[0]:
            self.engine.command(Command(CommandKind.TRACK))
        if name == SCENARIOS[4]:
            self.engine.moving_target = True

    def before_step(self) -> None:
        if self.status not in ('READY', 'RUNNING'):
            return
        if self.status == 'READY' and self.engine.armed:
            self.start_time = self.engine.time
            self._last_time = self.engine.time
            self.status = 'RUNNING'
            self.detail = 'Evaluating the assumed model, not the real hardware.'
        if self.status != 'RUNNING':
            return
        if not self.engine.armed or self.engine.estop:
            self.status = 'ABORTED'
            self.detail = 'Operator disarmed or latched E-STOP. Reload to repeat.'
            return
        self.elapsed = self.engine.time - (self.start_time or 0.)
        if not self.event_sent:
            if self.selected == SCENARIOS[3] and self.elapsed >= 1.:
                self.engine.nudge(2, 20.)
                self.event_sent = True
            elif self.selected == SCENARIOS[5] and self.elapsed >= 2.:
                self.engine.occluded_until = self.engine.time + 2.
                self.event_sent = True

    def after_step(self) -> None:
        if self.status != 'RUNNING':
            return
        dt = self.engine.time - self._last_time
        self._last_time = self.engine.time
        self.elapsed = self.engine.time - (self.start_time or 0.)
        # No snapshot() here: scoring must not consume telemetry sequence numbers.
        from core.models import angular_error
        reference = self.engine.target.copy()
        reference[0] = self.engine.tracking_roll
        error = float(np.max(np.abs(angular_error(reference, self.engine.plant.attitude))))
        self.max_error = max(self.max_error, error)
        self.in_band = self.in_band + dt if error <= 2. else 0.
        name = self.selected
        if name == SCENARIOS[1]:
            if self.in_band >= .25 and self.elapsed < 6.:
                self._finish('SIM PASS', f'Entered +/-2 deg and stayed 0.25 s; t={self.elapsed:.2f} s.')
            elif self.elapsed >= 6.:
                self._finish('SIM FAIL', 'Capture criterion not met before 6 simulated seconds.')
        elif name == SCENARIOS[2]:
            if error > 2.:
                self._finish('SIM FAIL', 'Exceeded +/-2 deg during the 60 s observation.')
            elif self.elapsed >= 60.:
                self._finish('SIM PASS', f'60 s inside +/-2 deg; peak error {self.max_error:.2f} deg.')
        elif name == SCENARIOS[3] and self.event_sent:
            if self.in_band >= .25 and 1.25 < self.elapsed < 5.:
                self._finish('SIM PASS', f'Recovery after nudge: {self.elapsed - 1.:.2f} s.')
            elif self.elapsed >= 5.:
                self._finish('SIM FAIL', 'Recovery criterion not met before 4 s after the nudge.')
        elif name == SCENARIOS[4] and self.elapsed >= 20.:
            self._finish('REVIEW', f'20 s trace complete; peak error {self.max_error:.2f} deg. Review tracking.')
        elif name == SCENARIOS[5] and self.elapsed >= 8.:
            reacquired = 'yes' if self.engine.target_uv is not None else 'no'
            self._finish('REVIEW', f'2 s occlusion exercised; reacquired={reacquired}. Review trace.')

    def _finish(self, status: str, detail: str) -> None:
        self.status, self.detail = status, detail

    def invalidate(self, reason: str) -> None:
        if self.status in ('READY', 'RUNNING'):
            self.status, self.detail = 'ABORTED', reason
