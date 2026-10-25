from __future__ import annotations
import numpy as np
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton,
                             QComboBox, QPlainTextEdit)
from core.models import Command, CommandKind, Telemetry, AXES
from gui.widgets import Card, label, spin


class ControlTab(QWidget):
    command_requested = pyqtSignal(object)
    read_gains_requested = pyqtSignal(str)

    def __init__(self, loops: tuple[str, ...]) -> None:
        super().__init__()
        self.setObjectName('Page')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 20)
        layout.setSpacing(14)
        layout.addWidget(label('Manual control & tuning', 'PageTitle'))
        layout.addWidget(label('Send references and configuration. The virtual onboard module closes the control loops.'))
        state = Card('Actuation state')
        toolbar = QHBoxLayout()
        self.state = label('DISARMED', 'Badge', False)
        self.arm_button = QPushButton('Arm SIM')
        self.arm_button.setObjectName('Primary')
        self.disarm_button = QPushButton('Disarm SIM')
        self.hold_button = QPushButton('Hold current attitude')
        self.track_button = QPushButton('Track camera target')
        toolbar.addWidget(self.state)
        for b in [self.arm_button, self.disarm_button, self.hold_button, self.track_button]:
            toolbar.addWidget(b)
        toolbar.addStretch()
        state.box.addLayout(toolbar)
        self.applied_target = label('Applied reference [deg]: Roll --  /  Pitch --  /  Yaw --')
        state.box.addWidget(self.applied_target)
        layout.addWidget(state)
        self.arm_button.clicked.connect(lambda: self.request(CommandKind.ARM))
        self.disarm_button.clicked.connect(lambda: self.request(CommandKind.DISARM))
        self.hold_button.clicked.connect(lambda: self.request(CommandKind.HOLD))
        self.track_button.clicked.connect(lambda: self.request(CommandKind.TRACK))
        columns = QHBoxLayout()
        left = QVBoxLayout()
        attitude = Card('Attitude setpoint', 'Fixed attitude plus a maximum slew rate for each axis.')
        grid = QGridLayout()
        for col, text in enumerate(['Axis', 'Target [deg]', 'Max rate [deg/s]']):
            grid.addWidget(label(text), 0, col)
        self.target_inputs, self.rate_inputs = [], []
        for row, name in enumerate(AXES, 1):
            grid.addWidget(label(name, 'CardTitle'), row, 0)
            lo, hi = (-75., 75.) if row == 2 else (-180., 180.)
            target = spin(lo, hi, 0., step=1.)
            rate = spin(.1, 60., 15., step=1.)
            target.setToolTip('Demo attitude convention: Rz(Yaw) Ry(Pitch) Rx(Roll).')
            rate.setToolTip('Maximum body rotation rate, NOT reaction-wheel rpm.')
            self.target_inputs.append(target); self.rate_inputs.append(rate)
            grid.addWidget(target, row, 1); grid.addWidget(rate, row, 2)
        attitude.box.addLayout(grid)
        self.send_attitude = QPushButton('Send attitude setpoint')
        self.send_attitude.setObjectName('Primary')
        attitude.box.addWidget(self.send_attitude)
        self.send_attitude.clicked.connect(lambda: self.request(CommandKind.ATTITUDE, {
            'attitude_deg': [w.value() for w in self.target_inputs],
            'max_rate_deg_s': [w.value() for w in self.rate_inputs]}))
        attitude.box.addWidget(label('The target is stored while disarmed; motion starts only after Arm SIM.\nPitch is limited to +/-75 deg in this demonstrator.'))
        left.addWidget(attitude)
        bench = Card('Wheel-speed bench test', 'Separate test mode: attitude hold is disabled.')
        bench_grid = QGridLayout()
        self.wheel_inputs = []
        for i, axis in enumerate(AXES):
            bench_grid.addWidget(label(axis), i, 0)
            field = spin(-2000., 2000., 0., decimals=0, step=50.)
            field.setSuffix(' rpm')
            bench_grid.addWidget(field, i, 1)
            self.wheel_inputs.append(field)
        bench.box.addLayout(bench_grid)
        self.send_wheels = QPushButton('Apply wheel speeds (SIM only)')
        self.send_wheels.clicked.connect(lambda: self.request(CommandKind.WHEEL_SPEED, {
            'wheel_rpm': [w.value() for w in self.wheel_inputs]}))
        bench.box.addWidget(self.send_wheels)
        bench.box.addWidget(label('These are rotor speed commands, not cube orientation commands.\nNo motor-controller hardware interface is enabled.', 'Warning'))
        left.addWidget(bench)
        right = QVBoxLayout()
        tuning = Card('Controller gains', 'SIM model values only. Real loop structure and units remain pending.')
        self.loop_combo = QComboBox()
        self.loop_combo.addItems(list(loops))
        tuning.box.addWidget(self.loop_combo)
        self.gain_units = label('')
        tuning.box.addWidget(self.gain_units)
        gains_grid = QGridLayout()
        for c, name in enumerate(['Axis', 'Kp', 'Ki', 'Kd']):
            gains_grid.addWidget(label(name), 0, c)
        self.gain_inputs = []
        for r, name in enumerate(AXES, 1):
            gains_grid.addWidget(label(name, 'CardTitle'), r, 0)
            row = []
            for c in range(1, 4):
                field = spin(0., 20., 0., decimals=6, step=.001)
                field.setMinimumWidth(95)
                field.valueChanged.connect(self._mark_edited)
                row.append(field)
                gains_grid.addWidget(field, r, c)
            self.gain_inputs.append(row)
        tuning.box.addLayout(gains_grid)
        gain_buttons = QHBoxLayout()
        self.read_gains = QPushButton('Read applied (SIM)')
        self.apply_gains = QPushButton('Apply gains (SIM)')
        self.apply_gains.setObjectName('Primary')
        gain_buttons.addWidget(self.read_gains); gain_buttons.addWidget(self.apply_gains)
        tuning.box.addLayout(gain_buttons)
        self.gain_status = label('Values are not yet loaded.')
        tuning.box.addWidget(self.gain_status)
        tuning.box.addWidget(label('Gain changes require DISARMED state in this version.\nApplying gains does not arm the controller.', 'Warning'))
        self.read_gains.clicked.connect(lambda: self.read_gains_requested.emit(self.loop_combo.currentText()))
        self.loop_combo.currentTextChanged.connect(self.read_gains_requested.emit)
        self.apply_gains.clicked.connect(lambda: self.request(CommandKind.GAINS, {
            'loop': self.loop_combo.currentText(),
            'gains': [[f.value() for f in row] for row in self.gain_inputs]}))
        right.addWidget(tuning)
        receipt = Card('Command receipt', 'Requested is not the same as applied.')
        self.receipt = label('No command sent yet.')
        self.receipt.setTextInteractionFlags(Qt.TextSelectableByMouse)
        receipt.box.addWidget(self.receipt)
        receipt.box.addWidget(label('This version confirms commands inside the local simulator.\nReal receipt, acceptance and application require the ESP32 ICD.'))
        right.addWidget(receipt)
        right.addStretch()
        columns.addLayout(left, 1)
        columns.addLayout(right, 1)
        layout.addLayout(columns)
        events = Card('Event log', 'Operator actions and simulator confirmations. No hidden hardware writes.')
        self.event_log = QPlainTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumBlockCount(500)
        self.event_log.setMinimumHeight(120)
        events.box.addWidget(self.event_log)
        layout.addWidget(events)
        layout.addStretch()
        self._loading_gains = False

    def request(self, kind: CommandKind, payload: dict | None = None) -> None:
        self.command_requested.emit(Command(kind, payload or {}))

    def _mark_edited(self, *_args) -> None:
        if not getattr(self, '_loading_gains', False) and hasattr(self, 'gain_status'):
            self.gain_status.setText('EDITED / NOT SENT. Click Apply gains to request a change.')

    def load_gains(self, values: np.ndarray) -> None:
        self._loading_gains = True
        loop_index = self.loop_combo.currentIndex()
        limits = [20., 5., 2.] if loop_index == 0 else [1., 1., 1.]
        for r, row in enumerate(self.gain_inputs):
            for c, field in enumerate(row):
                field.setMaximum(limits[c])
                field.setSingleStep(.1 if loop_index == 0 else (.0001 if loop_index == 1 else .000005))
                field.setValue(float(values[r, c]))
        units = [
            'Input: angle [deg] -> output: rate [deg/s]. Kp [1/s], Ki [1/s2], Kd [1].',
            'Input: rate [deg/s] -> output: body torque [Nm]. Kp [Nm s/deg], Ki [Nm/deg], Kd [Nm s2/deg].',
            'Input: wheel speed [rpm] -> output: motor torque [Nm]. Kp [Nm/rpm], Ki [Nm/(rpm s)], Kd [Nm s/rpm].'
        ]
        self.gain_units.setText(units[loop_index])
        self.gain_status.setText('CONFIRMED / applied in the local SIM. These are not validated hardware gains.')
        self._loading_gains = False

    def update_safety(self, armed: bool, estop: bool, mode: str) -> None:
        self.state.setText('E-STOP LATCHED' if estop else ('ARMED / SIM' if armed else 'DISARMED / SIM'))
        self.arm_button.setEnabled(not armed and not estop)
        self.disarm_button.setEnabled(armed)
        for b in [self.hold_button, self.track_button, self.send_attitude, self.send_wheels]:
            b.setEnabled(not estop)
        self.apply_gains.setEnabled(not armed and not estop)

    def update_sample(self, t: Telemetry) -> None:
        self.applied_target.setText('Applied reference [deg]:  Roll {:+.2f}  /  Pitch {:+.2f}  /  Yaw {:+.2f}'.format(*t.reference_deg))
