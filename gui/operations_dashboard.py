from __future__ import annotations
from collections import deque
import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QFrame, QLabel, QPushButton,
    QComboBox, QDoubleSpinBox
)

from core.models import Command, CommandKind, Telemetry, angular_error
from gui.widgets import Card, CubeView, CameraView, label, spin
from gui.style import AXIS_COLORS
from simulation.engine import LOOPS


class MiniPanel(QFrame):
    def __init__(self, title: str):
        super().__init__()
        self.setObjectName('MiniPanel')
        self.box = QVBoxLayout(self)
        self.box.setContentsMargins(12, 10, 12, 10)
        self.box.setSpacing(7)
        self.box.addWidget(label(title, 'CardTitle'))

    def add_row(self, name: str, value: str = '--') -> QLabel:
        row = QHBoxLayout()
        row.setSpacing(8)
        row.addWidget(label(name, 'Muted', False))
        row.addStretch()
        value_label = label(value, 'SectionValue', False)
        row.addWidget(value_label)
        self.box.addLayout(row)
        return value_label


class OperationsDashboard(QWidget):
    command_requested = pyqtSignal(object)

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName('Page')
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(12)

        body = QHBoxLayout()
        body.setSpacing(12)
        root.addLayout(body)

        # ---------------- LEFT: TELEMETRY ----------------
        left = QVBoxLayout()
        left.setSpacing(12)
        body.addLayout(left, 11)

        telemetry = Card('Telemetry Summary')
        summary = QHBoxLayout()
        summary.setSpacing(8)

        attitude = MiniPanel('Attitude (deg)')
        self.roll_value = attitude.add_row('Roll', '--')
        self.pitch_value = attitude.add_row('Pitch', '--')
        self.yaw_value = attitude.add_row('Yaw', '--')

        rates = MiniPanel('Angular Rates (deg/s)')
        self.roll_rate = rates.add_row('Roll Rate', '--')
        self.pitch_rate = rates.add_row('Pitch Rate', '--')
        self.yaw_rate = rates.add_row('Yaw Rate', '--')

        wheels = MiniPanel('Reaction Wheels (RPM)')
        self.rw_values = [wheels.add_row(f'RW{i+1}', '--') for i in range(3)]

        system = MiniPanel('System')
        self.battery_value = system.add_row('Bus Voltage', '--')
        self.mode_value = system.add_row('Control Mode', '--')
        self.link_value = system.add_row('Link', 'SIM')
        self.track_value = system.add_row('Tracking', '--')

        for panel in (attitude, rates, wheels, system):
            summary.addWidget(panel, 1)
        telemetry.box.addLayout(summary)
        left.addWidget(telemetry)

        cube_card = Card('Cube Attitude')
        cube_header = QHBoxLayout()
        cube_header.addWidget(label('Live orientation from the current telemetry source.', 'Muted', False))
        cube_header.addStretch()
        self.cube_readout = label('Roll --   Pitch --   Yaw --', 'Muted', False)
        cube_header.addWidget(self.cube_readout)
        cube_card.box.addLayout(cube_header)
        self.cube = CubeView()
        self.cube.setMinimumHeight(270)
        cube_card.box.addWidget(self.cube)
        left.addWidget(cube_card, 3)

        plot_card = Card('Telemetry Plot')
        plot_tools = QHBoxLayout()
        plot_tools.addWidget(label('Quantity:', 'Muted', False))
        self.plot_kind = QComboBox()
        self.plot_kind.addItems(['Roll', 'Pitch', 'Yaw', 'Wheel Speeds'])
        plot_tools.addWidget(self.plot_kind)
        plot_tools.addSpacing(12)
        plot_tools.addWidget(label('Window:', 'Muted', False))
        self.time_window = QComboBox()
        self.time_window.addItems(['15 s', '30 s', '60 s'])
        self.time_window.setCurrentText('30 s')
        plot_tools.addWidget(self.time_window)
        plot_tools.addStretch()
        self.plot_status = label('Awaiting telemetry', 'Muted', False)
        plot_tools.addWidget(self.plot_status)
        plot_card.box.addLayout(plot_tools)

        self.plot = pg.PlotWidget(background='#050505')
        self.plot.setMinimumHeight(235)
        self.plot.showGrid(x=True, y=True, alpha=.16)
        self.plot.setLabel('bottom', 'Time', units='s')
        self.plot.setLabel('left', 'Angle', units='deg')
        self.plot.addLegend(offset=(8, 6), labelTextColor='#bdbdbd')
        self.actual_curve = self.plot.plot(pen=pg.mkPen('#2f8fff', width=2), name='Actual')
        self.target_curve = self.plot.plot(pen=pg.mkPen('#b5b5b5', width=1.5, style=Qt.DashLine), name='Target')
        self.wheel_curves = [self.plot.plot(pen=pg.mkPen(c, width=2), name=f'RW{i+1}') for i, c in enumerate(AXIS_COLORS)]
        for c in self.wheel_curves:
            c.hide()
        plot_card.box.addWidget(self.plot)
        left.addWidget(plot_card, 2)

        # ---------------- RIGHT: OPERATIONS ----------------
        right = QVBoxLayout()
        right.setSpacing(12)
        body.addLayout(right, 9)

        manual = Card('Manual Control')
        manual_grid = QGridLayout()
        manual_grid.setHorizontalSpacing(10)
        headings = ['Target Roll (deg)', 'Target Pitch (deg)', 'Target Yaw (deg)', 'Max Angular Rate (deg/s)']
        for i, text in enumerate(headings):
            manual_grid.addWidget(label(text, 'Muted'), 0, i)
        self.target_inputs = [spin(-180, 180, 0, 1, 1), spin(-75, 75, 0, 1, 1), spin(-180, 180, 0, 1, 1)]
        self.max_rate = spin(.1, 60, 10, 1, 1)
        for i, w in enumerate(self.target_inputs):
            manual_grid.addWidget(w, 1, i)
        manual_grid.addWidget(self.max_rate, 1, 3)
        self.send_setpoint = QPushButton('Send Setpoint')
        self.send_setpoint.setObjectName('Primary')
        manual_grid.addWidget(self.send_setpoint, 1, 4)
        manual.box.addLayout(manual_grid)
        quick = QHBoxLayout()
        self.hold_current = QPushButton('Hold Current')
        self.track_target = QPushButton('Track Camera Target')
        self.disturbance = QPushButton('SIM: +20° Yaw Disturbance')
        quick.addWidget(self.hold_current)
        quick.addWidget(self.track_target)
        quick.addWidget(self.disturbance)
        quick.addStretch()
        manual.box.addLayout(quick)
        self.manual_note = label('Setpoint is sent to the active source. The closed-loop correction remains onboard.', 'Muted')
        manual.box.addWidget(self.manual_note)
        right.addWidget(manual)

        wheel = Card('Wheel Command')
        wg = QGridLayout()
        self.wheel_inputs = []
        for i in range(3):
            wg.addWidget(label(f'RW{i+1} Speed (RPM)', 'Muted'), 0, i)
            w = spin(-3000, 3000, 0, 0, 50)
            self.wheel_inputs.append(w)
            wg.addWidget(w, 1, i)
        self.send_wheels = QPushButton('Send Wheel Command')
        self.send_wheels.setObjectName('Primary')
        wg.addWidget(self.send_wheels, 1, 3)
        wheel.box.addLayout(wg)
        wheel.box.addWidget(label('Bench-test command. Final limits and feedback fields must be confirmed with Edson.', 'Muted'))
        right.addWidget(wheel)

        tuning = Card('Control Tuning')
        tg = QGridLayout()
        tg.setHorizontalSpacing(16)
        tg.addWidget(label('Attitude Loop (outer)', 'CardTitle'), 0, 0, 1, 3)
        tg.addWidget(label('Rate Loop (inner)', 'CardTitle'), 0, 4, 1, 3)
        for col, name in enumerate(('Kp','Ki','Kd')):
            tg.addWidget(label(name, 'Muted', False), 1, col)
            tg.addWidget(label(name, 'Muted', False), 1, col+4)
        self.attitude_gains = [spin(0, 20, 0, 4, .01) for _ in range(3)]
        self.rate_gains = [spin(0, 1, 0, 6, .001) for _ in range(3)]
        for i, w in enumerate(self.attitude_gains): tg.addWidget(w, 2, i)
        for i, w in enumerate(self.rate_gains): tg.addWidget(w, 2, i+4)
        self.apply_gains = QPushButton('Apply Gains')
        self.apply_gains.setObjectName('Primary')
        tg.addWidget(self.apply_gains, 2, 7)
        tuning.box.addLayout(tg)
        self.gain_note = label('These fields configure the onboard controller; the Ground Station does not close the attitude loop.', 'Muted')
        tuning.box.addWidget(self.gain_note)
        right.addWidget(tuning)

        camera = Card('Camera View')
        cam_header = QHBoxLayout()
        cam_header.addWidget(label('Reserved for onboard camera stream / processed target view.', 'Muted', False))
        cam_header.addStretch()
        self.camera_status = label('Target: --', 'Muted', False)
        cam_header.addWidget(self.camera_status)
        camera.box.addLayout(cam_header)
        cam_body = QHBoxLayout()
        self.camera = CameraView()
        self.camera.setMinimumHeight(260)
        cam_body.addWidget(self.camera, 4)
        cam_stats = MiniPanel('Tracking')
        self.target_detected = cam_stats.add_row('Detected', '--')
        self.target_x = cam_stats.add_row('Target X', '--')
        self.target_y = cam_stats.add_row('Target Y', '--')
        self.target_error = cam_stats.add_row('Error', '--')
        cam_body.addWidget(cam_stats, 1)
        camera.box.addLayout(cam_body)
        right.addWidget(camera, 3)

        self.receipt = label('No command sent yet.', 'Muted')
        right.addWidget(self.receipt)

        self.samples: deque[Telemetry] = deque(maxlen=1200)
        self.plot_kind.currentTextChanged.connect(self.refresh_plot)
        self.time_window.currentTextChanged.connect(self.refresh_plot)
        self.send_setpoint.clicked.connect(self._send_attitude)
        self.send_wheels.clicked.connect(self._send_wheel_command)
        self.apply_gains.clicked.connect(self._send_gains)
        self.hold_current.clicked.connect(lambda: self.command_requested.emit(Command(CommandKind.HOLD)))
        self.track_target.clicked.connect(lambda: self.command_requested.emit(Command(CommandKind.TRACK)))
        self.disturbance.clicked.connect(lambda: self.command_requested.emit(('sim_nudge', (2, 20.0))))

    def _send_attitude(self) -> None:
        rate = self.max_rate.value()
        self.command_requested.emit(Command(CommandKind.ATTITUDE, {
            'attitude_deg': [w.value() for w in self.target_inputs],
            'max_rate_deg_s': [rate, rate, rate],
        }))

    def _send_wheel_command(self) -> None:
        self.command_requested.emit(Command(CommandKind.WHEEL_SPEED, {
            'wheel_rpm': [w.value() for w in self.wheel_inputs]
        }))

    def _send_gains(self) -> None:
        # Current SIM engine stores gains per axis. For this first UI we apply the same
        # requested outer/inner PID triplet to all three axes. This is explicit and easy
        # to replace once the team's ICD defines the real command schema.
        outer = np.array([[w.value() for w in self.attitude_gains]] * 3, dtype=float)
        inner = np.array([[w.value() for w in self.rate_gains]] * 3, dtype=float)
        self.command_requested.emit(Command(CommandKind.GAINS, {'loop': LOOPS[0], 'gains': outer.tolist()}))
        self.command_requested.emit(Command(CommandKind.GAINS, {'loop': LOOPS[1], 'gains': inner.tolist()}))

    def load_sim_gains(self, outer: np.ndarray, inner: np.ndarray) -> None:
        # UI displays one shared triplet; simulation currently supports axis-specific matrices.
        for w, v in zip(self.attitude_gains, outer[0]): w.setValue(float(v))
        for w, v in zip(self.rate_gains, inner[0]): w.setValue(float(v))

    def set_command_enabled(self, enabled: bool, live_placeholder: bool = False) -> None:
        for w in [self.send_setpoint, self.send_wheels, self.apply_gains, self.hold_current, self.track_target, self.disturbance]:
            w.setEnabled(enabled)
        if live_placeholder:
            self.receipt.setText('LIVE mode selected. TMTC adapter is intentionally not configured yet.')

    def reset_history(self) -> None:
        self.samples.clear()
        self.actual_curve.setData([], [])
        self.target_curve.setData([], [])
        for c in self.wheel_curves: c.setData([], [])

    def update_sample(self, t: Telemetry) -> None:
        self.samples.append(t)
        vals = [self.roll_value, self.pitch_value, self.yaw_value]
        for widget, value, color in zip(vals, t.attitude_deg, AXIS_COLORS):
            widget.setText(f'{value:+.1f}')
            widget.setStyleSheet(f'color:{color};')
        rate_widgets = [self.roll_rate, self.pitch_rate, self.yaw_rate]
        for widget, value, color in zip(rate_widgets, t.rate_deg_s, AXIS_COLORS):
            widget.setText(f'{value:+.2f}')
            widget.setStyleSheet(f'color:{color};')
        for widget, value, color in zip(self.rw_values, t.wheel_rpm, AXIS_COLORS):
            widget.setText(f'{value:+.0f}')
            widget.setStyleSheet(f'color:{color};')
        self.battery_value.setText('--' if t.battery_v is None else f'{t.battery_v:.2f} V')
        self.mode_value.setText(t.mode)
        self.track_value.setText('LOCKED' if t.target_detected else 'NO TARGET')
        self.cube.set_attitude(t.attitude_deg)
        self.cube_readout.setText('Roll {:+.1f}°   Pitch {:+.1f}°   Yaw {:+.1f}°'.format(*t.attitude_deg))
        self.camera.set_frame(t.frame, t.target_uv)
        if t.target_detected and t.target_uv is not None:
            self.target_detected.setText('YES')
            self.target_detected.setStyleSheet('color:#4ee58a;')
            self.target_x.setText(f'{t.target_uv[0]:.1f} px')
            self.target_y.setText(f'{t.target_uv[1]:.1f} px')
            if t.visual_error_deg is not None:
                self.target_error.setText(f'{np.hypot(*t.visual_error_deg):.2f}°')
            self.camera_status.setText('Target detected')
        else:
            self.target_detected.setText('NO')
            self.target_x.setText('--')
            self.target_y.setText('--')
            self.target_error.setText('--')
            self.camera_status.setText('No target')
        self.plot_status.setText(f'{t.source} · seq {t.sequence}')
        self.refresh_plot()

    def refresh_plot(self, *_args) -> None:
        if not self.samples:
            return
        window = float(self.time_window.currentText().split()[0])
        latest = self.samples[-1].sim_time_s
        samples = [s for s in self.samples if latest - s.sim_time_s <= window]
        times = np.array([s.sim_time_s for s in samples])
        kind = self.plot_kind.currentText()
        if kind == 'Wheel Speeds':
            self.actual_curve.hide(); self.target_curve.hide()
            for i, curve in enumerate(self.wheel_curves):
                curve.show(); curve.setData(times, [s.wheel_rpm[i] for s in samples])
            self.plot.setLabel('left', 'Wheel speed', units='rpm')
        else:
            for c in self.wheel_curves: c.hide()
            self.actual_curve.show(); self.target_curve.show()
            idx = {'Roll':0, 'Pitch':1, 'Yaw':2}[kind]
            self.actual_curve.setData(times, [s.attitude_deg[idx] for s in samples])
            self.target_curve.setData(times, [s.reference_deg[idx] for s in samples])
            self.actual_curve.setPen(pg.mkPen(AXIS_COLORS[idx], width=2))
            self.plot.setLabel('left', f'{kind} angle', units='deg')

    def set_link_text(self, text: str, ok: bool = True) -> None:
        self.link_value.setText(text)
        self.link_value.setStyleSheet('color:#4ee58a;' if ok else 'color:#ffd24a;')
