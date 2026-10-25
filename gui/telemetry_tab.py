from __future__ import annotations
from collections import deque
import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QComboBox, QTableWidgetItem, QPushButton
import pyqtgraph as pg
from core.models import Telemetry, angular_error
from gui.widgets import Card, StatTile, CubeView, CameraView, label, make_table
from gui.style import AXIS_COLORS


class TelemetryTab(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName('Page')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 20)
        layout.setSpacing(14)
        heading = QHBoxLayout()
        heading.addWidget(label('Telemetry', 'PageTitle'))
        heading.addStretch()
        self.freshness = label('Awaiting local telemetry', 'Badge', False)
        heading.addWidget(self.freshness)
        layout.addLayout(heading)
        layout.addWidget(label('One view of all three axes. Every measurement on this screen is simulated.'))
        row = QHBoxLayout()
        self.stats = []
        for title, unit, color in zip(('Roll', 'Pitch', 'Yaw', 'Max attitude error', 'Battery'),
                                      ('deg', 'deg', 'deg', 'deg', 'V'),
                                      (*AXIS_COLORS, '#e8f0fa', '#e8f0fa')):
            item = StatTile(title, unit, color)
            self.stats.append(item)
            row.addWidget(item)
        layout.addLayout(row)
        visuals = QHBoxLayout()
        cube = Card('Cube attitude', 'X / Roll  |  Y / Pitch  |  Z / Yaw')
        self.cube = CubeView()
        cube.box.addWidget(self.cube)
        self.rate_text = label('Angular rates: --')
        cube.box.addWidget(self.rate_text)
        camera = Card('Camera & target', 'Image processing is inside the virtual onboard module.')
        self.camera = CameraView()
        camera.box.addWidget(self.camera)
        self.camera_text = label('Target: --')
        camera.box.addWidget(self.camera_text)
        visuals.addWidget(cube, 1)
        visuals.addWidget(camera, 1)
        layout.addLayout(visuals)
        lower = QHBoxLayout()
        chart = Card('Telemetry history', 'Last 30 simulated seconds. Dashed curves are commanded references.')
        self.plot_kind = QComboBox()
        self.plot_kind.addItems(['Attitude', 'Angular rate', 'Wheel speed', 'Attitude error'])
        chart.box.addWidget(self.plot_kind)
        self.plot = pg.PlotWidget(background='#111d2e')
        self.plot.setMinimumHeight(180)
        self.plot.showGrid(x=True, y=True, alpha=.18)
        self.plot.setLabel('bottom', 'Simulation time', units='s')
        self.plot.setLabel('left', 'Angle', units='deg')
        self.plot.addLegend(offset=(8, 6), labelTextColor='#bdd0e4')
        self.curves, self.references = [], []
        for name, color in zip(('Roll / X', 'Pitch / Y', 'Yaw / Z'), AXIS_COLORS):
            self.curves.append(self.plot.plot(pen=pg.mkPen(color, width=2), name=name))
            self.references.append(self.plot.plot(pen=pg.mkPen(color, width=1, style=Qt.DashLine)))
        chart.box.addWidget(self.plot)
        wheels = Card('Reaction wheels', 'Three independent, orthogonal wheel channels.')
        self.wheels = make_table(['Axis', 'rpm', 'Angle deg', 'Torque Nm'], 3)
        wheels.box.addWidget(self.wheels)
        self.wheel_limit = label('Assumed SIM limit: 3,000 rpm / 0.035 Nm.\nWheel angle is diagnostic, not the pointing target.')
        wheels.box.addWidget(self.wheel_limit)
        self.mode_text = label('Mode: --')
        wheels.box.addWidget(self.mode_text)
        self.saturation = label('Saturation: --')
        wheels.box.addWidget(self.saturation)
        wheels.box.addStretch()
        lower.addWidget(chart, 3)
        lower.addWidget(wheels, 2)
        layout.addLayout(lower)
        self.sensor_toggle = QPushButton('Show raw IMU & diagnostics')
        self.sensor_toggle.setCheckable(True)
        layout.addWidget(self.sensor_toggle, alignment=Qt.AlignLeft)
        self.sensor_card = Card('Raw sensor channels', 'Synthetic values, not a calibrated sensor or a tested fusion algorithm.')
        self.sensors = make_table(['Signal', 'X', 'Y', 'Z'], 3)
        self.sensor_card.box.addWidget(self.sensors)
        self.sequence_text = label('Sequence: --')
        self.sensor_card.box.addWidget(self.sequence_text)
        layout.addWidget(self.sensor_card)
        self.sensor_card.hide()
        self.sensor_toggle.toggled.connect(self.sensor_card.setVisible)
        self.sensor_toggle.toggled.connect(lambda checked: self.sensor_toggle.setText(
            'Hide raw IMU & diagnostics' if checked else 'Show raw IMU & diagnostics'))
        layout.addStretch()
        self.samples: deque[Telemetry] = deque(maxlen=600)
        self.plot_kind.currentIndexChanged.connect(self.refresh_plot)

    def reset_history(self) -> None:
        self.samples.clear()
        for curve in self.curves + self.references:
            curve.setData([], [])

    def update_sample(self, t: Telemetry) -> None:
        self.samples.append(t)
        while len(self.samples) > 1 and t.sim_time_s - self.samples[0].sim_time_s > 30.:
            self.samples.popleft()
        error = angular_error(t.reference_deg, t.attitude_deg)
        numbers = [*t.attitude_deg, np.max(np.abs(error)), t.battery_v]
        for tile, value in zip(self.stats, numbers):
            tile.value.setText('--' if value is None else f'{value:.2f}')
        self.cube.set_attitude(t.attitude_deg)
        self.camera.set_frame(t.frame, t.target_uv)
        self.rate_text.setText('Angular rate [deg/s]   X {:+.2f}   Y {:+.2f}   Z {:+.2f}'.format(*t.rate_deg_s))
        if t.target_detected and t.target_uv is not None:
            self.camera_text.setText('DETECTED  |  u {:.1f}, v {:.1f} px  |  image error {:+.2f}, {:+.2f} deg'.format(
                *t.target_uv, *(t.visual_error_deg or (0., 0.))))
        else:
            self.camera_text.setText('TARGET NOT DETECTED  |  onboard holds last reference, then searches')
        for r in range(3):
            values = [('Roll / X', 'Pitch / Y', 'Yaw / Z')[r], f'{t.wheel_rpm[r]:+.1f}',
                      f'{t.wheel_angle_deg[r]:.1f}', f'{t.motor_torque_nm[r]:+.4f}']
            for c, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                self.wheels.setItem(r, c, item)
        self.mode_text.setText(f'Mode: {t.mode}')
        self.saturation.setText('Wheel limit: ' + ('SATURATED' if np.any(t.saturated) else 'within SIM limits'))
        for r, (name, vals) in enumerate([('Gyro [deg/s]', t.gyro_deg_s), ('Accel [m/s2]', t.accel_m_s2), ('Mag [uT]', t.mag_ut)]):
            for c, value in enumerate([name, *(f'{v:+.3f}' for v in vals)]):
                self.sensors.setItem(r, c, QTableWidgetItem(value))
        self.sequence_text.setText(f'Source: {t.source}  |  sequence {t.sequence}  |  timestamp {t.sim_time_s:.2f} s (SIM clock)')
        self.refresh_plot()

    def refresh_plot(self, *_args) -> None:
        if not self.samples:
            return
        samples = list(self.samples)
        times = np.array([t.sim_time_s for t in samples])
        kind = self.plot_kind.currentText()
        if kind == 'Attitude':
            values = np.array([t.attitude_deg for t in samples]); unit = 'deg'; name = 'Angle'
        elif kind == 'Angular rate':
            values = np.array([t.rate_deg_s for t in samples]); unit = 'deg/s'; name = 'Angular rate'
        elif kind == 'Wheel speed':
            values = np.array([t.wheel_rpm for t in samples]); unit = 'rpm'; name = 'Wheel speed'
        else:
            values = np.array([angular_error(t.reference_deg, t.attitude_deg) for t in samples]); unit = 'deg'; name = 'Error'
        refs = np.array([t.reference_deg for t in samples])
        for i in range(3):
            self.curves[i].setData(times, values[:, i])
            self.references[i].setVisible(kind == 'Attitude')
            if kind == 'Attitude':
                self.references[i].setData(times, refs[:, i])
        self.plot.setLabel('left', name, units=unit)
