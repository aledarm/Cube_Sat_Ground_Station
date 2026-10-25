"""Main application window.

The Ground Station displays/requests data. Closed-loop attitude control remains
inside the active source: the PC simulation now, and the ESP32 firmware later.
"""
from __future__ import annotations
from pathlib import Path
import time
import traceback

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QFrame, QPushButton,
    QComboBox, QShortcut
)

from core.models import Command, CommandKind
from communication.local_backend import LocalBackend
from simulation.engine import LOOPS
from gui.widgets import label
from gui.style import STYLESHEET
from gui.operations_dashboard import OperationsDashboard


class MainWindow(QMainWindow):
    def __init__(self, project_root: Path | None = None) -> None:
        super().__init__()
        self.root = project_root or Path(__file__).resolve().parents[1]
        self.backend = LocalBackend()
        self.source_mode = 'Simulation'
        self.last_rx_wall = None
        self.received = 0
        self.last_sequence = None
        self.skipped = 0

        self.setWindowTitle('CubeSat Ground Station')
        self.resize(1580, 930)
        self.setMinimumSize(1200, 760)
        self.setStyleSheet(STYLESHEET)

        root = QWidget(); root.setObjectName('Root')
        outer = QVBoxLayout(root); outer.setContentsMargins(0, 0, 0, 0); outer.setSpacing(0)

        header = QFrame(); header.setObjectName('Header')
        top = QHBoxLayout(header); top.setContentsMargins(18, 10, 18, 10); top.setSpacing(10)
        top.addWidget(label('CubeSat Ground Station', 'Brand', False))
        top.addSpacing(18)
        top.addWidget(label('Mode:', 'Muted', False))
        self.source_selector = QComboBox()
        self.source_selector.addItems(['Simulation', 'Live'])
        self.source_selector.setMinimumWidth(140)
        top.addWidget(self.source_selector)
        self.source_badge = label('Receiving dummy telemetry', 'Badge', False)
        top.addWidget(self.source_badge)
        top.addStretch()

        self.arm_button = QPushButton('ARM')
        self.disarm_button = QPushButton('DISARM')
        self.clear_button = QPushButton('CLEAR E-STOP'); self.clear_button.setObjectName('DangerOutline')
        self.stop_button = QPushButton('EMERGENCY STOP'); self.stop_button.setObjectName('Emergency')
        for b in (self.arm_button, self.disarm_button, self.clear_button, self.stop_button): top.addWidget(b)
        outer.addWidget(header)

        self.dashboard = OperationsDashboard()
        outer.addWidget(self.dashboard, 1)
        self.setCentralWidget(root)

        self.footer = label('Simulation source ready.', 'Subtitle', False)
        self.statusBar().addPermanentWidget(self.footer, 1)

        self.source_selector.currentTextChanged.connect(self.change_source)
        self.arm_button.clicked.connect(lambda: self.send_command(Command(CommandKind.ARM)))
        self.disarm_button.clicked.connect(lambda: self.send_command(Command(CommandKind.DISARM)))
        self.clear_button.clicked.connect(lambda: self.send_command(Command(CommandKind.CLEAR_ESTOP)))
        self.stop_button.clicked.connect(lambda: self.send_command(Command(CommandKind.ESTOP)))
        self.dashboard.command_requested.connect(self.send_command)

        self.escape_shortcut = QShortcut(QKeySequence('Escape'), self)
        self.escape_shortcut.setContext(Qt.ApplicationShortcut)
        self.escape_shortcut.setAutoRepeat(False)
        self.escape_shortcut.activated.connect(lambda: self.send_command(Command(CommandKind.ESTOP)))

        self.dashboard.load_sim_gains(
            self.backend.engine.controllers[LOOPS[0]].gains,
            self.backend.engine.controllers[LOOPS[1]].gains,
        )

        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.tick)
        self.timer.start()
        self.refresh_safety()

    def change_source(self, mode: str) -> None:
        self.source_mode = mode
        if mode == 'Simulation':
            self.source_badge.setObjectName('Badge')
            self.source_badge.setStyleSheet('')
            self.source_badge.setText('Receiving dummy telemetry')
            self.dashboard.set_command_enabled(True)
            self.dashboard.set_link_text('SIM', True)
            self.footer.setText('Simulation selected. Local dummy telemetry is active.')
        else:
            self.source_badge.setObjectName('BadgeWarn')
            self.source_badge.setStyleSheet('color:#ffd27c;background:#241b0c;border:1px solid #6c4f1f;border-radius:5px;padding:6px 10px;font-weight:600;')
            self.source_badge.setText('Live adapter not configured')
            self.dashboard.set_command_enabled(False, live_placeholder=True)
            self.dashboard.set_link_text('NO TMTC', False)
            self.footer.setText('Live selected. Awaiting Ricardo/Edson TMTC ICD; no network traffic is sent.')

    def send_command(self, command) -> None:
        if isinstance(command, tuple) and command and command[0] == 'sim_nudge':
            if self.source_mode == 'Simulation' and not self.backend.engine.estop:
                axis, degrees = command[1]
                self.backend.engine.nudge(int(axis), float(degrees))
                self.dashboard.receipt.setText(f'SIM disturbance applied: axis {axis}, {degrees:+.1f} deg.')
            else:
                self.dashboard.receipt.setText('Simulation disturbance unavailable in Live mode or while E-STOP is latched.')
            return
        if self.source_mode == 'Live':
            # Intentional placeholder. Real adapter will serialize Command through the final TMTC ICD.
            self.dashboard.receipt.setText(f'{command.kind.value}: NOT SENT — Live TMTC adapter not configured.')
            return
        result = self.backend.send_command(command)
        self.dashboard.receipt.setText(
            f'{command.kind.value}: ' + ('APPLIED IN SIM' if result.applied else 'REJECTED') + f' — {result.message}'
        )
        if command.kind == CommandKind.GAINS and result.applied:
            self.dashboard.load_sim_gains(
                self.backend.engine.controllers[LOOPS[0]].gains,
                self.backend.engine.controllers[LOOPS[1]].gains,
            )
        self.refresh_safety()

    def refresh_safety(self) -> None:
        e = self.backend.engine
        self.clear_button.setEnabled(e.estop)
        self.arm_button.setEnabled(self.source_mode == 'Simulation' and not e.armed and not e.estop)
        self.disarm_button.setEnabled(self.source_mode == 'Simulation' and e.armed)
        if e.estop:
            self.source_badge.setText('E-STOP LATCHED')
            self.source_badge.setObjectName('BadgeDanger')
            self.source_badge.setStyleSheet('color:#ff9ca6;background:#2c1014;border:1px solid #7d2632;border-radius:5px;padding:6px 10px;font-weight:700;')
        elif self.source_mode == 'Simulation':
            self.source_badge.setText('Simulation armed' if e.armed else 'Receiving dummy telemetry')
            self.source_badge.setStyleSheet('')

    def tick(self) -> None:
        if self.source_mode != 'Simulation':
            self.refresh_safety()
            return
        try:
            sample = self.backend.tick(.05)
            if sample is not None:
                if self.last_sequence is not None and sample.sequence > self.last_sequence:
                    self.skipped += max(0, sample.sequence - self.last_sequence - 1)
                self.last_sequence = sample.sequence
                self.received += 1
                self.last_rx_wall = time.monotonic()
                self.dashboard.update_sample(sample)
            self.refresh_safety()
            self.footer.setText(
                f'Source: LOCAL SIM · received {self.received} · skipped {self.skipped} · '
                f'sim time {self.backend.engine.time:.1f} s · TMTC hardware protocol: pending'
            )
        except Exception as exc:
            self.timer.stop()
            self.backend.send_command(Command(CommandKind.ESTOP))
            self.dashboard.receipt.setText(f'Runtime error: {exc}. Simulation E-STOP latched.')
            traceback.print_exc()
            self.refresh_safety()

    def closeEvent(self, event) -> None:
        self.backend.close()
        event.accept()
