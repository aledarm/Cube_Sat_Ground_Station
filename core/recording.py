"""Bounded in-memory history for plots/export; no hidden network or file uploads."""
from __future__ import annotations
from collections import deque
import csv
from datetime import datetime
from pathlib import Path
from core.models import Telemetry, angular_error


class TelemetryHistory:
    def __init__(self, max_samples: int = 24000) -> None:
        self.rows: deque[dict] = deque(maxlen=max_samples)  # 20 minutes at 20 Hz

    def append(self, t: Telemetry) -> None:
        row = dict(sim_time_s=t.sim_time_s, sequence=t.sequence, source=t.source,
                   mode=t.mode, armed=int(t.armed), estop=int(t.estop), battery_v=t.battery_v,
                   target_detected=int(t.target_detected))
        error = angular_error(t.reference_deg, t.attitude_deg)
        for i, axis in enumerate(('roll', 'pitch', 'yaw')):
            row.update({f'{axis}_deg': t.attitude_deg[i], f'{axis}_ref_deg': t.reference_deg[i],
                        f'{axis}_error_deg': error[i], f'{axis}_rate_deg_s': t.rate_deg_s[i],
                        f'wheel_{axis}_rpm': t.wheel_rpm[i],
                        f'wheel_{axis}_angle_deg': t.wheel_angle_deg[i],
                        f'wheel_{axis}_torque_nm': t.motor_torque_nm[i],
                        f'truth_{axis}_deg': t.truth_attitude_deg[i]})
        self.rows.append(row)

    def export(self, directory: Path) -> Path:
        if not self.rows:
            raise ValueError('No telemetry to export yet.')
        directory.mkdir(parents=True, exist_ok=True)
        filename = directory / ('sim_telemetry_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.csv')
        with filename.open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)
        return filename
