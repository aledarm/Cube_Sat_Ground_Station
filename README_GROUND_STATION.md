# CubeSat Ground Station — v0.2

Ground Station prototype in Python / PyQt5 / pyqtgraph for the reaction-wheel attitude-control cube.

## Architecture

- `main.py`: application entry point.
- `gui/`: visual interface only.
- `core/`: telemetry and command data models.
- `communication/`: communication abstraction. Real TMTC is intentionally pending.
- `simulation/`: local virtual onboard controller, plant and synthetic camera.
- `tests/`: simulator/data-model tests.

The Ground Station does **not** close the attitude-control loop. In Simulation mode the local virtual onboard module closes the loop. In future Live mode the ESP32 firmware will do it onboard.

## Current UI

One two-column Operations screen:

- black professional theme;
- top-level source dropdown: `Simulation` / `Live`;
- always-visible ARM / DISARM / CLEAR E-STOP / EMERGENCY STOP;
- telemetry for Roll/Pitch/Yaw, angular rates, three reaction wheels and system state;
- animated cube driven by the received telemetry object;
- actual-vs-target plots;
- manual Roll/Pitch/Yaw setpoints and slew-rate limit;
- wheel-speed test commands;
- attitude-loop and rate-loop gain fields;
- camera view and target detection data;
- local target tracking and disturbance demonstration.

`Live` is deliberately a placeholder until the TMTC ICD is agreed. Selecting it sends no packets.

## Run on Windows / PyCharm

From the project directory:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-gs.txt
.\.venv\Scripts\python.exe main.py
```

## First simulation demo

1. Leave `Mode = Simulation`.
2. Click `ARM`.
3. Set Roll/Pitch/Yaw and press `Send Setpoint`.
4. Observe the cube and plot move from dummy telemetry.
5. Press `Track Camera Target` to let the virtual onboard controller use camera target information.
6. Use `SIM: +20° Yaw Disturbance` to demonstrate recovery.
7. Press `EMERGENCY STOP` at any time. Clear the latch and ARM explicitly to resume.

## Important pending definitions

Before Live integration, confirm with the team:

- TMTC UDP packet schema, CRC and ACK semantics;
- motor-controller telemetry variables and units;
- reaction-wheel command limits;
- exact onboard controller gain structure and units;
- camera streaming/target-data format.
