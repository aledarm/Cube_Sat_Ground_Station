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

## v0.3 - Interactive simulation additions

- The cube attitude panel is telemetry-driven and uses a projected 3-D body with rotating body axes.
- In Simulation mode, drag the cube with the left mouse button to apply Pitch/Yaw disturbances. Hold Shift while dragging horizontally to disturb Roll.
- A Simulation Target panel places the virtual poster point by world Pitch/Yaw and defines a desired camera-image offset.
- Default visual-control profile is **Original brief - Yaw only**: camera error closes Yaw, while Roll/Pitch damp toward zero. A clearly labelled **3-axis extension** is available for the team's confirmed three-wheel hardware exploration.
- Original-project scenarios are exposed in the GUI: 30 deg acquisition, 60 s hold, 20 deg disturbance/recovery, moving target, and 2 s occlusion.
- The simulator remains a test model. It does not replace the onboard ESP32 control implementation or the final TMTC ICD.

## Native C controller integration (v0.4)

The desktop simulator now uses the team's `feature/attitude-rate-control-level0`
C implementation for the functionality that actually exists in that branch:

- yaw outer loop (`calcular_malha_externa_yaw`)
- yaw rate/torque loop (`calcular_malha_intermedia_yaw`)
- roll/pitch rate damping (`calcular_damping_roll_pitch`)

The files are kept under `native_control/`. A prebuilt 64-bit Windows DLL is
included in `native_control/bin/controlo_atitude.dll`; the Linux test build is
also included for verification in this package. To rebuild on another machine:

```powershell
python native_control/build_native.py
```

The branch does **not** contain rigid-body dynamics. The PC-only plant physics
therefore remain in `simulation/plant.py`, now using rigid-body gyroscopic
coupling, wheel angular momentum, quaternion attitude integration, actuator
limits and small deterministic environmental/bench disturbance torques.
Disarming or E-STOP removes motor torque; it does not freeze the simulated body.

`Original brief — Yaw only` uses the native C yaw controller and C roll/pitch
damping. `3-axis extension` remains a separate Python simulator controller until
the team provides an equivalent three-axis implementation in firmware/C.

The Camera View layout and click-to-place-target behaviour are unchanged.

## v0.4 mockup-fidelity update

- Preserves the existing two-column Operations layout.
- Camera View remains the large lower-right panel and retains click-to-place target behaviour.
- Cube Attitude now uses a lightweight solid low-poly QPainter model with body panels, structural rails, reaction-wheel hubs, antennas and body axes.
- Added local 3D-view toggles for axes, grid and antennas without adding OpenGL dependencies.
- Native C controller and simulation physics integration from v0.4 are unchanged.
