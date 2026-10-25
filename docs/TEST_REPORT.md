# Test report - v0.1

## Executed in the delivery environment

- Interpreter: Python 3.13.5 (Linux).
- NumPy: 2.3.5.
- Syntax: every Python file checked with `python -m compileall -q .`.
- Command: `python -m pytest -q`.
- Result: **39 passed, 1 module skipped**.
- Skipped module: `tests/test_gui.py`, because PyQt5 is not installed in this execution environment and dependency installation could not reach the package index.

The three GUI smoke tests are included for execution in Alessia's configured environment. **The windows have not been launched or visually verified here; Windows execution is not yet verified.** This is a limitation of this delivery's verification, not an assertion that those tests passed.

## Covered logic

- All three attitude references, input limits, finite-number validation and angle wrapping.
- Initially disarmed state; latched E-STOP; no automatic re-arm; zero torque without pretending that inertia disappears.
- Command rejection while latched; gain changes only while disarmed; duplicate command IDs.
- Wheel-speed test as a separate mode; actuator speed/torque limiting.
- Per-axis momentum conservation when external drag is removed.
- Pixel-based target detection, offscreen target rejection, occlusion and absence of a roll reference from one centred point.
- Continued virtual onboard control during telemetry dropout.
- Reproducible acquisition, hold, disturbance, moving-target and occlusion scenarios.
- No fabricated success from the unconfigured hardware adapter.
- CSV export and bounded history.

## Scenario observations from a separate deterministic run

| Scenario | Observed in the assumed model |
|---|---|
| Acquisition | SIM PASS at 2.40 simulated seconds with the declared capture/dwell criterion |
| Hold | SIM PASS after 60 simulated seconds |
| Disturbance | SIM PASS, approximately 1.75 simulated seconds after the displacement |
| Moving target | REVIEW, approximately 1.24 deg peak model-truth error in the 20 s example |
| Occlusion | REVIEW, target reacquired after the 2 s hide interval |

These are **not measurements of the user's cuboid, ESP32 or reaction wheels**. Parameters are illustrative and the controller is a substitute for GUI testing. Do not report these numbers as project hardware performance.

## Manual visual acceptance on Windows

Run `main.py` with the existing Python 3.12 virtual environment. Check that all tabs fit or scroll, all three axis controls appear, the persistent emergency button remains visible, plots and the synthetic camera update, and the simulator's command receipt changes appropriately. Then run `python -m pytest -q` in that same environment to include the GUI smoke tests.
