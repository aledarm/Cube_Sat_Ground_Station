import numpy as np
from simulation.native_controller import NativeYawController
from simulation.engine import SimulationEngine


def test_team_c_controller_is_actually_loaded():
    c = NativeYawController(auto_build=False)
    c.reset()
    # The branch defines MAX_TARGET_VELOCITY = 1.0 rad/s.
    assert np.isclose(c.yaw_outer(15.0, 0.05), 1.0)
    # Inner loop is capped by MAX_TORQUE_COMMAND = 0.2 N.m.
    assert abs(c.yaw_inner(1.0, 0.0, 0.05)) <= 0.200001
    assert c.damping_body_torque(0.5, 0.004) < 0.0


def test_disarmed_body_does_not_visually_freeze():
    e = SimulationEngine(seed=3)
    initial = e.plant.attitude.copy()
    for _ in range(200):
        e.advance(0.05)
    # No motors are armed, yet small environmental/bench torques evolve attitude.
    assert np.linalg.norm(e.plant.attitude - initial) > 0.05
