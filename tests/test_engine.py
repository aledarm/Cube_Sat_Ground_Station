import numpy as np
import pytest
from core.models import Command, CommandKind as C, angular_error
from simulation.engine import SimulationEngine, LOOPS
from simulation.plant import ReactionWheelPlant


def cmd(engine, kind, **payload):
    return engine.command(Command(kind, payload))


def test_initially_disarmed():
    e=SimulationEngine(); e.advance()
    assert not e.armed and not e.estop
    np.testing.assert_array_equal(e.plant.motor_torque, 0.)


def test_all_three_attitude_axes_respond():
    e=SimulationEngine()
    assert cmd(e, C.ATTITUDE, attitude_deg=[12,-8,45], max_rate_deg_s=[15,15,15]).applied
    cmd(e, C.ARM)
    for _ in range(200): e.advance(.05)
    assert np.max(np.abs(angular_error(np.array([12,-8,45]), e.plant.attitude))) < .1


def test_stop_latches_and_does_not_freeze_motion():
    e=SimulationEngine(); cmd(e, C.ARM)
    for _ in range(8): e.advance()
    before=e.plant.rate.copy()
    assert np.linalg.norm(before)>0
    cmd(e, C.ESTOP)
    assert not e.armed and e.estop
    e.advance()
    np.testing.assert_allclose(e.plant.motor_torque, 0)
    assert np.linalg.norm(e.plant.rate)>0  # Torque-off is not magical mechanical braking.
    assert not cmd(e, C.ARM).accepted
    cmd(e, C.CLEAR_ESTOP)
    assert not e.armed and not e.estop
    assert cmd(e, C.ARM).applied


@pytest.mark.parametrize('kind,payload', [
    (C.ATTITUDE, {'attitude_deg':[0,0,10], 'max_rate_deg_s':[5,5,5]}),
    (C.TRACK, {}), (C.WHEEL_SPEED, {'wheel_rpm':[0,10,0]}),
    (C.GAINS, {'loop':LOOPS[0], 'gains':np.zeros((3,3)).tolist()}), (C.HOLD,{})])
def test_all_actuation_commands_blocked_while_latched(kind,payload):
    e=SimulationEngine(); cmd(e,C.ESTOP)
    assert not e.command(Command(kind,payload)).accepted
    assert e.estop and not e.armed


def test_scenario_reset_cannot_bypass_stop():
    e=SimulationEngine(); cmd(e,C.ESTOP)
    with pytest.raises(ValueError): e.load_initial_state(np.zeros(3))
    assert e.estop


def test_gains_require_disarm_and_confirm_applied_values():
    e=SimulationEngine(); cmd(e,C.ARM)
    gains=np.tile([1.5,0.,0.],(3,1))
    assert not cmd(e,C.GAINS,loop=LOOPS[0],gains=gains).accepted
    cmd(e,C.DISARM)
    assert cmd(e,C.GAINS,loop=LOOPS[0],gains=gains).applied
    np.testing.assert_array_equal(e.controllers[LOOPS[0]].gains,gains)
    gains[0,0]=float('nan')
    assert not cmd(e,C.GAINS,loop=LOOPS[0],gains=gains).accepted


def test_command_id_is_not_applied_twice():
    e=SimulationEngine(); request=Command(C.ARM)
    e.command(request); cmd(e,C.ESTOP)
    e.command(request)  # cached receipt, but NO second arm
    assert not e.armed and e.estop


@pytest.mark.parametrize('payload', [
    {'attitude_deg':[0,0,0], 'max_rate_deg_s':[0,15,15]},
    {'attitude_deg':[0,90,0], 'max_rate_deg_s':[15,15,15]},
    {'attitude_deg':[0,float('inf'),0], 'max_rate_deg_s':[15,15,15]},
    {'attitude_deg':[0,0,0], 'max_rate_deg_s':[100,15,15]}])
def test_invalid_setpoint_rejected_without_mutation(payload):
    e=SimulationEngine(); original=e.reference.copy()
    assert not e.command(Command(C.ATTITUDE,payload)).accepted
    np.testing.assert_array_equal(e.reference, original)


def test_wheel_speed_mode_is_separate():
    e=SimulationEngine()
    assert cmd(e,C.WHEEL_SPEED,wheel_rpm=[200,-100,150]).applied
    assert e.mode=='WHEEL BENCH TEST'
    cmd(e,C.ARM)
    for _ in range(120): e.advance()
    np.testing.assert_allclose(e.plant.wheel_rpm,[200,-100,150],atol=10)


def test_momentum_is_conserved_without_external_drag():
    p=ReactionWheelPlant(); p.damping=0
    for _ in range(100): p.step(np.array([.01,-.015,.005]),.01)
    h=p.body_inertia*p.rate + p.wheel_inertia*(p.rate+p.wheel_relative_rate)
    np.testing.assert_allclose(h,0.,atol=1e-12)


def test_outward_torque_blocked_at_wheel_limit():
    p=ReactionWheelPlant()
    p.wheel_relative_rate[:]=3100*2*np.pi/60
    p.step(np.full(3,.01),.01)
    np.testing.assert_allclose(p.motor_torque,0.)
    assert p.saturated.all()


def test_simulation_target_configuration_and_mouse_vector_nudge():
    e=SimulationEngine()
    result=cmd(e,C.SIM_TARGET,target_pitch_deg=8,target_yaw_deg=30,
               desired_x_deg=4,desired_y_deg=-2,moving=False)
    assert result.applied
    np.testing.assert_allclose(e.target,[0,8,30])
    np.testing.assert_allclose(e.desired_visual_offset_deg,[4,-2])
    before=e.plant.attitude.copy()
    e.nudge_vector([3,-4,5])
    np.testing.assert_allclose(e.plant.attitude-before,[3,-4,5])


def test_tracking_from_thirty_degrees_reduces_visual_error():
    e=SimulationEngine(seed=3)
    cmd(e,C.SIM_TARGET,target_pitch_deg=0,target_yaw_deg=0,
        desired_x_deg=0,desired_y_deg=0,moving=False)
    e.load_initial_state(np.array([0.,0.,30.]))
    cmd(e,C.TRACK); cmd(e,C.ARM)
    first=e.snapshot()
    assert first.target_detected
    first_error=np.hypot(*first.visual_error_deg)
    for _ in range(120):
        e.advance(.05)
    final=e.snapshot()
    assert final.target_detected
    assert np.hypot(*final.visual_error_deg) < first_error


def test_place_target_from_camera_click_anywhere_in_frame():
    e = SimulationEngine(seed=5)
    pitch, yaw = e.place_target_from_image_uv((40.0, 200.0))
    assert np.isfinite([pitch, yaw]).all()
    snap = e.snapshot()
    assert snap.target_detected
    u, v = snap.target_uv
    assert abs(u - 40.0) < 2.0
    assert abs(v - 200.0) < 2.0
