import csv
import numpy as np
import pytest
from core.models import Command, CommandKind as C
from communication.local_backend import LocalBackend
from communication.protocol_adapter import ProtocolAdapter, ProtocolNotConfigured
from communication.udp_transport import UdpTransport
from core.recording import TelemetryHistory
from simulation.scenarios import SCENARIOS


def test_telemetry_dropout_does_not_stop_onboard_control():
    b=LocalBackend(); b.send_command(Command(C.ARM))
    first=b.tick(); initial=b.engine.plant.attitude.copy()
    b.engine.dropout_until=b.engine.time+1.
    assert b.tick() is None
    for _ in range(25): b.tick()
    final=b.tick()
    assert final is not None and final.sequence>first.sequence+20
    assert np.linalg.norm(b.engine.plant.attitude-initial)>1.


def test_stop_works_while_simulation_is_paused():
    b=LocalBackend(); b.send_command(Command(C.ARM)); b.paused=True
    assert b.send_command(Command(C.ESTOP)).applied
    assert b.engine.estop and not b.engine.armed


@pytest.mark.parametrize('scenario,expected', list(zip(SCENARIOS[1:],['SIM PASS','SIM PASS','SIM PASS','REVIEW','REVIEW'])))
def test_repeatable_scenarios(scenario,expected):
    b=LocalBackend(); b.scenario.load(scenario); b.send_command(Command(C.ARM))
    for _ in range(1300):
        b.tick()
        if b.scenario.status not in ('READY','RUNNING'): break
    assert b.scenario.status==expected, b.scenario.detail


def test_stop_aborts_automatic_score():
    b=LocalBackend(); b.scenario.load(SCENARIOS[1]); b.send_command(Command(C.ARM)); b.tick()
    b.send_command(Command(C.ESTOP)); b.tick()
    assert b.scenario.status=='ABORTED'


def test_hardware_adapter_never_fakes_success():
    adapter=ProtocolAdapter()
    with pytest.raises(ProtocolNotConfigured): adapter.encode_command(Command(C.ARM))
    with pytest.raises(ProtocolNotConfigured): adapter.decode_telemetry(b'anything')
    with pytest.raises(ProtocolNotConfigured): adapter.decode_ack(b'anything')
    with pytest.raises(ProtocolNotConfigured): adapter.decode_camera_frame([])


def test_closed_transport_does_not_send():
    with pytest.raises(RuntimeError): UdpTransport().send(b'test')


def test_csv_export(tmp_path):
    b=LocalBackend(); h=TelemetryHistory(max_samples=5)
    for _ in range(8): h.append(b.tick())
    path=h.export(tmp_path)
    with path.open() as f: rows=list(csv.DictReader(f))
    assert len(rows)==5
    assert rows[-1]['source']=='LOCAL SIMULATOR'
    assert 'wheel_pitch_angle_deg' in rows[-1]
    assert 'truth_yaw_deg' in rows[-1]
