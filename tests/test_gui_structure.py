from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_main_window_uses_operations_dashboard():
    text = (ROOT / 'gui' / 'main_window.py').read_text(encoding='utf-8')
    assert 'OperationsDashboard' in text
    assert "['Simulation', 'Live']" in text
    assert 'EMERGENCY STOP' in text


def test_operations_has_three_axis_commands_and_camera():
    text = (ROOT / 'gui' / 'operations_dashboard.py').read_text(encoding='utf-8')
    for name in ('Target Roll', 'Target Pitch', 'Target Yaw', 'Camera View', 'Reaction Wheels', 'Simulation Target', 'Desired image X'):
        assert name in text
