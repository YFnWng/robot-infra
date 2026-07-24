import pytest
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from control_interface.msg import DeviceStream
from control_interface_py.manager import ControlManager


def manager(last_pos):
    obj = object.__new__(ControlManager)
    obj._pos_lower = [0.0] * 6
    obj._pos_upper = [40.0] * 6
    obj._vel_min = [2.0] * 6
    obj._vel_max = [10.0] * 6
    obj._position_guard_horizon_s = 0.05
    obj._last_pos = last_pos
    return obj


def test_position_targets_are_clamped():
    out = manager([20.0] * 6)._clamp_command(
        DeviceStream.POS, [-1.0, 20.0, 41.0])
    assert out == [0.0, 20.0, 40.0]


def test_velocity_is_speed_and_position_clamped():
    out = manager([0.1, 39.9])._clamp_command(
        DeviceStream.VEL, [-10.0, 10.0])
    assert out == pytest.approx([-2.0, 2.0])


def test_outside_limit_only_allows_inward_velocity():
    low = manager([-2.0])._clamp_command(DeviceStream.VEL, [-3.0])[0]
    low_inward = manager([-2.0])._clamp_command(DeviceStream.VEL, [3.0])[0]
    high = manager([42.0])._clamp_command(DeviceStream.VEL, [3.0])[0]
    high_inward = manager([42.0])._clamp_command(DeviceStream.VEL, [-3.0])[0]
    assert (low, low_inward, high, high_inward) == (0.0, 3.0, 0.0, -3.0)


def test_nonzero_velocity_is_lifted_to_reliable_minimum():
    out = manager([20.0] * 3)._clamp_command(
        DeviceStream.VEL, [0.0, 0.5, -0.5])
    assert out == [0.0, 2.0, -2.0]


def test_position_guard_stops_instead_of_emitting_subminimum_speed():
    out = manager([39.95])._clamp_command(DeviceStream.VEL, [0.5])
    assert out == [0.0]


def test_stale_source_commands_zero_velocity(monkeypatch):
    published = []
    warnings = []
    obj = object.__new__(ControlManager)
    obj.active_source = "slicer"
    obj.last_input_time = {"slicer": 9.0}
    obj.SOURCE_TIMEOUT = 0.2
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    obj.get_logger = lambda: SimpleNamespace(warn=warnings.append)
    monkeypatch.setattr("control_interface_py.manager.time.time", lambda: 10.0)

    obj._source_watchdog_tick()

    assert obj.active_source is None
    assert len(published) == 1
    assert published[0].predicate == DeviceStream.VEL
    assert list(published[0].data) == [0.0] * 6
    assert warnings


def test_fresh_source_is_not_stopped(monkeypatch):
    published = []
    obj = object.__new__(ControlManager)
    obj.active_source = "autonomy"
    obj.last_input_time = {"autonomy": 9.9}
    obj.SOURCE_TIMEOUT = 0.2
    obj.control_pub = SimpleNamespace(publish=published.append)
    monkeypatch.setattr("control_interface_py.manager.time.time", lambda: 10.0)

    obj._source_watchdog_tick()

    assert obj.active_source == "autonomy"
    assert published == []
