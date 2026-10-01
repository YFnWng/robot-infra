import pytest
from types import SimpleNamespace
from collections import defaultdict
import threading

from builtin_interfaces.msg import Time
from control_interface.msg import (
    ControlStream, DeviceEvent, DeviceStream, ManagerEvent)
from control_interface.srv import DeviceCmd
from control_interface_py.manager import (
    ControlManager, parse_driver_diagnostic, parse_fault_status)


def manager(last_pos):
    obj = object.__new__(ControlManager)
    obj._pos_lower = [0.0] * 6
    obj._pos_upper = [40.0] * 6
    obj._vel_min = [2.0] * 6
    obj._vel_max = [10.0] * 6
    obj._feedback_limit_tolerance = [0.001] * 6
    obj._position_guard_horizon_s = 0.05
    obj._last_pos = last_pos
    obj._last_pos_time = 10.0
    obj._last_enc_time = 10.0
    obj.FEEDBACK_TIMEOUT = 0.25
    obj._fault_latched = False
    obj._fault_reason = ""
    obj._last_safety_status = None
    obj._transport_ready = True
    obj._driver_power_qualified = True
    obj.allow_debug_commands = False
    obj.allowed_sources = {"slicer", "autonomy"}
    obj.COMMAND_MAX_AGE = 0.1
    obj.COMMAND_FUTURE_TOLERANCE = 0.05
    obj._last_command_stamp_ns = defaultdict(lambda: 0)
    obj.safety_status_pub = SimpleNamespace(publish=lambda msg: None)
    return obj


def clock_at(seconds):
    whole = int(seconds)
    nanosec = int(round((seconds - whole) * 1e9))
    return SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=int(round(seconds * 1e9)),
        to_msg=lambda: Time(sec=whole, nanosec=nanosec)))


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


def _recovery_manager(position):
    obj = manager(position)
    obj.LIMIT_RECOVERY_MAX_VIOLATION = 0.25
    obj.LIMIT_RECOVERY_INTERIOR_MARGIN = 0.10
    obj.LIMIT_RECOVERY_TIMEOUT = 0.50
    obj.LIMIT_RECOVERY_COMMAND_RATE = 100.0
    obj.LIMIT_RECOVERY_OUTWARD_TOLERANCE = 0.02
    return obj


def test_limit_recovery_plan_is_inward_and_bounded_to_axis_zero():
    obj = _recovery_manager([40.022, 5.0, 5.0, 5.0, 5.0, 5.0])

    axis, boundary, excursion, target, speed = (
        ControlManager._limit_recovery_plan(obj))

    assert axis == 0
    assert boundary == 'upper'
    assert excursion == pytest.approx(0.022)
    assert target == pytest.approx(39.9)
    assert speed == pytest.approx(-2.0)


@pytest.mark.parametrize('position, expected', [
    ([40.30, 5.0, 5.0, 5.0, 5.0, 5.0], 'exceeds bounded'),
    ([5.0, 40.1, 5.0, 5.0, 5.0, 5.0], 'restricted to'),
    ([40.022, 40.1, 5.0, 5.0, 5.0, 5.0], 'exactly one'),
    ([5.0] * 6, 'exactly one'),
])
def test_limit_recovery_plan_rejects_generic_or_ambiguous_motion(
        position, expected):
    obj = _recovery_manager(position)

    with pytest.raises(ValueError, match=expected):
        ControlManager._limit_recovery_plan(obj)


def test_limit_recovery_service_emits_only_inward_axis_zero_then_stops(
        monkeypatch):
    published = []
    sleep_count = 0
    obj = _recovery_manager([40.022, 5.0, 5.0, 5.0, 5.0, 5.0])
    obj._qualification_lock = threading.Lock()
    obj._limit_recovery_active = False
    obj.control_mode = ManagerEvent.NONE
    obj.active_source = None
    obj._transport_ready = True
    obj._driver_power_qualified = True
    obj._last_pos_time = 1.0
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.get_clock = lambda: clock_at(10.0)
    obj._publish_safety_status = lambda force=False: None
    obj._feedback_is_fresh = lambda: True
    obj._restore_startup_encoder_fault = lambda: (
        True, 'firmware fault state clean')
    obj._probe_all_drivers = lambda: (True, 'all probes passed')
    obj._wait_for_stable_feedback = lambda: (True, '')
    device_calls = []
    obj._call_device_sync = lambda predicate, timeout=1.5, cmd='': (
        device_calls.append(predicate) or (True, 'OK'))

    def advance(_duration):
        nonlocal sleep_count
        sleep_count += 1
        if sleep_count == 1:
            obj._last_pos_time = 2.0
        else:
            obj._last_pos = [39.89, 5.0, 5.0, 5.0, 5.0, 5.0]

    monkeypatch.setattr('control_interface_py.manager.time.sleep', advance)
    response = SimpleNamespace(success=False, message='')

    result = ControlManager.recover_catheter_linear_limit(
        obj, SimpleNamespace(), response)

    assert result.success is True
    assert 'axis 0 recovered inward' in result.message
    nonzero = [message for message in published if any(message.data)]
    assert len(nonzero) == 1
    assert list(nonzero[0].data) == [-2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert list(published[-1].data) == [0.0] * 6
    assert device_calls == [ManagerEvent.STOP_MOTOR, ManagerEvent.STOP_MOTOR]
    assert obj._driver_power_qualified is False
    assert obj._limit_recovery_active is False


def test_limit_recovery_execution_honours_external_safety_stop():
    published = []
    obj = _recovery_manager([40.022, 5.0, 5.0, 5.0, 5.0, 5.0])
    obj._limit_recovery_cancelled = True
    obj._transport_ready = True
    obj._feedback_is_fresh = lambda: True
    obj.control_pub = SimpleNamespace(publish=published.append)

    ok, reason = ControlManager._execute_limit_recovery(
        obj, 0, 'upper', 39.9, -2.0)

    assert ok is False
    assert reason == 'limit recovery cancelled by safety stop'
    assert published == []


def test_stale_source_commands_zero_velocity(monkeypatch):
    published = []
    warnings = []
    obj = object.__new__(ControlManager)
    obj.active_source = "slicer"
    obj.last_input_time = {"slicer": 9.0}
    obj.SOURCE_TIMEOUT = 0.2
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.get_clock = lambda: clock_at(10.01)
    obj.get_logger = lambda: SimpleNamespace(warn=warnings.append)
    monkeypatch.setattr(
        "control_interface_py.manager.time.monotonic", lambda: 10.0)

    obj._source_watchdog_tick()

    assert obj.active_source is None
    assert obj.control_mode == ManagerEvent.NONE
    assert len(published) == 1
    assert published[0].predicate == DeviceStream.VEL
    assert list(published[0].data) == [0.0] * 6
    assert warnings


def test_stale_position_source_dispatches_stop(monkeypatch):
    dispatched = []
    published = []
    warnings = []
    obj = object.__new__(ControlManager)
    obj.active_source = 'autonomy'
    obj.last_input_time = {'autonomy': 9.0}
    obj.SOURCE_TIMEOUT = 0.2
    obj.control_mode = ManagerEvent.JOINT_POS
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj._dispatch_device_command = dispatched.append
    obj.get_clock = lambda: clock_at(10.01)
    obj.get_logger = lambda: SimpleNamespace(warn=warnings.append)
    monkeypatch.setattr(
        'control_interface_py.manager.time.monotonic', lambda: 10.0)

    obj._source_watchdog_tick()

    assert obj.active_source is None
    assert obj.control_mode == ManagerEvent.NONE
    assert published == []
    assert len(dispatched) == 1
    assert dispatched[0].predicate == ManagerEvent.STOP_MOTOR
    assert warnings


def test_fresh_source_is_not_stopped(monkeypatch):
    published = []
    obj = object.__new__(ControlManager)
    obj.active_source = "autonomy"
    obj.last_input_time = {"autonomy": 9.9}
    obj.SOURCE_TIMEOUT = 0.2
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.control_pub = SimpleNamespace(publish=published.append)
    monkeypatch.setattr(
        "control_interface_py.manager.time.monotonic", lambda: 10.0)

    obj._source_watchdog_tick()

    assert obj.active_source == "autonomy"
    assert published == []


def test_position_speed_magnitudes_are_reliability_clamped():
    out = manager([20.0] * 6)._clamp_position_speeds(
        [-0.5, 50.0, 0.0, 3.0, 2.0, 12.0])
    assert out == [2.0, 10.0, 0.0, 3.0, 2.0, 10.0]


def test_position_control_forwards_target_and_speed_atomically(monkeypatch):
    published = []
    obj = manager([5.0] * 6)
    obj.deadman = False
    obj.estop = False
    obj.control_mode = ManagerEvent.JOINT_POS
    obj.active_source = None
    obj.last_input_time = {}
    obj._lock_blocks = lambda source: False
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.event_pub = SimpleNamespace(publish=lambda msg: None)
    obj.get_clock = lambda: clock_at(10.01)
    monkeypatch.setattr(
        'control_interface_py.manager.time.monotonic', lambda: 10.0)
    msg = ControlStream()
    msg.header.frame_id = 'autonomy'
    msg.header.stamp = Time(sec=10)
    msg.joint_pos = [-1.0, 20.0, 41.0, 4.0, 5.0, 6.0]
    msg.joint_vel = [0.5, 20.0, 1.0, 0.0, 0.0, 0.0]

    ControlManager.teleop_callback(obj, msg)

    assert len(published) == 1
    assert published[0].predicate == DeviceStream.POS
    assert list(published[0].data[:6]) == [0.0, 20.0, 40.0, 4.0, 5.0, 6.0]
    assert list(published[0].data[6:]) == [2.0, 10.0, 2.0, 0.0, 0.0, 0.0]


def test_velocity_command_requires_six_finite_values(monkeypatch):
    published = []
    warnings = []
    obj = manager([5.0] * 6)
    obj.deadman = False
    obj.estop = False
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.active_source = None
    obj.last_input_time = {}
    obj._lock_blocks = lambda source: False
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.event_pub = SimpleNamespace(publish=lambda msg: None)
    obj.get_logger = lambda: SimpleNamespace(warn=warnings.append)
    monkeypatch.setattr(
        'control_interface_py.manager.time.monotonic', lambda: 10.0)
    msg = ControlStream()
    msg.header.frame_id = 'slicer'
    msg.joint_vel = [1.0, 2.0]

    ControlManager.teleop_callback(obj, msg)

    assert published == []
    assert obj.active_source is None
    assert obj.last_input_time == {}
    assert warnings


def test_motion_is_blocked_until_pos_and_enc_are_fresh(monkeypatch):
    published = []
    statuses = []
    obj = manager([5.0] * 6)
    obj._last_enc_time = None
    obj.deadman = False
    obj.estop = False
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.active_source = None
    obj.last_input_time = {}
    obj._lock_blocks = lambda source: False
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.event_pub = SimpleNamespace(publish=statuses.append)
    obj.get_clock = lambda: clock_at(10.01)
    monkeypatch.setattr(
        'control_interface_py.manager.time.monotonic', lambda: 10.0)
    msg = ControlStream()
    msg.header.frame_id = 'slicer'
    msg.header.stamp = Time(sec=10)
    msg.joint_vel = [1.0] * 6

    ControlManager.teleop_callback(obj, msg)

    assert published == []
    assert obj.active_source is None
    assert statuses[-1].text == 'MANAGER_INHIBITED:FEEDBACK_NOT_QUALIFIED'


def test_motion_is_blocked_when_position_feedback_is_out_of_range(monkeypatch):
    published = []
    statuses = []
    obj = manager([500000.0] + [5.0] * 5)
    obj.deadman = False
    obj.estop = False
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.active_source = None
    obj.last_input_time = {}
    obj._lock_blocks = lambda source: False
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.event_pub = SimpleNamespace(publish=statuses.append)
    obj.get_clock = lambda: clock_at(10.01)
    monkeypatch.setattr(
        'control_interface_py.manager.time.monotonic', lambda: 10.0)
    msg = ControlStream()
    msg.header.frame_id = 'slicer'
    msg.header.stamp = Time(sec=10)
    msg.joint_vel = [1.0] * 6

    ControlManager.teleop_callback(obj, msg)

    assert published == []
    assert obj.active_source is None
    assert statuses[-1].text == (
        'MANAGER_INHIBITED:POSITION_FEEDBACK_OUT_OF_RANGE')


def test_subresolution_feedback_overshoot_is_plausible():
    obj = manager([-0.0001, 40.0001, 5.0, 5.0, 5.0, 5.0])

    assert obj._position_feedback_is_plausible() is True


def test_feedback_tolerance_does_not_expand_command_limits():
    obj = manager([-0.0001] + [5.0] * 5)

    assert obj._clamp_command(DeviceStream.POS, [-0.0001] * 6) == [0.0] * 6


def test_material_feedback_overshoot_remains_implausible():
    obj = manager([-0.0011] + [5.0] * 5)

    assert obj._position_feedback_is_plausible() is False
    assert 'axis 0' in obj._position_feedback_violation()


def test_expired_nonzero_command_does_not_refresh_source_or_publish():
    published = []
    warnings = []
    obj = manager([5.0] * 6)
    obj.deadman = False
    obj.estop = False
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.active_source = 'autonomy'
    obj.last_input_time = {'autonomy': 7.0}
    obj._lock_blocks = lambda source: False
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.get_clock = lambda: clock_at(10.0)
    obj.get_logger = lambda: SimpleNamespace(warn=warnings.append)
    msg = ControlStream()
    msg.header.frame_id = 'autonomy'
    msg.header.stamp = Time(sec=9)
    msg.joint_vel = [1.0] * 6

    ControlManager.teleop_callback(obj, msg)

    assert published == []
    assert obj.last_input_time == {'autonomy': 7.0}
    assert 'source_command_expired' in warnings[-1]


def test_manager_preserves_fresh_source_stamp(monkeypatch):
    published = []
    obj = manager([5.0] * 6)
    obj.deadman = False
    obj.estop = False
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.active_source = 'autonomy'
    obj.last_input_time = {}
    obj._lock_blocks = lambda source: False
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.get_clock = lambda: clock_at(10.01)
    monkeypatch.setattr(
        'control_interface_py.manager.time.monotonic', lambda: 10.0)
    msg = ControlStream()
    msg.header.frame_id = 'autonomy'
    msg.header.stamp = Time(sec=10, nanosec=1_000_000)
    msg.joint_vel = [1.0] * 6

    ControlManager.teleop_callback(obj, msg)

    assert len(published) == 1
    assert published[0].header.stamp == msg.header.stamp
    assert published[0].header.frame_id == 'autonomy'


def test_delayed_zero_velocity_preempts_even_with_old_stamp(monkeypatch):
    published = []
    obj = manager([5.0] * 6)
    obj.deadman = False
    obj.estop = False
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.active_source = 'autonomy'
    obj.last_input_time = {'autonomy': 9.9}
    obj._last_command_stamp_ns['autonomy'] = 10_000_000_000
    obj._lock_blocks = lambda source: False
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.get_clock = lambda: clock_at(10.01)
    monkeypatch.setattr(
        'control_interface_py.manager.time.monotonic', lambda: 10.0)
    msg = ControlStream()
    msg.header.frame_id = 'autonomy'
    msg.header.stamp = Time(sec=9)
    msg.joint_vel = [0.0] * 6

    ControlManager.teleop_callback(obj, msg)

    assert len(published) == 1
    assert list(published[0].data) == [0.0] * 6
    assert obj.last_input_time == {'autonomy': 9.9}


def test_feedback_watchdog_stops_active_motion_on_implausible_position(
        monkeypatch):
    published = []
    statuses = []
    errors = []
    obj = manager([500000.0] + [5.0] * 5)
    obj.active_source = 'slicer'
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.event_pub = SimpleNamespace(publish=statuses.append)
    obj.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    obj.get_logger = lambda: SimpleNamespace(error=errors.append)
    monkeypatch.setattr(
        'control_interface_py.manager.time.monotonic', lambda: 10.0)

    ControlManager._feedback_watchdog_tick(obj)

    assert obj.active_source is None
    assert obj.control_mode == ManagerEvent.NONE
    assert list(published[-1].data) == [0.0] * 6
    assert statuses[-1].text == (
        'MANAGER_INHIBITED:POSITION_FEEDBACK_OUT_OF_RANGE')
    assert errors


def test_feedback_watchdog_stops_active_motion(monkeypatch):
    published = []
    statuses = []
    errors = []
    obj = manager([5.0] * 6)
    obj._last_pos_time = 9.0
    obj._last_enc_time = 9.0
    obj.active_source = 'slicer'
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.event_pub = SimpleNamespace(publish=statuses.append)
    obj.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    obj.get_logger = lambda: SimpleNamespace(error=errors.append)
    monkeypatch.setattr(
        'control_interface_py.manager.time.monotonic', lambda: 10.0)

    ControlManager._feedback_watchdog_tick(obj)

    assert obj.active_source is None
    assert obj.control_mode == ManagerEvent.NONE
    assert list(published[-1].data) == [0.0] * 6
    assert statuses[-1].text == 'MANAGER_INHIBITED:FEEDBACK_NOT_QUALIFIED'
    assert errors


def test_confirmed_device_fault_latches_and_commands_zero():
    published = []
    events = []
    obj = manager([5.0] * 6)
    obj.active_source = 'slicer'
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.event_pub = SimpleNamespace(publish=events.append)
    obj.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    msg = DeviceEvent()
    msg.predicate = ManagerEvent.STALL
    msg.text = 'MOTION_CONFIRMED:STALL'
    msg.data = [3.0, float(ManagerEvent.MOTION_CONFIRMED)]

    ControlManager.device_event_callback(obj, msg)

    assert obj._fault_latched is True
    assert obj.active_source is None
    assert obj.control_mode == ManagerEvent.NONE
    assert list(published[-1].data) == [0.0] * 6
    assert events[-1].text == 'MANAGER_INHIBITED:MOTION_CONFIRMED:STALL'


def test_firmware_watchdog_event_latches_manager_and_commands_zero():
    published = []
    events = []
    obj = manager([5.0] * 6)
    obj.active_source = 'catheter_mppi'
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.event_pub = SimpleNamespace(publish=events.append)
    obj.get_clock = lambda: clock_at(10.0)
    msg = DeviceEvent()
    msg.predicate = ManagerEvent.FIRMWARE_STATUS
    msg.text = 'FIRMWARE_WATCHDOG_STOP:test-build'
    msg.data = [1.0, 5.0, 1500.0]

    ControlManager.device_event_callback(obj, msg)

    assert obj._fault_latched is True
    assert obj._fault_reason == 'FIRMWARE_WATCHDOG_STOP'
    assert obj.active_source is None
    assert obj.control_mode == ManagerEvent.NONE
    assert list(published[-1].data) == [0.0] * 6
    assert events[-1].text == 'MANAGER_INHIBITED:FIRMWARE_WATCHDOG_STOP'


def test_position_timeout_status_is_not_a_confirmed_motion_fault():
    published = []
    events = []
    obj = manager([5.0] * 6)
    obj.active_source = 'autonomy'
    obj.control_mode = ManagerEvent.JOINT_POS
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.event_pub = SimpleNamespace(publish=events.append)
    obj.get_clock = lambda: clock_at(10.0)
    msg = DeviceEvent()
    msg.predicate = ManagerEvent.POSITION_STATUS
    msg.text = 'POSITION_TIMED_OUT'
    msg.data = [1.0, float(ManagerEvent.POSITION_TIMED_OUT), 1.0]

    ControlManager.device_event_callback(obj, msg)

    assert obj._fault_latched is False
    assert obj.active_source == 'autonomy'
    assert obj.control_mode == ManagerEvent.JOINT_POS
    assert published == []
    assert events[-1].text == 'POSITION_TIMED_OUT'


def test_retrying_stall_is_forwarded_without_latching_or_zeroing():
    published = []
    events = []
    obj = manager([5.0] * 6)
    obj.active_source = 'autonomy'
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.event_pub = SimpleNamespace(publish=events.append)
    obj.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    msg = DeviceEvent()
    msg.predicate = ManagerEvent.STALL
    msg.text = 'MOTION_RETRYING:STALL'
    msg.data = [3.0, float(ManagerEvent.MOTION_RETRYING),
                float(ManagerEvent.FAULT_STALL)]

    ControlManager.device_event_callback(obj, msg)

    assert obj._fault_latched is False
    assert obj.active_source == 'autonomy'
    assert obj.control_mode == ManagerEvent.JOINT_VEL
    assert published == []
    assert events[-1].text == 'MOTION_RETRYING:STALL'


def test_stop_is_accepted_while_fault_is_latched():
    published = []
    dispatched = []
    obj = manager([5.0] * 6)
    obj._fault_latched = True
    obj._fault_reason = 'MOTION_CONFIRMED:STALL'
    obj.active_source = 'autonomy'
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.last_input_time = {}
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj._dispatch_device_command = dispatched.append
    obj.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    msg = ManagerEvent()
    msg.header.frame_id = 'slicer'
    msg.predicate = ManagerEvent.STOP_MOTOR

    ControlManager.teleop_event_callback(obj, msg)

    assert obj.active_source is None
    assert obj.control_mode == ManagerEvent.NONE
    assert list(published[-1].data) == [0.0] * 6
    assert dispatched == [msg]


def test_failed_device_command_latches_manager_fault():
    published = []
    events = []
    warnings = []
    obj = manager([5.0] * 6)
    obj.active_source = 'slicer'
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.event_pub = SimpleNamespace(publish=events.append)
    obj.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    obj.get_logger = lambda: SimpleNamespace(warn=warnings.append)
    response = SimpleNamespace(success=False, response='Timeout')
    future = SimpleNamespace(result=lambda: response)
    request = DeviceCmd.Request()
    request.predicate = ManagerEvent.FAULT_STATUS

    ControlManager._on_device_response(obj, future, request)

    assert obj._fault_latched is True
    assert obj.active_source is None
    assert obj.control_mode == ManagerEvent.NONE
    assert list(published[-1].data) == [0.0] * 6
    assert 'DEVICE_COMMAND_FAILED' in events[-1].text
    assert warnings


def test_transport_loss_immediately_stops_active_motion():
    published = []
    statuses = []
    errors = []
    obj = manager([5.0] * 6)
    obj.active_source = 'slicer'
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.control_pub = SimpleNamespace(publish=published.append)
    obj.event_pub = SimpleNamespace(publish=statuses.append)
    obj.safety_status_pub = SimpleNamespace(publish=lambda msg: None)
    obj.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    obj.get_logger = lambda: SimpleNamespace(error=errors.append)
    msg = SimpleNamespace(data='SERIAL_DISCONNECTED:RX_ERROR')

    ControlManager.transport_status_callback(obj, msg)

    assert obj._transport_ready is False
    assert obj.active_source is None
    assert obj.control_mode == ManagerEvent.NONE
    assert list(published[-1].data) == [0.0] * 6
    assert statuses[-1].text == 'MANAGER_INHIBITED:TRANSPORT_NOT_READY'
    assert errors


def test_driver_power_must_be_explicitly_qualified(monkeypatch):
    obj = manager([5.0] * 6)
    obj._driver_power_qualified = False
    monkeypatch.setattr(
        'control_interface_py.manager.time.monotonic', lambda: 10.0)

    assert obj._motion_inhibit_reason() == 'DRIVER_POWER_NOT_QUALIFIED'


def test_transport_reconnect_revokes_previous_power_qualification():
    obj = manager([5.0] * 6)
    obj.active_source = None
    obj.event_pub = SimpleNamespace(publish=lambda msg: None)
    obj.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    obj.get_logger = lambda: SimpleNamespace(error=lambda message: None)

    ControlManager.transport_status_callback(
        obj, SimpleNamespace(data='SERIAL_DISCONNECTED:test'))
    ControlManager.transport_status_callback(
        obj, SimpleNamespace(data='SERIAL_READY:/dev/ttyACM0'))

    assert obj._driver_power_qualified is False


def test_manager_fault_status_parser_accepts_encoder_startup_latch():
    status = parse_fault_status('V1,L=12,E=00,Q=41,F=0,7,0,0,7,0')

    assert status['latched_mask'] == 0x12
    assert status['enabled_mask'] == 0
    assert status['faults'] == [0, 7, 0, 0, 7, 0]


def test_driver_diagnostic_parser_requires_matching_axis_and_real_ack():
    result = parse_driver_diagnostic(
        'OK_DRIVER_UART,V1,A=4,S=K,F=0,N=1,R=214C31',
        expected_axis=4)

    assert result['axis'] == 4
    assert result['response'] == b'!L1'
    with pytest.raises(ValueError, match='expected 3'):
        parse_driver_diagnostic(result['raw'], expected_axis=3)
    with pytest.raises(ValueError, match='invalid acknowledgement'):
        parse_driver_diagnostic(
            'OK_DRIVER_UART,V1,A=4,S=K,F=0,N=1,R=',
            expected_axis=4)


def test_all_driver_uart_probes_are_required():
    calls = []
    obj = object.__new__(ControlManager)

    def call(predicate, timeout=1.5, cmd=''):
        calls.append((predicate, timeout, cmd))
        axis = int(cmd)
        return True, (
            f'OK_DRIVER_UART,V1,A={axis},S=K,F=0,N=1,R=214C31')

    obj._call_device_sync = call

    ok, message = ControlManager._probe_all_drivers(obj)

    assert ok is True
    assert message == 'all 6 driver UART probes passed'
    assert [call[2] for call in calls] == list('012345')


def test_driver_uart_probe_failure_identifies_axis():
    obj = object.__new__(ControlManager)
    obj._call_device_sync = lambda predicate, timeout=1.5, cmd='': (
        (False, 'ERR_DRIVER_UART,V1,A=2,S=0,F=1,N=3,R=')
        if cmd == '2' else
        (True, f'OK_DRIVER_UART,V1,A={cmd},S=K,F=0,N=1,R=214C31'))

    ok, message = ControlManager._probe_all_drivers(obj)

    assert ok is False
    assert 'axis 2' in message


def test_startup_encoder_fault_is_restored_before_feedback_validation():
    calls = []
    obj = object.__new__(ControlManager)
    obj._fault_latched = True
    obj._fault_reason = 'MOTION_CONFIRMED:ENCODER_INTEGRITY'

    def call(predicate, timeout=1.5, cmd=''):
        calls.append(predicate)
        if predicate == ManagerEvent.FAULT_STATUS:
            return True, 'V1,L=01,E=00,Q=41,F=7,0,0,0,0,0'
        if predicate == ManagerEvent.RESET_FAULT:
            return True, 'OK_ENCODER_RESTORED'
        pytest.fail(f'unexpected predicate {predicate}')

    obj._call_device_sync = call

    ok, message = ControlManager._restore_startup_encoder_fault(obj)

    assert ok is True
    assert 'axes [0]' in message
    assert calls == [ManagerEvent.FAULT_STATUS, ManagerEvent.RESET_FAULT]
    assert obj._fault_latched is False
    assert obj._fault_reason == ''


def test_startup_recovery_rejects_non_encoder_fault_without_reset():
    calls = []
    obj = object.__new__(ControlManager)

    def call(predicate, timeout=1.5, cmd=''):
        calls.append(predicate)
        return True, 'V1,L=01,E=00,Q=41,F=1,0,0,0,0,0'

    obj._call_device_sync = call

    ok, message = ControlManager._restore_startup_encoder_fault(obj)

    assert ok is False
    assert 'non-encoder faults' in message
    assert calls == [ManagerEvent.FAULT_STATUS]


def test_startup_recovery_requires_firmware_restore_confirmation():
    obj = object.__new__(ControlManager)
    replies = iter((
        (True, 'V1,L=01,E=00,Q=41,F=7,0,0,0,0,0'),
        (True, 'OK'),
    ))
    obj._call_device_sync = lambda predicate, timeout=1.5, cmd='': next(replies)

    ok, message = ControlManager._restore_startup_encoder_fault(obj)

    assert ok is False
    assert 'did not confirm retained encoder restoration' in message


def test_graceful_shutdown_is_idempotent_and_requests_stop(monkeypatch):
    published = []
    requests = []
    obj = object.__new__(ControlManager)
    obj._shutdown_started = False
    obj._driver_power_qualified = True
    obj.active_source = 'slicer'
    obj.locked_source = 'autonomy'
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj._command_zero_velocity = lambda: published.append('zero')
    obj.device_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: requests.append(request) or object())
    obj.get_logger = lambda: SimpleNamespace(
        info=lambda message: None, warn=lambda message: None)
    monkeypatch.setattr('control_interface_py.manager.rclpy.ok', lambda: True)

    first = ControlManager.initiate_shutdown(obj)
    second = ControlManager.initiate_shutdown(obj)

    assert first is not None
    assert second is None
    assert published == ['zero']
    assert len(requests) == 1
    assert requests[0].predicate == ManagerEvent.STOP_MOTOR
    assert obj.control_mode == ManagerEvent.NONE
    assert obj.active_source is None
    assert obj.locked_source is None
    assert obj._driver_power_qualified is False


def test_shutdown_with_invalid_ros_context_defers_to_serial_stop(monkeypatch):
    published = []
    obj = object.__new__(ControlManager)
    obj._shutdown_started = False
    obj._driver_power_qualified = True
    obj.active_source = 'slicer'
    obj.locked_source = None
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj._command_zero_velocity = lambda: published.append('zero')
    obj.device_client = SimpleNamespace(
        service_is_ready=lambda: pytest.fail(
            'invalid ROS context must not touch the service client'))
    monkeypatch.setattr('control_interface_py.manager.rclpy.ok', lambda: False)

    future = ControlManager.initiate_shutdown(obj)

    assert future is None
    assert published == []
    assert obj.control_mode == ManagerEvent.NONE
    assert obj._driver_power_qualified is False


def _command_test_manager(monkeypatch):
    obj = manager([5.0] * 6)
    obj.active_source = None
    obj.control_mode = ManagerEvent.NONE
    obj.last_input_time = {}
    obj.event_pub = SimpleNamespace(publish=lambda msg: None)
    obj.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    obj.get_logger = lambda: SimpleNamespace(
        warn=lambda message: None, info=lambda message: None)
    monkeypatch.setattr(
        'control_interface_py.manager.time.monotonic', lambda: 10.0)
    return obj


def test_legacy_start_motor_is_rejected_without_device_dispatch(monkeypatch):
    events = []
    obj = _command_test_manager(monkeypatch)
    obj.event_pub = SimpleNamespace(publish=events.append)
    obj._dispatch_device_command = lambda msg: pytest.fail(
        'unsupported START_MOTOR must not reach firmware')
    msg = ManagerEvent()
    msg.header.frame_id = 'slicer'
    msg.predicate = ManagerEvent.START_MOTOR

    ControlManager.teleop_event_callback(obj, msg)

    assert events[-1].text == 'COMMAND_REJECTED:START_MOTOR_UNSUPPORTED'


def test_debug_mode_is_disabled_in_production(monkeypatch):
    events = []
    obj = _command_test_manager(monkeypatch)
    obj.event_pub = SimpleNamespace(publish=events.append)
    msg = ManagerEvent()
    msg.header.frame_id = 'slicer'
    msg.predicate = ManagerEvent.MODE
    msg.text = chr(ManagerEvent.DEBUG)

    ControlManager.teleop_event_callback(obj, msg)

    assert events[-1].text == 'COMMAND_REJECTED:DEBUG_DISABLED'
    assert obj.control_mode == ManagerEvent.NONE


def test_set_zero_is_unconditionally_forbidden(monkeypatch):
    events = []
    obj = _command_test_manager(monkeypatch)
    obj.active_source = 'slicer'
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.event_pub = SimpleNamespace(publish=events.append)
    obj._dispatch_device_command = lambda msg: pytest.fail(
        'active-motion SET_ZERO must not reach firmware')
    msg = ManagerEvent()
    msg.header.frame_id = 'slicer'
    msg.predicate = ManagerEvent.SET_ZERO

    ControlManager.teleop_event_callback(obj, msg)

    assert events[-1].text == 'COMMAND_REJECTED:SET_ZERO_FORBIDDEN'


def test_connection_request_stops_motion_and_revokes_qualification(monkeypatch):
    controls = []
    dispatched = []
    obj = _command_test_manager(monkeypatch)
    obj.active_source = 'slicer'
    obj.control_mode = ManagerEvent.JOINT_VEL
    obj.control_pub = SimpleNamespace(publish=controls.append)
    obj._dispatch_device_command = dispatched.append
    msg = ManagerEvent()
    msg.header.frame_id = 'slicer'
    msg.predicate = ManagerEvent.CONNECTION
    msg.text = 'disconnect'

    ControlManager.teleop_event_callback(obj, msg)

    assert list(controls[-1].data) == [0.0] * 6
    assert dispatched == [msg]
    assert obj.control_mode == ManagerEvent.NONE
    assert obj._driver_power_qualified is False
