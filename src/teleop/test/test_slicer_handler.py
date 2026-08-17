import math
from types import SimpleNamespace

import pytest

from control_interface.msg import ManagerEvent
from teleop.slicer import QUALIFY_DRIVER_POWER, SlicerHandler


def test_point_vector_requires_two_finite_points():
    short = SimpleNamespace(pointdata=[SimpleNamespace(x=0.0, y=0.0, z=0.0)])
    with pytest.raises(ValueError, match='at least two'):
        SlicerHandler._point_vector(short)

    invalid = SimpleNamespace(pointdata=[
        SimpleNamespace(x=0.0, y=0.0, z=0.0),
        SimpleNamespace(x=0.0, y=math.nan, z=0.0),
    ])
    with pytest.raises(ValueError, match='non-finite'):
        SlicerHandler._point_vector(invalid)


def test_malformed_igtl_name_is_ignored_not_raised():
    warnings = []
    fake = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(warn=warnings.append))

    assert SlicerHandler._parse_igtl_name(fake, 'missing_separator') is None
    assert warnings


def test_stale_key_watchdog_publishes_zero_only_once(monkeypatch):
    zero_calls = []
    warnings = []
    fake = SimpleNamespace(
        key_motion_active=True,
        last_key_time=1.0,
        input_timeout_s=0.25,
        _publish_zero_velocity=lambda: zero_calls.append(True),
        get_logger=lambda: SimpleNamespace(warn=warnings.append),
    )
    monkeypatch.setattr('teleop.slicer.time.monotonic', lambda: 2.0)

    SlicerHandler.input_watchdog_callback(fake)
    SlicerHandler.input_watchdog_callback(fake)

    assert zero_calls == [True]
    assert fake.key_motion_active is False
    assert warnings


def test_volatile_manager_safety_mirror_is_not_forwarded_twice():
    forwarded = []
    fake = SimpleNamespace(_forward_manager_event=forwarded.append)
    safety = ManagerEvent()
    safety.predicate = ManagerEvent.FAULT_STATUS
    safety.text = 'MANAGER_READY'

    SlicerHandler.manager_event_callback(fake, safety)

    assert forwarded == []


def test_safety_status_is_cached_and_republished_for_late_bridge():
    published = []
    fake = SimpleNamespace(
        latest_safety_payload=None,
        manager_event=SimpleNamespace(data=''),
        manager_event_pub=SimpleNamespace(publish=published.append),
    )
    fake._event_payload = lambda msg: SlicerHandler._event_payload(msg)
    fake._forward_manager_event = (
        lambda msg: SlicerHandler._forward_manager_event(fake, msg))
    safety = ManagerEvent()
    safety.predicate = ManagerEvent.FAULT_STATUS
    safety.text = 'MANAGER_READY'

    SlicerHandler.manager_safety_callback(fake, safety)
    SlicerHandler.safety_heartbeat_callback(fake)

    assert fake.latest_safety_payload == 'QMANAGER_READY'
    assert len(published) == 2
    assert published[-1].data == 'QMANAGER_READY'


def test_qualification_result_is_forwarded_to_operator():
    published = []
    fake = SimpleNamespace(
        manager_event_pub=SimpleNamespace(publish=published.append))
    fake._publish_operator_event = (
        lambda predicate, subject:
        SlicerHandler._publish_operator_event(fake, predicate, subject))
    future = SimpleNamespace(result=lambda: SimpleNamespace(
        success=True, message='qualified'))

    SlicerHandler._on_driver_power_qualification(fake, future)

    assert published[-1].name == 'robot/event'
    assert published[-1].data == chr(QUALIFY_DRIVER_POWER) + 'OK:qualified'
