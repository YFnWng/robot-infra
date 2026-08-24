from datetime import datetime
from types import SimpleNamespace

from control_interface.msg import ManagerEvent
from teleop.bag_recorder import (
    COIL_TRANSFORM_PATTERN, TOPIC_TYPES, TeleopBagRecorder,
    allocate_session_dir)


def test_allocate_session_dir_adds_suffix_on_collision(tmp_path):
    now = datetime(2026, 8, 23, 12, 34, 56)
    first = allocate_session_dir(tmp_path, 'teleop', now)
    second = allocate_session_dir(tmp_path, 'teleop', now)

    assert first.name == 'teleop_20260823_123456'
    assert second.name == 'teleop_20260823_123456_01'
    assert first.is_dir()
    assert second.is_dir()


def test_only_slicer_mode_events_control_recording():
    def event(frame_id, predicate, text):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=frame_id),
            predicate=predicate,
            text=text,
        )

    assert TeleopBagRecorder._slicer_mode(event(
        'slicer', ManagerEvent.MODE, chr(ManagerEvent.JOINT_VEL))) == '3'
    assert TeleopBagRecorder._slicer_mode(event(
        'autonomy', ManagerEvent.MODE, chr(ManagerEvent.JOINT_VEL))) is None
    assert TeleopBagRecorder._slicer_mode(event(
        'slicer', ManagerEvent.STOP_MOTOR, '')) is None


def test_enable_and_disable_dispatch_to_recorder_methods():
    calls = []
    fake = SimpleNamespace(
        _slicer_mode=lambda msg: msg.text,
        start_recording=lambda: calls.append('start'),
        record_message=lambda topic, msg: calls.append(('write', topic)),
        stop_recording=lambda reason: calls.append(('stop', reason)),
    )

    TeleopBagRecorder.event_callback(fake, SimpleNamespace(text='3'))
    TeleopBagRecorder.event_callback(fake, SimpleNamespace(text='0'))

    assert calls == [
        'start',
        ('write', '/teleop/event'),
        ('write', '/teleop/event'),
        ('stop', 'keyboard disabled'),
    ]


def test_message_timestamp_prefers_nonzero_header_stamp():
    msg = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=123, nanosec=456)))

    assert TeleopBagRecorder._message_timestamp_ns(msg, 7) == 123000000456


def test_message_timestamp_falls_back_for_zero_or_missing_stamp():
    zero = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=0, nanosec=0)))

    assert TeleopBagRecorder._message_timestamp_ns(zero, 7) == 7
    assert TeleopBagRecorder._message_timestamp_ns(object(), 8) == 8


def test_raw_sensor_transforms_are_recorded_but_estimated_shape_is_not():
    assert '/IGTL_TRANSFORM_IN' in TOPIC_TYPES
    assert '/IGTL_POINT_IN' not in TOPIC_TYPES
    assert '/IGTL_POINT_OUT' not in TOPIC_TYPES


def test_only_exact_raw_and_filtered_rx_transforms_are_recorded():
    calls = []
    fake = SimpleNamespace(
        record_message=lambda topic, msg: calls.append((topic, msg.name)))

    for name in ('robot/joint_pos_target', 'RX1', 'RX1_filtered',
                 'RX10', 'RX10_filtered', 'RX0', 'RX1 filtered',
                 'estimator/shape'):
        TeleopBagRecorder.coil_transform_callback(
            fake, SimpleNamespace(name=name))

    assert calls == [
        ('/IGTL_TRANSFORM_IN', 'RX1'),
        ('/IGTL_TRANSFORM_IN', 'RX1_filtered'),
        ('/IGTL_TRANSFORM_IN', 'RX10'),
        ('/IGTL_TRANSFORM_IN', 'RX10_filtered'),
    ]
    assert COIL_TRANSFORM_PATTERN.fullmatch('RX2_filtered')
