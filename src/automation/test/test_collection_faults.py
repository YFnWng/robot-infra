from types import SimpleNamespace

import numpy as np
import pytest

from automation.collection.node import CollectionNode, parse_fault_status
from control_interface.msg import DeviceEvent, ManagerEvent


def preflight_feedback(**overrides):
    now_ns = 2_000_000_000
    pos = [0.0] * 6
    enc = [0.0] * 6
    values = dict(
        _last_pos=pos,
        _last_enc={"data": enc, "stamp_ns": now_ns},
        _pos_history=[(1_400_000_000, pos), (now_ns, pos)],
        _enc_history=[(1_400_000_000, enc), (now_ns, enc)],
        _preflight_require_enc=True,
        _preflight_max_age_s=0.25,
        _preflight_stability_s=0.5,
        _preflight_limit_tolerance=0.1,
        _preflight_position_drift=np.array([0.1, 1.0, 0.1, 0.1, 1.0, 1.0]),
        _preflight_encoder_drift=100.0,
        _pos_lower6=np.array([0.0, -180.0, 0.0, 0.0, -180.0, -360.0]),
        _pos_upper6=np.array([40.0, 180.0, 10.0, 80.0, 180.0, 360.0]),
        _target_idx=[0, 1, 2],
    )
    values.update(overrides)
    fake = SimpleNamespace(**values)
    return CollectionNode._feedback_preflight_error(fake, now_ns)


def test_feedback_preflight_accepts_fresh_stable_in_range_state():
    assert preflight_feedback() is None


def test_feedback_preflight_rejects_missing_and_stale_frames():
    assert "missing POS" in preflight_feedback(_last_pos=None)
    assert "missing ENC" in preflight_feedback(_last_enc=None)
    assert "stale POS" in preflight_feedback(
        _pos_history=[(1_000_000_000, [0.0] * 6)])


def test_feedback_preflight_rejects_out_of_range_target_position():
    pos = [-1297.0, 69577.0, 0.0, 0.0, 0.0, 0.0]
    assert "catheter_lin position" in preflight_feedback(
        _last_pos=pos,
        _pos_history=[(1_400_000_000, pos), (2_000_000_000, pos)])


def test_feedback_preflight_rejects_position_drift():
    moving = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert "catheter_lin moved" in preflight_feedback(
        _last_pos=moving,
        _pos_history=[(1_400_000_000, [0.0] * 6),
                      (2_000_000_000, moving)])


def test_feedback_preflight_rejects_encoder_drift():
    moving = [101.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert "catheter_lin encoder moved" in preflight_feedback(
        _last_enc={"data": moving, "stamp_ns": 2_000_000_000},
        _enc_history=[(1_400_000_000, [0.0] * 6),
                      (2_000_000_000, moving)])


def test_feedback_preflight_ignores_motion_before_latest_stability_window():
    old_motion = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    zero = [0.0] * 6
    assert preflight_feedback(
        _pos_history=[
            (1_000_000_000, zero),
            (1_400_000_000, old_motion),
            (1_500_000_000, zero),
            (2_000_000_000, zero)],
        _enc_history=[
            (1_000_000_000, zero),
            (1_400_000_000, [500.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            (1_500_000_000, zero),
            (2_000_000_000, zero)]) is None


def test_collection_velocity_bounds_preserve_zero_and_lift_nonzero_speed():
    fake = SimpleNamespace(
        _min_speeds=np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        _max_speeds=np.array([10.0, 40.0, 1.0, 4.0, 25.0, 25.0]),
    )
    out = CollectionNode._apply_velocity_bounds(
        fake, np.array([0.5, 0.0, -2.0, 0.0, 0.0, 0.0]))
    assert out.tolist() == [2.0, 0.0, -1.0, 0.0, 0.0, 0.0]


def test_position_return_target_can_select_encoder_zero():
    fake = SimpleNamespace(
        _last_pos=[5.0, 20.0, 2.0, 4.0, 5.0, 6.0],
        _start_pos=[1.0, 2.0, 3.0, 7.0, 8.0, 9.0],
        _return_to_zero=True,
        _target_idx=[0, 1, 2],
        _pos_lower6=np.array([0.0, -180.0, 0.0, 0.0, -180.0, -360.0]),
        _pos_upper6=np.array([40.0, 180.0, 10.0, 80.0, 180.0, 360.0]),
    )
    target = CollectionNode._position_return_target(fake)
    assert target.tolist() == [0.0, 0.0, 0.0, 4.0, 5.0, 6.0]


def test_position_return_requires_stable_per_joint_tolerance():
    fake = SimpleNamespace(
        _last_pos=[0.05, 0.2, 0.02, 0.0, 0.0, 0.0],
        _return_target_pos=np.zeros(6),
        _target_idx=[0, 1, 2],
        _return_tolerances=np.array([0.1, 0.5, 0.05, 0.1, 0.5, 0.5]),
        _return_position_settle_s=0.2,
        _return_within_since_ns=None,
    )
    fake._return_error = lambda joint: CollectionNode._return_error(fake, joint)
    assert not CollectionNode._position_return_done(fake, 1_000_000_000)
    assert not CollectionNode._position_return_done(fake, 1_199_999_999)
    assert CollectionNode._position_return_done(fake, 1_200_000_000)
    fake._last_pos[2] = 0.06
    assert not CollectionNode._position_return_done(fake, 1_300_000_000)
    assert fake._return_within_since_ns is None


def test_position_status_event_is_not_treated_as_motor_fault():
    errors = []
    fake = SimpleNamespace(
        _position_status=None,
        _position_complete_seen=False,
        get_logger=lambda: SimpleNamespace(error=errors.append),
    )
    msg = DeviceEvent()
    msg.predicate = ManagerEvent.POSITION_STATUS
    msg.text = 'POSITION_COMPLETE'
    msg.data = [1.0, float(ManagerEvent.POSITION_COMPLETE), 7.0] + [0.0] * 6
    CollectionNode._device_event_cb(fake, msg)
    assert fake._position_complete_seen
    assert errors == []


def test_position_return_stops_velocity_before_mode_switch():
    events = []
    clock_value = SimpleNamespace(nanoseconds=1_000_000_000)
    fake = SimpleNamespace(
        _last_pos=[5.0, 20.0, 2.0, 0.0, 0.0, 0.0],
        _start_pos=[0.0] * 6,
        _target_idx=[0, 1, 2],
        _return_to_zero=True,
        _return_control_mode='position',
        _return_position_speed_factor=0.5,
        _return_position_mode_delay_s=0.1,
        _min_speeds=np.array([2.0, 7.0, 1.0, 0.0, 0.0, 0.0]),
        _max_speeds=np.array([10.0, 40.0, 1.0, 4.0, 25.0, 25.0]),
        _pos_lower6=np.array([0.0, -180.0, 0.0, 0.0, -180.0, -360.0]),
        _pos_upper6=np.array([40.0, 180.0, 10.0, 80.0, 180.0, 360.0]),
        _publish_velocity=lambda velocity: None,
        _send_event=lambda predicate, text='': events.append((predicate, text)),
        _marker=lambda *args, **kwargs: None,
        get_parameter=lambda name: SimpleNamespace(value=True),
        get_clock=lambda: SimpleNamespace(now=lambda: clock_value),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None, warn=lambda message: None),
    )
    fake._position_return_target = lambda: CollectionNode._position_return_target(fake)

    CollectionNode._begin_return(fake)

    assert [predicate for predicate, _ in events] == [
        ManagerEvent.STOP_MOTOR, ManagerEvent.MODE]
    assert events[1][1] == chr(ManagerEvent.JOINT_POS)
    assert fake._return_target_pos[:3].tolist() == [0.0, 0.0, 0.0]
    assert fake._return_position_speeds[:3].tolist() == [5.0, 20.0, 1.0]
    assert fake._position_mode_ready_ns == 1_100_000_000


def floor_tracker():
    return SimpleNamespace(
        _target_idx=[0, 1, 2],
        _min_speeds=np.array([2.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
        _max_speeds=np.array([10.0, 40.0, 1.0, 4.0, 25.0, 25.0]),
        _floor_tracking_kp=2.0,
        _floor_tracking_enter_time_s=0.10,
        _floor_tracking_exit_time_s=0.05,
        _floor_tracking_direction=np.zeros(6, dtype=np.int8),
    )


def track(fake, feedforward, reference, measured):
    return CollectionNode._floor_aware_velocity(
        fake, np.asarray(feedforward, dtype=float),
        np.asarray(reference, dtype=float), np.asarray(measured, dtype=float))


def test_floor_tracker_uses_hysteresis_instead_of_integrating_small_velocity():
    fake = floor_tracker()
    zero = np.zeros(6)

    # Joint 0 enters at 2 mm/s * 0.10 s = 0.20 mm error.
    assert track(fake, zero, [0.19, 0, 0, 0, 0, 0], zero)[0] == 0.0
    assert track(fake, zero, [0.21, 0, 0, 0, 0, 0], zero)[0] == 2.0
    # It remains on until error drops below the 0.10 mm exit threshold.
    assert track(fake, zero, [0.15, 0, 0, 0, 0, 0], zero)[0] == 2.0
    assert track(fake, zero, [0.09, 0, 0, 0, 0, 0], zero)[0] == 0.0
    assert track(fake, zero, [-0.21, 0, 0, 0, 0, 0], zero)[0] == -2.0


def test_floor_tracker_inserts_stop_before_direction_reversal():
    fake = floor_tracker()
    zero = np.zeros(6)
    assert track(fake, zero, [0.21, 0, 0, 0, 0, 0], zero)[0] == 2.0
    assert track(fake, zero, [-0.21, 0, 0, 0, 0, 0], zero)[0] == 0.0
    assert track(fake, zero, [-0.21, 0, 0, 0, 0, 0], zero)[0] == -2.0


def test_floor_tracker_preserves_reliable_and_unfloored_commands():
    fake = floor_tracker()
    out = track(
        fake,
        [3.0, 0.25, 0.3, 0, 0, 0],
        [0.0, 100.0, 0.0, 0, 0, 0],
        np.zeros(6))
    assert out[0] == 3.0
    assert out[1] == 0.25
    assert out[2] == 0.0


def test_floor_tracker_follows_slow_joint2_reference_without_runaway():
    fake = floor_tracker()
    dt = 0.01
    measured = np.zeros(6)
    positions = []
    for step in range(1601):
        t = step * dt
        if t <= 8.0:
            target = 0.25 * t
            feedforward = 0.25
        else:
            target = 2.0 - 0.25 * (t - 8.0)
            feedforward = -0.25
        reference = np.zeros(6)
        reference[2] = target
        velocity = np.zeros(6)
        velocity[2] = feedforward
        command = track(fake, velocity, reference, measured)
        assert command[2] in (-1.0, 0.0, 1.0)
        measured += command * dt
        positions.append(measured[2])

    assert max(positions) < 2.15
    assert abs(measured[2]) < 0.15


@pytest.mark.parametrize(
    "kp,enter,exit_",
    [(-1.0, 0.1, 0.05), (1.0, 0.0, 0.0), (1.0, 0.1, 0.1)],
)
def test_floor_tracker_rejects_invalid_parameters(kp, enter, exit_):
    fake = SimpleNamespace(
        _floor_tracking_kp=kp,
        _floor_tracking_enter_time_s=enter,
        _floor_tracking_exit_time_s=exit_)
    with pytest.raises(ValueError):
        CollectionNode._validate_floor_tracking_parameters(fake)


def test_parse_fault_status():
    status = parse_fault_status("V1,L=05,E=00,Q=17,F=2,0,2,0,0,0")
    assert status["latched_mask"] == 0x05
    assert status["enabled_mask"] == 0
    assert status["sequence"] == 17
    assert status["faults"] == [2, 0, 2, 0, 0, 0]


@pytest.mark.parametrize("response", ["", "V2,L=00", "V1,L=00,E=00,Q=1"])
def test_parse_fault_status_rejects_malformed(response):
    with pytest.raises((KeyError, ValueError)):
        parse_fault_status(response)


def test_confirmed_fault_finishes_active_run():
    finished = []
    fake = SimpleNamespace(
        _t0=object(), _done=False, _returning=True,
        _return_status="in_progress", _hardware_fault=None,
        _run_status="running",
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=123)),
        get_logger=lambda: SimpleNamespace(error=lambda message: None),
        _finish=lambda: finished.append(True),
    )
    msg = DeviceEvent()
    msg.predicate = ManagerEvent.STALL
    msg.text = "MOTION_CONFIRMED:DRIVER_COMMUNICATION"
    msg.data = [float(value) for value in [
        2, ManagerEvent.MOTION_CONFIRMED,
        ManagerEvent.FAULT_DRIVER_COMMUNICATION,
        1, -1, 9, 1.0, 0.0, 0.0, 2, 0, ord("O")]]

    CollectionNode._device_event_cb(fake, msg)

    assert finished == [True]
    assert fake._run_status == "hardware_fault"
    assert fake._return_status == "aborted_hardware_fault"
    assert fake._hardware_fault["driver_stage"] == "O"


def test_run_end_records_final_fault_status_separately_from_preflight():
    markers = []
    preflight = {"latched_mask": 0, "enabled_mask": 0}
    fake = SimpleNamespace(
        _finish_return_result={"error_target_minus_final": None},
        _run_status="hardware_fault",
        _hardware_fault={"axis": 0},
        _fault_status=preflight,
        _enc_seen=True,
        _shutdown_on_done=True,
        should_exit=False,
        _marker=lambda event, **fields: markers.append((event, fields)),
        get_logger=lambda: SimpleNamespace(
            error=lambda message: None, info=lambda message: None),
    )
    response = SimpleNamespace(
        success=True, response="V1,L=05,E=00,Q=2,F=1,0,1,0,0,0")
    future = SimpleNamespace(result=lambda: response)

    CollectionNode._on_final_fault_status(fake, future)

    assert fake.should_exit
    assert markers[0][0] == "run_end"
    fields = markers[0][1]
    assert fields["preflight_fault_status"] is preflight
    assert fields["fault_status"]["latched_mask"] == 0x05
    assert fields["fault_status"]["faults"] == [1, 0, 1, 0, 0, 0]
