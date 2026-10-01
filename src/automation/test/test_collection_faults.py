from types import SimpleNamespace

import numpy as np
import pytest

from automation.collection.node import (
    CollectionNode, collection_marker_qos, parse_fault_status)
from automation.collection.session_check import _critical_suffixes
from control_interface.msg import DeviceEvent, ManagerEvent
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy


def test_collection_marker_qos_retains_run_start_for_rosbag_discovery():
    qos = collection_marker_qos()
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 128
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_identification_episode_markers_survive_skipped_timer_boundaries():
    episodes = [
        SimpleNamespace(name="first", start_s=0.0, duration_s=1.0,
                        maximum_command_speed_limits=np.zeros(3)),
        SimpleNamespace(name="second", start_s=1.0, duration_s=2.0,
                        maximum_command_speed_limits=np.ones(3)),
        SimpleNamespace(name="third", start_s=3.0, duration_s=1.0,
                        maximum_command_speed_limits=np.zeros(3)),
    ]
    generator = SimpleNamespace(
        episodes=episodes,
        episode_index=lambda t: 0 if t < 1.0 else (1 if t < 3.0 else 2),
    )
    markers = []
    fake = SimpleNamespace(
        _mode="identification", _gen=generator, _episode_index=-1,
        _marker=lambda event, **fields: markers.append((event, fields)),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )

    CollectionNode._update_episode_markers(fake, 0.0)
    CollectionNode._update_episode_markers(fake, 3.5)  # skips two boundaries
    CollectionNode._update_episode_markers(fake, 4.0, finishing=True)
    CollectionNode._update_episode_markers(fake, 4.0, finishing=True)

    assert [(event, fields["name"]) for event, fields in markers] == [
        ("episode_start", "first"), ("episode_end", "first"),
        ("episode_start", "second"), ("episode_end", "second"),
        ("episode_start", "third"), ("episode_end", "third"),
    ]


def test_identification_floor_correction_respects_episode_ceiling():
    fake = SimpleNamespace(
        _mode="identification",
        _gen=SimpleNamespace(
            command_speed_limits=lambda t: np.array([0.0, 12.0, 0.0])),
        _target_idx=[0, 1, 2],
        _floor_tracking_direction=np.array([1, 1, -1, 0, 0, 0], dtype=np.int8),
    )
    result = CollectionNode._apply_identification_speed_ceiling(
        fake, 19.31, np.array([2.0, 14.65, -1.0, 0.0, 0.0, 0.0]))
    assert np.allclose(result, [0.0, 12.0, 0.0, 0.0, 0.0, 0.0])
    assert fake._floor_tracking_direction[:3].tolist() == [0, 1, 0]


def test_identification_dwell_forces_exact_zero_command():
    fake = SimpleNamespace(
        _mode="identification",
        _gen=SimpleNamespace(command_speed_limits=lambda t: np.zeros(3)),
        _target_idx=[0, 1, 2],
        _floor_tracking_direction=np.array([1, -1, 1, 0, 0, 0], dtype=np.int8),
    )
    result = CollectionNode._apply_identification_speed_ceiling(
        fake, 5.0, np.array([-2.0, 7.0, 1.0, 0.0, 0.0, 0.0]))
    assert np.allclose(result, 0.0)
    assert np.all(fake._floor_tracking_direction == 0)


class TimingContract:
    @staticmethod
    def logical_to_motor_axis_velocity(logical):
        motor = np.asarray(logical, dtype=float).copy()
        motor[0] -= motor[2]
        return motor

    @staticmethod
    def project_logical_velocity(logical, _position):
        return np.asarray(logical, dtype=float).copy()


def test_phase_3_bypasses_logical_speed_floor():
    episode = SimpleNamespace(is_timing_episode=True)
    fake = SimpleNamespace(
        _mode="causal",
        _gen=SimpleNamespace(
            active_episode=lambda _t: episode,
            command_speed_limits=lambda _t: np.array([4.0, 0.0, 2.0])),
        _target_idx=[0, 1, 2],
        _floor_tracking_direction=np.zeros(6, dtype=np.int8),
        _velocity=lambda _t: np.array([4.0, 0.0, 2.0, 0.0, 0.0, 0.0]),
        _apply_identification_speed_ceiling=lambda t, value: (
            CollectionNode._apply_identification_speed_ceiling(
                fake, t, value)),
        _apply_velocity_bounds=lambda _value: pytest.fail(
            "logical speed floor must not run for Phase 3"),
    )
    result = CollectionNode._trajectory_velocity(fake, 1.0)
    assert np.allclose(result, [4.0, 0.0, 2.0, 0.0, 0.0, 0.0])
    assert np.allclose([result[0] - result[2], result[2]], [2.0, 2.0])


def test_phase_3_command_validation_rejects_projection_changes():
    expected = np.array([2.0, 2.0])
    generator = SimpleNamespace(timing_raw_velocity=lambda _t: expected)
    command = np.array([4.0, 0.0, 2.0, 0.0, 0.0, 0.0])
    fake = SimpleNamespace(
        _gen=generator, _causal_contract=TimingContract(),
        _last_pos=np.zeros(6))
    assert CollectionNode._timing_command_error(fake, 0.0, command) is None

    class DistortingContract(TimingContract):
        @staticmethod
        def project_logical_velocity(logical, _position):
            projected = np.asarray(logical, dtype=float).copy()
            projected[0] = projected[2]
            return projected

    fake._causal_contract = DistortingContract()
    assert "manager projection would change" in (
        CollectionNode._timing_command_error(fake, 0.0, command))


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


def causal_estimator_gate(**status_overrides):
    parameters = {
        "causal_require_estimator": True,
        "causal_require_estimator_tracking": True,
        "causal_estimator_max_age_s": 0.5,
        "causal_require_controller_disarmed": True,
        "causal_require_adaptation_disabled": True,
        "causal_minimum_accepted_observations": 8,
        "causal_maximum_marker_rejections": 3,
    }
    status = {
        "armed": False,
        "adaptation_enabled": False,
        "estimator_health": "TRACKING",
        "accepted_observations": 20,
        "consecutive_rejections": 0,
        "marker_diagnostic": "TRACKING",
    }
    status.update(status_overrides)
    return SimpleNamespace(
        _causal_estimator_status=status,
        _causal_estimator_status_receipt_ns=1_900_000_000,
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
    )


def test_causal_estimator_gate_requires_disarmed_frozen_tracking_ukf():
    clean = causal_estimator_gate()
    assert CollectionNode._causal_estimator_error(
        clean, 2_000_000_000, require_clean=True) is None
    armed = causal_estimator_gate(armed=True)
    assert "armed" in CollectionNode._causal_estimator_error(
        armed, 2_000_000_000, require_clean=True)
    adapting = causal_estimator_gate(adaptation_enabled=True)
    assert "adaptation" in CollectionNode._causal_estimator_error(
        adapting, 2_000_000_000, require_clean=True)


def test_causal_runtime_gate_allows_one_transient_rejection_only():
    transient = causal_estimator_gate(
        estimator_health="DEGRADED", consecutive_rejections=1,
        marker_diagnostic="CROSS_RIG_DISAGREEMENT")
    assert CollectionNode._causal_estimator_error(
        transient, 2_000_000_000, require_clean=False) is None
    repeated = causal_estimator_gate(
        estimator_health="DEGRADED", consecutive_rejections=3)
    assert "repeated marker" in CollectionNode._causal_estimator_error(
        repeated, 2_000_000_000, require_clean=False)


def test_causal_offline_shape_mode_keeps_controller_interlocks_only():
    offline = causal_estimator_gate(
        estimator_health="UNINITIALIZED", accepted_observations=0,
        consecutive_rejections=99, marker_diagnostic="MARKER_OUTLIER")
    offline.get_parameter = lambda name: SimpleNamespace(value={
        "causal_require_estimator": True,
        "causal_require_estimator_tracking": False,
        "causal_estimator_max_age_s": 0.5,
        "causal_require_controller_disarmed": True,
        "causal_require_adaptation_disabled": True,
        "causal_minimum_accepted_observations": 8,
        "causal_maximum_marker_rejections": 3,
    }[name])
    assert CollectionNode._causal_estimator_error(
        offline, 2_000_000_000, require_clean=True) is None

    offline._causal_estimator_status["armed"] = True
    assert "armed" in CollectionNode._causal_estimator_error(
        offline, 2_000_000_000, require_clean=True)


def test_offline_shape_session_does_not_require_accepted_online_estimates():
    suffixes = _critical_suffixes({
        "parameters": {"causal_require_estimator_tracking": False}})
    assert "/shape_tracking/marker_status" in suffixes
    assert "/catheter_mppi/status" in suffixes
    assert "/shape_tracking/markers" not in suffixes
    assert "/catheter_mppi/estimator_trace" not in suffixes


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


def test_causal_initialization_uses_guarded_position_transaction():
    events = []
    markers = []
    clock_value = SimpleNamespace(nanoseconds=1_000_000_000)
    timer = SimpleNamespace(cancel=lambda: None)
    fake = SimpleNamespace(
        _start_motor=True,
        _last_pos=[10.0, 0.0, 4.5, 0.0, 0.0, 0.0],
        _target_idx=[0, 1, 2],
        _causal_initial_position=np.array([20.0, 0.0, 0.0]),
        _pos_lower6=np.array([0.0, -180.0, 0.0, 0.0, -180.0, -360.0]),
        _pos_upper6=np.array([40.0, 180.0, 15.0, 80.0, 180.0, 360.0]),
        _min_speeds=np.array([2.0, 7.0, 1.0, 0.0, 0.0, 0.0]),
        _max_speeds=np.array([10.0, 40.0, 4.0, 4.0, 25.0, 25.0]),
        _return_position_speed_factor=0.5,
        _return_position_mode_delay_s=0.1,
        _return_tolerances=np.array([0.1, 0.5, 0.05, 0.1, 0.5, 0.5]),
        _rate=100.0,
        _position_status=None,
        _position_complete_seen=False,
        _initialization_timer=None,
        _send_event=lambda predicate, text='': events.append((predicate, text)),
        _marker=lambda event, **fields: markers.append((event, fields)),
        get_clock=lambda: SimpleNamespace(now=lambda: clock_value),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None, warn=lambda message: None),
        create_timer=lambda _period, _callback: timer,
        _causal_initialization_tick=lambda: None,
    )

    CollectionNode._begin_causal_initialization(fake)

    assert fake._initializing
    assert fake._initialization_target_pos.tolist() == [
        20.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert fake._initialization_position_speeds[:3].tolist() == [
        5.0, 20.0, 2.0]
    assert events == [(ManagerEvent.MODE, chr(ManagerEvent.JOINT_POS))]
    assert markers[0][0] == "initialization_start"
    assert markers[0][1]["start_position"] == [10.0, 0.0, 4.5]
    assert markers[0][1]["target_position"] == [20.0, 0.0, 0.0]


def test_causal_initialization_requires_actuating_run():
    failures = []
    fake = SimpleNamespace(
        _start_motor=False,
        _abort_preflight=lambda reason, status=None: failures.append(reason),
    )
    CollectionNode._begin_causal_initialization(fake)
    assert failures == ["causal initialization requires start_motor:=true"]


def test_causal_initialization_requalifies_before_run_start():
    class FakeTime:
        def __init__(self, nanoseconds):
            self.nanoseconds = nanoseconds

        def __sub__(self, other):
            return SimpleNamespace(
                nanoseconds=self.nanoseconds-other.nanoseconds)

    events = []
    markers = []
    requalifications = []
    timer = SimpleNamespace(cancel=lambda: None)
    fake = SimpleNamespace(
        _done=False,
        _initializing=True,
        _initialization_timer=timer,
        _initialization_t0=FakeTime(0),
        _initialization_target_pos=np.array([20.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        _initialization_position_speeds=np.ones(6),
        _initialization_within_since_ns=700_000_000,
        _initialization_mode_ready_ns=100_000_000,
        _initialization_complete=False,
        _last_pos=[20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        _target_idx=[0, 1, 2],
        _return_tolerances=np.array([0.1, 0.5, 0.05, 0.1, 0.5, 0.5]),
        _return_position_settle_s=0.2,
        _position_status=None,
        _preflight_timeout_s=3.0,
        _preflight_timer=None,
        _send_event=lambda predicate, text='': events.append((predicate, text)),
        _marker=lambda event, **fields: markers.append((event, fields)),
        _preflight_feedback_tick=lambda: requalifications.append(True),
        _publish_position=lambda target, speed: None,
        create_timer=lambda _period, _callback: timer,
        get_clock=lambda: SimpleNamespace(now=lambda: FakeTime(1_000_000_000)),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )
    fake._initialization_position_done = lambda now_ns: (
        CollectionNode._initialization_position_done(fake, now_ns))
    fake._stop_causal_initialization_motion = lambda: (
        CollectionNode._stop_causal_initialization_motion(fake))

    CollectionNode._causal_initialization_tick(fake)

    assert fake._initialization_complete
    assert not fake._initializing
    assert [predicate for predicate, _ in events] == [
        ManagerEvent.STOP_MOTOR, ManagerEvent.MODE]
    assert events[1][1] == chr(ManagerEvent.NONE)
    assert markers[0][0] == "initialization_complete"
    assert requalifications == [True]
    assert fake._preflight_deadline_ns == 4_000_000_000


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


def test_retrying_stall_records_marker_without_finishing_run():
    finished = []
    markers = []
    warnings = []
    fake = SimpleNamespace(
        _t0=object(), _done=False, _returning=False,
        _return_status="not_started", _hardware_fault=None,
        _run_status="running",
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=321)),
        get_logger=lambda: SimpleNamespace(
            warn=warnings.append, error=lambda message: None),
        _marker=lambda event, **fields: markers.append((event, fields)),
        _finish=lambda: finished.append(True),
    )
    msg = DeviceEvent()
    msg.predicate = ManagerEvent.STALL
    msg.text = "MOTION_RETRYING:STALL"
    msg.data = [float(value) for value in [
        3, ManagerEvent.MOTION_RETRYING, ManagerEvent.FAULT_STALL,
        1, -1, 12, -40.0, 0.0, 0.0, 89, 209, 2, 0, 0, 0]]

    CollectionNode._device_event_cb(fake, msg)

    assert finished == []
    assert fake._hardware_fault is None
    assert fake._run_status == "running"
    assert markers[0][0] == "stall_retry"
    assert markers[0][1]["axis"] == 1
    assert markers[0][1]["detail"] == 2
    assert warnings


def test_protocol_v3_driver_diagnostics_are_decoded():
    finished = []
    fake = SimpleNamespace(
        _t0=object(), _done=False, _returning=False,
        _return_status="not_started", _hardware_fault=None,
        _run_status="running",
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=456)),
        get_logger=lambda: SimpleNamespace(error=lambda message: None),
        _finish=lambda: finished.append(True),
    )
    msg = DeviceEvent()
    msg.predicate = ManagerEvent.STALL
    msg.text = "MOTION_CONFIRMED:DRIVER_COMMUNICATION"
    msg.data = [float(value) for value in [
        3, ManagerEvent.MOTION_CONFIRMED,
        ManagerEvent.FAULT_DRIVER_COMMUNICATION,
        4, 5, 1, 25.0, 0.0, 0.0, 56, 0, ord("S"),
        4, 3, 4, ord("!"), ord("E"), ord("R"), ord("R")]]

    CollectionNode._device_event_cb(fake, msg)

    assert finished == [True]
    assert fake._hardware_fault["driver_stage"] == "S"
    assert fake._hardware_fault["driver_ack_failure"] == "explicit_rejection"
    assert fake._hardware_fault["driver_ack_attempts"] == 3
    assert fake._hardware_fault["driver_response_hex"] == "21 45 52 52"
    assert fake._hardware_fault["driver_response_ascii"] == "!ERR"


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
