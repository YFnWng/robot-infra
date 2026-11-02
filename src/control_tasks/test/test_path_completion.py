"""Action serialization regression checks without ROS nodes or motion."""
from types import SimpleNamespace
import threading

from builtin_interfaces.msg import Time
import numpy as np
import pytest

from catheter_control.planning.path_tracking import (
    ArcLengthPath, GovernorConfig, PathProgressGovernor)
from control_tasks.path_action import TipPathActionServer


def test_action_result_reports_endpoint_not_latched_reference_error():
    update = SimpleNamespace(endpoint_error_mm=3.05, reference_error_mm=7.97,
                             covered_arc_m=.01, progress_m=.005,
                             coverage_fraction=1., traversal_complete=True)
    result = TipPathActionServer._result(None, False, "timeout", [1.0], update)
    assert result.final_error_mm == pytest.approx(3.05)


@pytest.mark.parametrize('failure', [None, 'wrong_id', 'stale', 'missing',
                                    'fault', 'armed', 'cancel'])
def test_path_reference_acknowledgement_precedes_arm(monkeypatch, failure):
    from control_tasks import path_action
    clock = SimpleNamespace(t=10.)
    monkeypatch.setattr(path_action.time, 'monotonic', lambda: clock.t)
    monkeypatch.setattr(path_action.time, 'sleep',
                        lambda delay: setattr(clock, 't', clock.t+delay))
    monkeypatch.setattr(path_action.rclpy, 'ok', lambda: True)
    node = SimpleNamespace(
        _lock=threading.Lock(), service_timeout_s=.2, feedback_timeout_s=.1,
        rate_hz=30., controller_armed=failure == 'armed',
        controller_state='FAULTED' if failure == 'fault' else 'DISARMED',
        controller_time=9., controller_reference_source='POINT',
        controller_path_id=None, controller_path_sequence=0)
    goal = SimpleNamespace(is_cancel_requested=failure == 'cancel')
    publications = []
    def pump():
        publications.append(clock.t)
        # Mimic independent topic/service callback scheduling: the first two
        # publications have not been consumed by the controller yet.
        if len(publications) < 3 or failure == 'missing':
            return
        node.controller_reference_source = 'PATH'
        node.controller_path_id = 'other' if failure == 'wrong_id' else 'circle'
        node.controller_path_sequence = 1
        node.controller_time = 9. if failure == 'stale' else clock.t
    ready, _ = TipPathActionServer._wait_reference_ready(node, 'circle', pump, goal)
    assert ready == (failure is None)
    if ready:
        assert len(publications) == 3
        assert node.controller_reference_source == 'PATH'
        assert node.controller_path_id == 'circle'
        assert not node.controller_armed
    if failure in {'fault', 'armed', 'cancel'}:
        assert publications == []


@pytest.mark.parametrize('sequence', ['7', 'none', 'invalid'])
def test_path_status_caches_reference_identity_atomically(sequence):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    node = SimpleNamespace(_lock=threading.Lock())
    status = DiagnosticStatus(name='catheter_control/mppi', message='DISARMED',
        values=[KeyValue(key=k, value=v) for k, v in {
            'armed': 'False', 'reason': 'disarmed_by_user',
            'reference_source': 'PATH', 'path_reference_id': 'circle',
            'path_reference_sequence': sequence}.items()])
    TipPathActionServer._status_cb(node, DiagnosticArray(status=[status]))
    assert node.controller_reference_source == 'PATH'
    assert node.controller_path_id == 'circle'
    assert node.controller_path_sequence == (7 if sequence == '7' else 0)
    assert not node.controller_armed


def test_execute_waits_for_topic_ack_before_calling_arm(monkeypatch):
    from types import MethodType
    from control_tasks import path_action
    monkeypatch.setattr(path_action.rclpy, 'ok', lambda: True)
    path = ArcLengthPath([[0., 0., 0.], [.01, 0., 0.]])
    config = GovernorConfig(.001, 10., 1., .1, 2., 4., 1.5, 8.)
    events = []
    node = SimpleNamespace(
        _lock=threading.Lock(), _goal_reserved=True,
        service_timeout_s=.5, feedback_timeout_s=.5, preview_s=.6,
        position_lookahead_s=.6, rate_hz=30.,
        controller_armed=False, controller_state='DISARMED',
        controller_time=0., controller_reference_source='POINT',
        controller_path_id=None, controller_path_sequence=0,
        _spec=lambda request: (path, config), _publish_path=lambda path: None,
        _invalidate=lambda *args: None,
        _result=lambda success, message, *args, **kwargs: SimpleNamespace(message=message))
    node._wait_reference_ready = MethodType(TipPathActionServer._wait_reference_ready, node)
    def snapshot():
        now = path_action.time.monotonic()
        return (np.zeros(3), now, node.controller_armed,
                node.controller_state, 'test', now)
    node._snapshot = snapshot
    def publish(path_id, sequence, *args):
        events.append('publish')
        if sequence >= 3:
            node.controller_reference_source = 'PATH'
            node.controller_path_id = path_id
            node.controller_path_sequence = sequence
            node.controller_time = path_action.time.monotonic()
    node._publish_reference = publish
    def arm(armed, pump=None):
        events.append('arm' if armed else 'disarm')
        if armed:
            assert node.controller_reference_source == 'PATH'
            assert node.controller_path_sequence >= 3
        node.controller_armed = armed
        # End the fake action immediately after the correctly ordered arm.
        node.controller_state = 'FAULTED'
        return True, 'test'
    node._set_armed = arm
    goal = SimpleNamespace(request=SimpleNamespace(auto_arm=True, x_only=False, xz_only=False),
                           is_cancel_requested=False, abort=lambda: None)
    TipPathActionServer._execute(node, goal)
    assert events[:4] == ['publish', 'publish', 'publish', 'arm']
    assert events[-1] == 'disarm'
    assert not node._goal_reserved


def stationary_update():
    path = ArcLengthPath([[0, 0, 0], [.01, 0, 0], [0, 0, 0]])
    config = GovernorConfig(.01, 10, 1, .1, 2, 4, 1.5, 8)
    governor = PathProgressGovernor(path, config)
    governor.start(0)
    governor.update([0, 0, 0], 0)
    governor.progress_m = path.length_m
    return governor.update([0, 0, 0], .1), config, path


def test_failed_or_canceled_result_retains_actual_coverage_not_reference():
    update, _, path = stationary_update()
    result = TipPathActionServer._result(
        None, False, "path canceled", [.1, .2], update)
    assert not result.success
    assert result.completed_arc_length_m == 0
    assert result.reference_arc_length_m == pytest.approx(path.length_m)
    assert result.path_coverage == 0
    assert not result.traversal_complete
    assert result.rms_closest_path_error_mm == pytest.approx(np.sqrt(.025))


def test_feedback_distinguishes_full_reference_from_no_traversal():
    update, config, path = stationary_update()
    feedback = TipPathActionServer._feedback(
        None, "circle", update, config, "TRACKING", path.length_m)
    assert feedback.progress == 1
    assert feedback.covered_arc_length_m == 0
    assert feedback.path_coverage == 0
    assert not feedback.traversal_complete


def test_existing_trace_schema_retains_reference_progress_for_bag_compatibility():
    update, config, path = stationary_update()
    messages = []
    node = SimpleNamespace(
        frame_id="robot_base",
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=Time)),
        trace_pub=SimpleNamespace(publish=messages.append))
    TipPathActionServer._publish_trace(
        node, "circle", 1, update, config, np.zeros(3), "TRACKING", path.length_m)
    assert messages[0].progress_m == pytest.approx(path.length_m)
    assert messages[0].reference_error_mm == 0


def test_unstarted_failure_has_no_fabricated_coverage():
    result = TipPathActionServer._result(None, False, "feedback unavailable", [])
    assert result.completed_arc_length_m == result.reference_arc_length_m == 0
    assert not result.traversal_complete


@pytest.mark.parametrize("objective", [{}, {"x_only": True}, {"xz_only": True}])
def test_reference_publication_refreshes_time_without_moving_spatial_target(objective):
    from rclpy.time import Time as RosTime
    path = ArcLengthPath([[0, 0, 0], [.02, 0, 0]])
    config = GovernorConfig(.001, 10, 1, .1, 2, 4, 1.5, 8)
    governor = PathProgressGovernor(path, config, **objective)
    governor.start(0)
    messages, points = [], []
    clock = SimpleNamespace(stamp=10_000_000_000)
    node = SimpleNamespace(
        history_s=.2, preview_s=.6, preview_dt=.02,
        frame_id="robot_base", reference_expiry_s=.15,
        get_clock=lambda: SimpleNamespace(now=lambda: RosTime(nanoseconds=clock.stamp)),
        reference_pub=SimpleNamespace(publish=messages.append),
        point_pub=SimpleNamespace(publish=points.append))
    for i in range(3):
        update = governor.update([0, 0, 0], i*.033)
        TipPathActionServer._publish_reference(node, "test", i+1, governor, update)
        clock.stamp += 33_000_000
    assert messages[0].header.stamp != messages[-1].header.stamp
    assert all(message.positions == messages[0].positions for message in messages)
    assert all(message.tangents == messages[0].tangents for message in messages)
    assert all(message.progress_m == 0 for message in messages)
    assert messages[0].positions[0].x > 0
    assert points[0].point == messages[0].positions[0]
    assert all(message.x_only == objective.get("x_only", False) for message in messages)
    assert all(message.xz_only == objective.get("xz_only", False) for message in messages)
