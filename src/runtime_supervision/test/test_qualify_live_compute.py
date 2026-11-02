"""Stationary live qualification interlocks and measurement gates."""
from dataclasses import replace
from types import SimpleNamespace
import threading
import time
import numpy as np

import pytest

from catheter_control.safety.lifecycle import FreshnessLimits, GateInputs
from runtime_supervision.qualify_live_compute import (
    initialize_sensor_evidence, live_parameters, make_probe_type,
    observation_ready, sensor_evidence_locked, controller_parameters, main,
    worker_timing_evidence)
from runtime_supervision.qualify_live_compute import CallbackMeasurements, parent_pause_evidence
from runtime_supervision.compute_profile import Measurements


def ready_inputs():
    return GateInputs(position_age_s=.01, encoder_age_s=.02, marker_age_s=.03,
        position_encoder_skew_s=.01, marker_diagnostic_age_s=.01,
        position_valid=True, encoder_valid=True, model_valid=True,
        accepted_observations=10, estimator_health="TRACKING")


def test_parent_pause_correlates_gc_and_callbacks_without_claiming_cause():
    plans = [dict(valid=False, plan_started_s=10., elapsed_ms=60.,
        process_stage_marks=[dict(stage='memory_copied',monotonic_s=10.002,thread_cpu_s=1.),
                             dict(stage='before_send',monotonic_s=10.049,thread_cpu_s=1.001)]),
        dict(valid=True,plan_started_s=10.1,elapsed_ms=20.)]
    capture = dict(events=[dict(phase='start',generation=2,native_thread_id=7,
                               monotonic_s=10.003,thread_cpu_s=2.),
                          dict(phase='stop',generation=2,native_thread_id=7,
                               monotonic_s=10.048,thread_cpu_s=2.044)],dropped=0)
    spans = [('marker',10.001,10.051,7),('encoder',10.2,10.21,8)]
    evidence = parent_pause_evidence(plans,capture,spans)
    row = evidence['invalid_plans'][0]
    assert row['plan_index'] == 0 and row['gc_evidence_complete']
    assert row['stage_intervals'][0]['wall_ms'] == pytest.approx(47.)
    assert row['stage_intervals'][0]['thread_cpu_ms'] == pytest.approx(1.)
    assert row['overlapping_gc'][0]['generation'] == 2
    assert row['overlapping_callbacks'][0]['stage'] == 'marker'
    assert len(row['overlapping_callbacks']) == 1
    assert 'not proof' in evidence['interpretation']


def test_missing_gc_events_make_absence_inconclusive():
    plans = [dict(valid=False,plan_started_s=10.,elapsed_ms=60.)]
    capture = dict(events=[dict(phase='start',generation=2,native_thread_id=7,
                               monotonic_s=10.003,thread_cpu_s=2.)],dropped=1)
    evidence = parent_pause_evidence(plans,capture,[],dropped_spans=3)
    assert not evidence['invalid_plans'][0]['gc_evidence_complete']
    assert evidence['unmatched_gc_starts']
    assert evidence['callback_spans_dropped'] == 3


def test_callback_spans_are_bounded_and_preserve_exception():
    from collections import deque
    meter = CallbackMeasurements()
    meter.spans = deque(maxlen=1)
    assert meter.call('ok',1,lambda: 42) == 42
    def fail():
        raise RuntimeError('expected')
    with pytest.raises(RuntimeError,match='expected'):
        meter.call('failed',2,fail)
    assert len(meter.spans) == 1 and meter.dropped_spans == 1
    assert meter.spans[0][0] == 'failed'
    assert meter.spans[0][2] >= meter.spans[0][1]
    assert meter.summaries()['ok']['count'] == 1


def test_parameters_force_noncommanding_cpu_estimation_and_worker():
    p = live_parameters({"command_output_enabled": True, "adaptation_enabled": True,
                         "simulation_state_reset_enabled": True}, "model", "limits", "cuda")
    assert p["command_output_enabled"] is False
    assert p["adaptation_enabled"] is False
    assert p["simulation_state_reset_enabled"] is False
    assert p["estimator_device"] == "cpu" and p["planner_process_enabled"]


def test_recorded_parameters_remain_supported(tmp_path):
    path = tmp_path/'controller.json'
    path.write_text('{"controller_parameters": {"samples": 512}}')
    assert controller_parameters(path) == {"samples": 512}


def test_reviewed_experimental_stack_uses_authoritative_composition(monkeypatch):
    from pathlib import Path
    import ament_index_python.packages
    share = Path(__file__).resolve().parents[2]/'catheter_control'
    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory', lambda name: str(share))
    p = controller_parameters(stack_config='hardware_engagement_twoaxis')
    p = live_parameters(p, 'trial.json', 'limits.yaml', 'cuda')
    assert p['engagement_control_enabled'] and p['samples'] == 512
    assert p['controller_velocity_max'][1] == 0
    assert p['marker_estimator'] == 'ukf'
    assert p['command_output_enabled'] is False


def test_compute_only_excludes_manager_authorization_not_sensor_gates():
    inputs = ready_inputs()
    assert not inputs.manager_ready
    assert observation_ready(inputs, FreshnessLimits()) == "ready"
    assert inputs.manager_ready is False  # No production state mutation.


@pytest.mark.parametrize("change,reason", [
    ({"encoder_age_s": None}, "encoder_stale"),
    ({"position_age_s": .3}, "position_stale"),
    ({"marker_age_s": .3}, "accepted_marker_stale"),
    ({"accepted_observations": 0}, "estimator_initializing"),
    ({"marker_diagnostic_error": True}, "marker_diagnostic_degraded"),
    ({"collection_present": True}, "collection_node_present"),
])
def test_live_feedback_missing_or_stale_never_qualifies(change, reason):
    assert observation_ready(replace(ready_inputs(), **change), FreshnessLimits()) == reason


def test_probe_never_arms_or_publishes_even_if_parameter_changes():
    class Base:
        def _publish_velocity_locked(self, velocity):
            raise AssertionError("actuator publication")
        def _send_mode_locked(self, mode):
            raise AssertionError("mode claim")
    node = make_probe_type(Base)()
    node.command_output_enabled = True
    response = SimpleNamespace(success=True, message="")
    node._arm_cb(SimpleNamespace(data=True), response)
    assert not response.success
    node._publish_velocity_locked([1]*6)
    node._send_mode_locked(1)


def fake_live_node():
    node = make_probe_type(object)()
    node._plan_lock, node._lock, node._snapshot_lock = (
        threading.Lock(), threading.Lock(), threading.Lock())
    node.probe_stop = threading.Event()
    node.armed = node.mode_claimed = node.command_output_enabled = node.fault_latched = False
    node.limits = FreshnessLimits()
    node._gate_inputs_locked = lambda now: ready_inputs()
    node.position_valid = True
    node.position, node.observed_tip = np.zeros(6), np.array([.01, .02, .07])
    node._planner_snapshot = SimpleNamespace(timestamp_ns=123)
    node._planner_snapshot_source_time = time.monotonic()
    node._planner_transmission_snapshot = None
    node.probe_started = node.probe_target = None
    node.probe_duration_s = 1.
    node.probe_offset_mm = np.array([0., 0., 5.])
    node.probe_errors, node.probe_plans, node.probe_gate_failures = [], [], []
    node.probe_meter = Measurements()
    node.planner = SimpleNamespace(plan=lambda *a, **kw: SimpleNamespace(valid=True, reason="ok"),
                                   reset=lambda: None, process_timings_s={},
                                   process_trace=[], last_transport_failure=None)
    initialize_sensor_evidence(node)
    node._fault_snapshot_locked = lambda reason: {"reason": reason,
        "marker_age_ms": 200., "marker_update_reason": "innovation_gate"}
    node._now_ns = lambda: 3_000_000_000
    node.pending_marker_timestamp_ns = node.pending_marker_arrival = None
    node.accepted_observations = 10
    node.estimator_health = "TRACKING"
    return node


def test_worker_timing_correlates_discarded_reply_with_original_deadline_miss():
    late = dict(sequence=7, request_started_s=100., discarded=True,
                solve_wall_s=.048, solve_cpu_s=.020,
                request_to_worker_ms=4., reply_ready_to_decoded_ms=12.)
    recovered = dict(sequence=8, request_started_s=100.1, discarded=False,
                     solve_wall_s=.015, solve_cpu_s=.009)
    plans = [dict(valid=False, reason='deadline_missed', plan_started_s=100.,
                  plan_started_offset_s=4.7, elapsed_ms=62.8,
                  process_timings_ms={'process_reply_wait_decode': 53.7},
                  process_trace=[], transport_failure=None),
             dict(valid=True, plan_started_s=100.1, process_trace=[late, recovered])]
    result = worker_timing_evidence(plans)
    assert result['decoded_requests'] == 2 and result['discarded_replies'] == 1
    assert result['metrics']['solve_wall_ms']['count'] == 2
    assert result['metrics']['solve_wall_ms']['max_ms'] == pytest.approx(48.)
    assert result['invalid_plans'][0]['worker_trace'] == late
    assert result['invalid_plans'][0]['worker_trace_status'] == 'decoded'


def test_worker_timing_does_not_invent_trace_for_unsent_or_final_pending_request():
    plan = dict(valid=False, reason='deadline_missed', plan_started_s=100.,
                plan_started_offset_s=29.9, elapsed_ms=61.,
                process_timings_ms={}, process_trace=[],
                transport_failure={'bytes_sent': 0})
    result = worker_timing_evidence([plan])
    assert result['metrics'] == {} and result['decoded_requests'] == 0
    assert result['invalid_plans'][0]['worker_trace'] is None
    assert result['invalid_plans'][0]['worker_trace_status'] == 'unavailable_at_shutdown'
    assert result['invalid_plans'][0]['transport_failure'] == {'bytes_sent': 0}


def test_probe_copies_worker_trace_without_extra_plan_or_wait():
    node = fake_live_node()
    node._plan_tick()  # Discarded warmup.
    trace = dict(sequence=1, request_started_s=100., discarded=False)
    node.planner.process_trace = [trace]
    node._plan_tick()
    trace['discarded'] = True
    assert node.probe_plans[0]['process_trace'][0]['discarded'] is False
    assert not node.armed and not node.mode_claimed


def test_warmup_is_discarded_and_target_is_private_and_frozen():
    node = fake_live_node()
    node._plan_tick()
    assert node.probe_started is not None and node.probe_plans == []
    target = node.probe_target.copy()
    np.testing.assert_allclose(target, [.01, .02, .075])
    node.observed_tip += .001
    node._plan_tick()
    assert len(node.probe_plans) == 1 and node.probe_plans[0]['valid']
    np.testing.assert_array_equal(node.probe_target, target)
    assert not node.armed and not node.mode_claimed


def test_missing_encoder_never_warms_and_measured_loss_is_reported():
    node = fake_live_node()
    node._gate_inputs_locked = lambda now: replace(ready_inputs(), encoder_age_s=None)
    node._plan_tick()
    assert node.probe_started is None and not node.probe_plans
    node.probe_started = time.monotonic()
    node._plan_tick()
    assert node.probe_gate_failures[0]['reason'] == 'encoder_stale'


def test_snapshot_ages_and_measurement_boundary_are_enforced():
    node = fake_live_node()
    node._planner_snapshot_source_time -= 1.
    node._plan_tick()
    assert node.probe_started is None
    node._planner_snapshot_source_time = time.monotonic()
    node._plan_tick()
    node.probe_started -= 2.
    node._plan_tick()
    assert not node.probe_plans


def test_unexpected_arm_stops_probe_without_solve():
    node = fake_live_node()
    node.armed = True
    node._plan_tick()
    assert node.probe_stop.is_set() and node.probe_errors
    assert not node.probe_plans


def test_sensor_evidence_keeps_arrival_acceptance_and_pending_time_distinct():
    node = fake_live_node()
    node.probe_marker_callbacks = 30
    node.probe_marker_arrival = 9.99
    node.probe_marker_source_ns = 2_900_000_000
    node.pending_marker_timestamp_ns = 2_950_000_000
    node.pending_marker_arrival = 9.98
    row = sensor_evidence_locked(node, 10., "accepted_marker_stale", 2_900_000_000)
    assert row['marker_callback_count'] == 30
    assert row['marker_callback_age_ms'] == pytest.approx(10.)
    assert row['marker_age_ms'] == 200.
    assert row['marker_callback_source_age_ms'] == pytest.approx(100.)
    assert row['pending_marker_ahead_of_snapshot_ms'] == pytest.approx(50.)
    assert row['marker_update_reason'] == 'innovation_gate'


def test_evidence_is_periodic_and_immediate_on_gate_loss_and_recovery():
    node = fake_live_node()
    node._record_sensor_evidence(10., 'ready', 1)
    node._record_sensor_evidence(10.1, 'ready', 1)
    node._record_sensor_evidence(10.2, 'accepted_marker_stale', 1)
    node._record_sensor_evidence(10.3, 'ready', 1)
    node._record_sensor_evidence(11.3, 'ready', 1)
    assert [r['reason'] for r in node.probe_sensor_snapshots] == [
        'ready', 'accepted_marker_stale', 'ready', 'ready']
    assert node.probe_sensor_snapshots.maxlen == 4096
    assert node.probe_marker_updates.maxlen == 4096


def test_marker_arrivals_count_without_changing_parent_handling():
    class Base:
        def _marker_cb(self, message):
            self.handled = message
            return 'parent_result'
    node = make_probe_type(Base)()
    node._lock = threading.Lock()
    initialize_sensor_evidence(node)
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=3)))
    assert node._marker_cb(message) == 'parent_result'
    assert node.handled is message
    assert node.probe_marker_callbacks == 1
    assert node.probe_marker_source_ns == 2_000_000_003


@pytest.mark.parametrize('accepted,reason', [(True, 'accepted_noop'), (False, 'innovation_gate')])
def test_marker_outcomes_include_rejection_even_when_freshness_hides_it(accepted, reason):
    class Base:
        def _process_marker_sample(self, sample):
            self.last_marker_result = SimpleNamespace(accepted=accepted, reason=reason)
            return accepted
    node = make_probe_type(Base)()
    node._lock = threading.Lock()
    initialize_sensor_evidence(node)
    node.probe_started = None
    node.last_marker_result = None
    node.accepted_observations = 10
    node.consecutive_rejections = 0 if accepted else 1
    assert node._process_marker_sample((123,)) is accepted
    row = node.probe_marker_updates[-1]
    assert row['accepted'] is accepted and row['reason'] == reason
    assert row['source_ns'] == 123
    assert row['measurement_offset_s'] is None  # Startup evidence is retained.


@pytest.mark.parametrize("arguments", [["--duration-s", "nan"], ["--duration-s", "121"],
                                      ["--startup-timeout-s", "0"]])
def test_invalid_bounds_rejected_before_ros_initialization(arguments):
    with pytest.raises(SystemExit) as error:
        main(["--controller-manifest", "unused", "--model-manifest", "unused",
              "--limits-file", "unused", "--output", "unused", *arguments])
    assert error.value.code == 2
