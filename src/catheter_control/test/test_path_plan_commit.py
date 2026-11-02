"""Exercise the real planning callback with an inert owner, never a ROS node.

No publishers, device links, manager services or heartbeat exist in this fixture.
Only framework timing instrumentation is bypassed; readiness and commit logic
are the production implementations.
"""
from contextlib import nullcontext
from dataclasses import replace
from threading import Lock, RLock
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

from catheter_control.node import CatheterControlNode
from catheter_control.planning.path_tracking import ReferenceHorizon
from catheter_control.safety.lifecycle import ControllerState, FreshnessLimits, GateInputs


def fixture():
    reference = ReferenceHorizon(
        path_id="circle", sequence=1, source_timestamp_ns=800_000_000,
        received_at_s=100., sample_period_s=.02,
        positions_m=np.tile([.02, .02, .06], (41, 1)), expiry_s=.15,
        progress_m=.001, total_length_m=.025, final_hold=False,
        progress_paused=False, tangents=np.tile([1., 0., 0.], (41, 1)))
    gates = GateInputs(
        manager_ready=True, manager_age_s=0., position_age_s=0.,
        encoder_age_s=0., position_encoder_skew_s=0., marker_age_s=0.,
        marker_diagnostic_age_s=0., accepted_observations=10,
        target_available=True, position_valid=True, encoder_valid=True,
        model_valid=True, estimator_health="TRACKING")
    faults, published = [], []
    plan = SimpleNamespace(
        valid=True, reason="ok", command_logical_velocity=np.array([1., 0., .2, 0., 0., 0.]),
        proposed_reversal_direction=np.zeros(3), reversal_axis_cost_improvement=np.zeros(3),
        reversal_axis_terminal_error_improvement_mm=np.zeros(3), command_tip_sequence_m=None)
    node = SimpleNamespace(
        _lock=RLock(), _plan_lock=Lock(), _snapshot_lock=Lock(),
        _steady=lambda: 100.04, reset_pending=False, armed=True, fault_latched=False,
        estimator_catchup_started_at=None,
        state=ControllerState.ACTIVE, limits=FreshnessLimits(), gates=gates,
        planner_warmed=True, mode_claimed=True, mode_requested_at=99., mode_settle_s=.01,
        engagement_control_enabled=False, takeup_transaction_enabled=False,
        position=np.zeros(6), target=np.zeros(3), observed_tip=None,
        reference_source="PATH", path_reference=reference,
        last_command=np.ones(6), last_effective_command=np.ones(6), last_command_time=100.,
        blocked_motor_direction=np.zeros(3), accepted_observations=10,
        backlash_snapshot=SimpleNamespace(phase=["ENGAGED"]*3, engaged_direction=np.ones(3),
                                          taking_up=np.zeros(3, dtype=bool)),
        takeup_response_free_mask=np.zeros(3, dtype=bool), contract=None,
        backlash_compensator=SimpleNamespace(motor_radians_per_second=lambda _: np.zeros(3)),
        reversal_scheduler=SimpleNamespace(
            synchronize=lambda *_: SimpleNamespace(lease_direction=np.zeros(3), approved_direction=np.zeros(3)),
            observe_plan=lambda *_: None),
        _planner_snapshot=SimpleNamespace(timestamp_ns=1_000_000_000),
        _planner_snapshot_source_time=100., _planner_transmission_snapshot=None,
        _planner_tip_readout_offset_m=None,
        planner_process_enabled=True, consecutive_deadline_misses=0, maximum_deadline_misses=3,
        _timing=SimpleNamespace(record_seconds=lambda *_: None),
        _scan_collection_locked=lambda: None, _gate_inputs_locked=lambda _: node.gates,
        _clear_estimator_catchup_locked=lambda: None,
        _pause_for_estimator_catchup_locked=lambda *_: False,
        _refresh_blocked_motor_directions_locked=lambda *_: None,
        _timed_lock=lambda *_: nullcontext(),
        _publish_control_cycle_timing_locked=lambda *_: None,
        _compensated_command_locked=lambda command, _: command.copy(),
        _publish_plan_locked=published.append,
        get_logger=lambda: SimpleNamespace(info=lambda *_: None, warn=lambda *_: None))
    def fault(reason):
        faults.append(reason)
        node.fault_latched = True
        node.last_command.fill(0.)
    node._fault_locked = fault
    node._planner_reference_matches_locked = MethodType(
        CatheterControlNode._planner_reference_matches_locked, node)
    node.planner = SimpleNamespace(
        config=SimpleNamespace(horizon_steps=4, step_s=.04), process_timings_s={},
        plan=lambda *_args, **_kwargs: plan,
        enforce_deadline=lambda result, _: result,
        note_capture_control_executed=lambda *_: None)
    return node, reference, plan, faults, published


def tick(node):
    # This inert fixture omits timer instrumentation and process GC ownership.
    from inspect import unwrap
    unwrap(CatheterControlNode._plan_tick)(node)


def test_small_refresh_commits_real_callback_command():
    node, reference, plan, faults, published = fixture()
    def solve(*_, **__):
        node.path_reference = replace(reference, sequence=2, progress_m=.00105,
            positions_m=reference.positions_m+5e-6,
            tangents=np.tile([1., .0015, 0.], (41, 1)))
        return plan
    node.planner.plan = solve
    tick(node)
    np.testing.assert_array_equal(node.last_command, plan.command_logical_velocity)
    assert node.state_reason == "active" and published == [plan] and not faults


def recovery_fixture(engagement=False):
    node, reference, plan, faults, published = fixture()
    node.gates = replace(node.gates, encoder_receive_age_s=0.)
    node.engagement_control_enabled = engagement
    node.model_diagnostics = {"prediction_ready": True}
    node.estimator_catchup_observations = 0
    node.estimator_catchup_pause_count = 0
    node.estimator_catchup_timeout_s = .35
    node.zeros = []
    node._publish_velocity_locked = lambda value: node.zeros.append(value.copy())
    node._pause_for_estimator_catchup_locked = MethodType(
        CatheterControlNode._pause_for_estimator_catchup_locked, node)
    node._clear_estimator_catchup_locked = MethodType(
        CatheterControlNode._clear_estimator_catchup_locked, node)
    return node, plan, faults, published


def test_pause_budget_is_shared_and_heartbeat_cannot_reuse_command():
    from inspect import unwrap
    node, _, faults, _ = recovery_fixture()
    node.consecutive_deadline_misses = 1
    assert node._pause_for_estimator_catchup_locked(100., "accepted_marker_stale")
    node.last_command[:] = 5.
    unwrap(CatheterControlNode._heartbeat_tick)(node)
    assert not np.any(node.last_command) and not np.any(node.zeros[-1])
    assert node.estimator_catchup_started_at == 100.
    assert node.estimator_catchup_pause_count == 1
    assert node.consecutive_deadline_misses == 1
    assert not node._pause_for_estimator_catchup_locked(100.36, "deadline_missed")
    assert not faults


@pytest.mark.parametrize("engagement", [False, True], ids=["baseline", "engagement"])
def test_recovery_requires_new_accepted_observation_then_valid_solve(engagement):
    node, plan, faults, published = recovery_fixture(engagement)
    node._pause_for_estimator_catchup_locked(100., "accepted_marker_stale")
    tick(node)
    assert not published and node.estimator_catchup_started_at == 100.
    node.accepted_observations += 1
    node._planner_snapshot_source_time = 100.02
    tick(node)
    assert published == [plan] and not faults
    assert node.estimator_catchup_started_at is None
    assert node.state_reason == "active"


def test_interruption_during_solve_discards_pre_pause_plan():
    node, plan, faults, published = recovery_fixture()
    def solve(*_, **__):
        node._pause_for_estimator_catchup_locked(100.01, "marker_diagnostic_error")
        node.accepted_observations += 1
        return plan
    node.planner.plan = solve
    tick(node)
    assert not published and not faults
    assert node.estimator_catchup_started_at == 100.01
    assert not np.any(node.last_command)


def test_ready_gates_without_recovery_observation_still_expire():
    node, _, faults, _ = recovery_fixture()
    node._pause_for_estimator_catchup_locked(99.6, "accepted_marker_stale")
    tick(node)
    assert faults == ["runtime_recovery_timeout"]


def test_pause_preserves_takeup_owner_and_requires_fresh_decision():
    from inspect import unwrap
    from catheter_control.transmission.backlash import TakeupTransactionArbiter
    node, _, faults, _ = recovery_fixture()
    advances = []
    decision = SimpleNamespace(state=TakeupTransactionArbiter.TAKEUP_ACTIVE,
        command_logical_velocity=np.array([1., 0., 0., 0., 0., 0.]), reason="takeup_active")
    arbiter = SimpleNamespace(state=TakeupTransactionArbiter.TAKEUP_ACTIVE, travel=2.4,
        advance=lambda *args: advances.append(args) or decision)
    node.takeup_transaction_enabled = True
    node.takeup_arbiter = arbiter
    node._pause_for_estimator_catchup_locked(100., "accepted_marker_stale")
    unwrap(CatheterControlNode._heartbeat_tick)(node)
    tick(node)
    assert not advances and not faults and arbiter.travel == 2.4
    node.accepted_observations += 1
    tick(node)
    assert len(advances) == 1 and node.takeup_arbiter is arbiter
    assert arbiter.travel == 2.4 and node.estimator_catchup_started_at is None
    assert node.last_command[0] == 1.


@pytest.mark.parametrize("change", [
    {"positions_m": np.tile([.02003, .02, .06], (41, 1))},
    {"tangents": np.tile([1., .003, 0.], (41, 1))},
    {"progress_m": .0012}, {"progress_m": .00099},
    {"path_id": "other"}, {"sequence": 0},
    {"final_hold": True}, {"progress_paused": True}, {"total_length_m": .03},
])
def test_changed_objective_zeros_then_retries(change):
    node, reference, plan, faults, published = fixture()
    def solve(*_, **__):
        node.path_reference = replace(reference, **({"sequence": 2} | change))
        return plan
    node.planner.plan = solve
    tick(node)
    assert node.state_reason == "planner_reference_changed_zero"
    assert not np.any(node.last_command) and not np.any(node.last_effective_command)
    assert not faults and not published
    node.planner.plan = lambda *_, **__: plan
    tick(node)
    assert node.state_reason == "active" and published == [plan]


@pytest.mark.parametrize("event,reason", [
    ("expiry", "path_reference_stale"),
    ("snapshot", "planner_snapshot_stale_at_commit"),
    ("readiness", "manager_not_ready"),
])
def test_freshness_still_faults_at_commit(event, reason):
    node, reference, plan, faults, published = fixture()
    def solve(*_, **__):
        if event == "expiry":
            node.path_reference = replace(reference, received_at_s=99.)
        elif event == "snapshot":
            node._steady = lambda: 100.2
        else:
            node.gates = replace(node.gates, manager_age_s=1.)
        return plan
    node.planner.plan = solve
    tick(node)
    assert faults == [reason] and not published and not np.any(node.last_command)


def test_cancellation_during_solve_cannot_commit():
    node, _, plan, faults, published = fixture()
    def solve(*_, **__):
        node.armed = False
        node.reference_source = "NONE"
        node.last_command.fill(0.)
        return plan
    node.planner.plan = solve
    tick(node)
    assert not published and not faults and not np.any(node.last_command)


@pytest.mark.parametrize("reason,zero_reason", [
    ("deadline_missed", "runtime_pause_zero:deadline_missed"),
    ("worker_state_changed", "planner_state_changed_zero"),
])
def test_invalid_solve_zero_and_valid_retry(reason, zero_reason):
    node, plan, faults, published = recovery_fixture()
    plan.valid, plan.reason = False, reason
    tick(node)
    assert node.state_reason == zero_reason and not np.any(node.last_command)
    assert not faults
    node.accepted_observations += 1
    node._planner_snapshot_source_time = 100.04
    plan.valid, plan.reason = True, "ok"
    tick(node)
    assert node.state_reason == "active"
    np.testing.assert_array_equal(node.last_command, plan.command_logical_velocity)


def test_repeated_deadlines_still_latch_fault():
    node, plan, faults, _ = recovery_fixture()
    plan.valid, plan.reason = False, "deadline_missed"
    for _ in range(3):
        node.accepted_observations += 1
        node._planner_snapshot_source_time = 100.04
        tick(node)
    assert faults == ["planner:repeated_deadline_miss"]


def test_final_hold_endpoint_not_relaxed():
    node, reference, plan, faults, _ = fixture()
    node.path_reference = reference = replace(reference, final_hold=True)
    def solve(*_, **__):
        node.path_reference = replace(reference, sequence=2, positions_m=reference.positions_m+5e-6)
        return plan
    node.planner.plan = solve
    tick(node)
    assert node.state_reason == "planner_reference_changed_zero" and not faults
    assert not np.any(node.last_command)
