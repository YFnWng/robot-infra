"""Commanding reduced-filter backend of catheter_mppi: the real owner, plan and heartbeat callbacks on an
inert fixture with a scripted filter owner. No ROS node, device link, manager or hardware."""
from contextlib import nullcontext
from inspect import unwrap
from pathlib import Path
from threading import Lock, RLock
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

from catheter_control.node import CatheterControlNode
from catheter_control.orchestration.configuration import ACTIVE_CONTROLLER_PARAMETERS, locate_stack, resolve_stack
from catheter_control.orchestration.reduced_filter_owner import OrderedInbox, OwnerHealth, RootResult
from catheter_control.orchestration.reduced_filter_settings import require_filter_owned_switches_off
from catheter_control.safety.lifecycle import ControllerState, FreshnessLimits, controller_motion_held, readiness

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE/'config'
SOURCE = PACKAGE/'catheter_control'/'node.py'
LIMITS = FreshnessLimits(manager_s=.5, feedback_s=.15, marker_s=.25, marker_diagnostic_s=.5, feedback_pair_skew_s=.15)


class Owner:
    """Scripted ReducedFilterOwner: process() returns queued (status, source_ns[, rejections]) results."""

    def __init__(self, results=(), ready=True):
        self.results, self.ready, self.health_value = list(results), ready, OwnerHealth('INITIALIZING', False, None, 0)
        self.encoders, self.images, self.history_lost, self.preparations = [], [], False, []
        self.root = RootResult(SimpleNamespace(plan_ns=2_000_000_000, image_ns=1_900_000_000, law_count=1),
                               np.array([20., 0., 1., 0., 0., 0.]), None, .001)

    def add_encoder(self, stamp, values):
        self.encoders.append(stamp)

    def add_image(self, stamp, points):
        self.images.append(stamp)

    def mark_history_lost(self):
        self.history_lost = True

    def process(self):
        if not self.results:
            return None
        status, source_ns, *rejections = self.results.pop(0)
        health = {'tracking': 'TRACKING', 'holding': 'HOLDING', 'fault': 'FAULT', 'initialized': 'INITIALIZING'}[status]
        self.health_value = OwnerHealth(health, health == 'TRACKING' and self.ready, None if status == 'tracking'
                                        else status, rejections[0] if rejections else int(status == 'holding'))
        return SimpleNamespace(status=status, source_ns=source_ns, reason=None if status == 'tracking' else status)

    def health(self):
        return self.health_value

    def plan_root(self):
        return self.root


def owner_node(owner, **overrides):
    planned = []
    node = SimpleNamespace(
        reduced_filter=True, reduced_owner=owner, reduced_inbox=OrderedInbox(), _lock=RLock(), _steady=lambda: 100.,
        encoder_time=None, consecutive_rejections=0, estimator_health='UNINITIALIZED', model_valid=False,
        last_marker_result=None, accepted_observations=0, marker_time=None, runtime_initialized=False,
        armed=True, fault_latched=False, state=ControllerState.ACTIVE, state_reason='active',
        last_command=np.ones(6), last_effective_command=np.ones(6), last_command_time=99.,
        reduced_last_plan_image_ns=None, reduced_plan_attempted=False, plan_rate_hz=5., faults=[])
    node._fault_if_active = node.faults.append

    def plan_tick():
        planned.append(node.accepted_observations)
        node.reduced_plan_attempted = True
    node._plan_tick = plan_tick
    for key, value in overrides.items():
        setattr(node, key, value)
    return node, planned


def owner_tick(node):
    unwrap(CatheterControlNode._reduced_owner_tick)(node)


def test_gate_fields_are_written_only_by_the_events_they_certify():
    owner = Owner([('initialized', 1_000), ('tracking', 2_000)])
    node, _ = owner_node(owner)
    node.reduced_inbox.put(('encoder', 5, [0.]*6, 42.))
    node.reduced_inbox.put(('marker', 6, np.zeros((4, 3)), 43.))
    owner_tick(node)
    assert owner.encoders == [5] and owner.images == [6]
    assert node.encoder_time == 42.                    # absorbed encoder arrival, not the image
    assert node.accepted_observations == 0 and node.marker_time is None   # initialization is not a tracked commit
    assert node.last_marker_result.accepted and node.model_valid
    owner_tick(node)                                   # no new encoder: encoder_time must not advance
    assert node.encoder_time == 42. and node.accepted_observations == 1 and node.marker_time == 100.
    assert node.estimator_health == 'TRACKING' and node.runtime_initialized


def test_holding_commands_zero_and_maps_to_a_tolerated_transient_rejection():
    node, planned = owner_node(Owner([('holding', 3_000, 1)]))
    owner_tick(node)
    assert not node.last_command.any() and not node.last_effective_command.any() and node.last_command_time == 100.
    assert node.state_reason == 'reduced_filter_holding_zero' and controller_motion_held(node.state_reason)
    assert node.estimator_health == 'DEGRADED' and not node.last_marker_result.accepted and not planned


def test_an_owner_fault_invalidates_the_model_gate():
    node, _ = owner_node(Owner([('fault', 3_000, 3)]))
    owner_tick(node)
    assert not node.model_valid and node.estimator_health == 'FAULT'


def test_input_overflow_loses_history_and_faults_an_active_controller():
    owner = Owner()
    node, _ = owner_node(owner, reduced_inbox=OrderedInbox(maximum_events=1))
    node.reduced_inbox.put(('encoder', 1, [0.]*6, 1.))
    node.reduced_inbox.put(('encoder', 2, [0.]*6, 2.))
    owner_tick(node)
    assert owner.history_lost and node.faults == ['reduced_filter_input_overflow']


def test_plans_follow_committed_images_and_a_skipped_plan_keeps_the_due_image():
    period_ns = 100_000_000
    owner = Owner([('tracking', k*period_ns) for k in range(7)])
    node, planned = owner_node(owner)
    for _ in range(3):
        owner_tick(node)
    assert planned == [1, 3]                           # every second committed image at 10 Hz
    owner.ready = False                                # caught-up check fails: not planned, due image kept
    owner_tick(node)
    assert planned == [1, 3] and node.reduced_last_plan_image_ns == 2*period_ns
    owner.ready = True
    owner_tick(node)                                   # image 4 retries immediately
    assert planned == [1, 3, 5] and node.reduced_last_plan_image_ns == 4*period_ns


def gate_node(**fields):
    node = SimpleNamespace(
        manager_ready=True, manager_time=100., position_time=100., encoder_time=100., encoder_receive_time=100.,
        position_source_timestamp_ns=1, encoder_source_timestamp_ns=1, marker_time=100., marker_diagnostic_time=100.,
        marker_diagnostic_error=False, accepted_observations=10, required_observations=8, consecutive_rejections=0,
        maximum_rejections=3, target=np.zeros(3), collection_present=False, position_valid=True, encoder_valid=True,
        model_valid=True, estimator_health='TRACKING', last_marker_result=None)
    for key, value in fields.items():
        setattr(node, key, value)
    return readiness(MethodType(CatheterControlNode._gate_inputs_locked, node)(100.), LIMITS)[1]


def test_each_reduced_gate_fails_closed():
    rejected = SimpleNamespace(accepted=False, reason='holding')
    assert gate_node() == 'ready'
    assert gate_node(accepted_observations=7) == 'estimator_initializing'
    assert gate_node(model_valid=False) == 'model_invalid'
    assert gate_node(encoder_time=99.8) == 'encoder_stale'
    assert gate_node(marker_time=99.7) == 'accepted_marker_stale'          # recoverable: bounded zero pause
    assert gate_node(estimator_health='INITIALIZING') == 'estimator_degraded'
    assert gate_node(estimator_health='DEGRADED', consecutive_rejections=1, last_marker_result=rejected) == 'ready'
    assert gate_node(estimator_health='DEGRADED', consecutive_rejections=3,
                     last_marker_result=rejected).startswith('repeated_marker_rejection')


def plan_node(owner, plan):
    calls = []
    gates = 'ready'
    node = SimpleNamespace(
        reduced_filter=True, reduced_owner=owner, _lock=RLock(), _plan_lock=Lock(), _snapshot_lock=Lock(),
        _steady=lambda: 100., reset_pending=False, armed=True, fault_latched=False, estimator_catchup_started_at=None,
        state=ControllerState.ACTIVE, state_reason='active', limits=LIMITS, planner_warmed=True, mode_claimed=True,
        mode_requested_at=99., mode_settle_s=.01, engagement_control_enabled=False, takeup_transaction_enabled=False,
        position=np.full(6, 9.), target=np.array([.02, .02, .06]), observed_tip=None, reference_source='POINT',
        path_reference=None, last_command=np.ones(6), last_effective_command=np.ones(6), last_command_time=99.9,
        blocked_motor_direction=np.zeros(3), accepted_observations=10, encoder_time=99.99,
        backlash_snapshot=SimpleNamespace(phase=['ENGAGED']*3, engaged_direction=np.ones(3),
                                          taking_up=np.zeros(3, dtype=bool)),
        takeup_response_free_mask=np.zeros(3, dtype=bool), contract=None, planner_process_enabled=False,
        consecutive_deadline_misses=0, maximum_deadline_misses=3, reduced_plan_attempted=False,
        reversal_scheduler=SimpleNamespace(
            synchronize=lambda *_: SimpleNamespace(lease_direction=np.zeros(3), approved_direction=np.zeros(3)),
            observe_plan=lambda *_: None),
        backlash_compensator=SimpleNamespace(motor_radians_per_second=lambda _: np.zeros(3)),
        _timing=SimpleNamespace(record_seconds=lambda *_: None), _scan_collection_locked=lambda: None,
        _gate_inputs_locked=lambda _: None, _clear_estimator_catchup_locked=lambda: None,
        _pause_for_estimator_catchup_locked=lambda *_: False, _refresh_blocked_motor_directions_locked=lambda *_: None,
        _timed_lock=lambda *_: nullcontext(), _publish_control_cycle_timing_locked=lambda *_: None,
        _compensated_command_locked=lambda command, _: command.copy(), _publish_plan_locked=lambda *_: None,
        get_logger=lambda: SimpleNamespace(info=lambda *_: None, warn=lambda *_: None), faults=[])

    def fault(reason):
        node.faults.append(reason)
        node.fault_latched = True

    def plan_call(*args, **kwargs):
        calls.append((args, kwargs))
        if isinstance(plan, Exception):
            raise plan
        return plan
    node._fault_locked = fault
    node.planner = SimpleNamespace(
        config=SimpleNamespace(horizon_steps=4, step_s=.04, point_rollout_step_s=.2), plan=plan_call,
        enforce_deadline=lambda result, _: result, note_capture_control_executed=lambda *_: None,
        point_capture_hold_active=lambda: False)
    return node, calls, gates


def plan_tick(node, monkeypatch):
    import catheter_control.node as module
    monkeypatch.setattr(module, 'readiness', lambda *_: (ControllerState.READY, 'ready'))
    unwrap(CatheterControlNode._plan_tick)(node)


def valid_plan():
    return SimpleNamespace(
        valid=True, reason='ok', command_logical_velocity=np.array([1., 0., .2, 0., 0., 0.]),
        proposed_reversal_direction=np.zeros(3), reversal_axis_cost_improvement=np.zeros(3),
        reversal_axis_terminal_error_improvement_mm=np.zeros(3), command_tip_sequence_m=None, elapsed_s=.01)


def test_a_committed_plan_becomes_the_command_with_the_dry_run_planner_call(monkeypatch):
    owner = Owner()
    node, calls, _ = plan_node(owner, valid_plan())
    plan_tick(node, monkeypatch)
    (args, kwargs), = calls
    assert args[0] is owner.root.root and np.array_equal(args[1], owner.root.joint_position)   # encoder-derived
    assert np.array_equal(args[2], node.target) and set(kwargs) == {'deadline_started_s'}
    assert np.array_equal(node.last_command, [1., 0., .2, 0., 0., 0.]) and node.state == ControllerState.ACTIVE
    assert node.reduced_plan_attempted and not node.faults


def test_a_refused_root_commands_zero_and_is_not_an_attempt(monkeypatch):
    owner = Owner()
    owner.root = RootResult(None, None, 'root_refused: holding', .001)
    node, calls, _ = plan_node(owner, valid_plan())
    plan_tick(node, monkeypatch)
    assert not calls and not node.last_command.any() and not node.last_effective_command.any()
    assert node.state_reason == 'reduced_filter_root_refused_zero' and not node.reduced_plan_attempted


def test_a_planner_error_commands_zero_instead_of_escaping(monkeypatch):
    node, calls, _ = plan_node(Owner(), ValueError('inputs outside support'))
    plan_tick(node, monkeypatch)
    assert len(calls) == 1 and not node.last_command.any() and not node.faults
    assert node.state_reason == 'reduced_filter_planner_error_zero' and controller_motion_held(node.state_reason)


def test_path_references_are_refused(monkeypatch):
    node, calls, _ = plan_node(Owner(), valid_plan())
    node.reference_source = 'PATH'
    plan_tick(node, monkeypatch)
    assert not calls and node.faults == ['reduced_filter_path_reference_unsupported']


def heartbeat_node(last_command_time, state_reason='active'):
    published = []
    node = SimpleNamespace(
        _lock=RLock(), mode_claimed=True, armed=True, fault_latched=False, _steady=lambda: 100., limits=LIMITS,
        _gate_inputs_locked=lambda _: None, estimator_catchup_started_at=None, state=ControllerState.ACTIVE,
        takeup_transaction_enabled=False, last_command_time=last_command_time, command_timeout_s=.40,
        state_reason=state_reason, last_command=np.array([1., 0., .2, 0., 0., 0.]),
        last_effective_command=np.array([1., 0., .2, 0., 0., 0.]), position=np.zeros(6),
        contract=SimpleNamespace(project_publishable_velocity=lambda command, _: command),
        _compensated_command_locked=lambda command, _: command.copy(), _publish_velocity_locked=published.append,
        _pause_for_estimator_catchup_locked=lambda *_: False, faults=[])
    node._fault_locked = node.faults.append
    return node, published


def test_heartbeat_repeats_a_fresh_command_and_faults_after_the_command_timeout(monkeypatch):
    import catheter_control.node as module
    monkeypatch.setattr(module, 'readiness', lambda *_: (ControllerState.READY, 'ready'))
    node, published = heartbeat_node(last_command_time=100.-.35)           # inside 0.40 s: replayed
    unwrap(CatheterControlNode._heartbeat_tick)(node)
    assert len(published) == 1 and np.array_equal(published[0], [1., 0., .2, 0., 0., 0.]) and not node.faults
    node, published = heartbeat_node(last_command_time=100.-.45)           # beyond: latched stale fault
    unwrap(CatheterControlNode._heartbeat_tick)(node)
    assert not published and node.faults == ['planned_command_stale']


def test_commanding_stack_resolves_with_the_decided_settings():
    stack = resolve_stack(locate_stack('hardware_reduced_filter_no_rotation_half_speed', CONFIG), config_root=CONFIG,
                          allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS), expected_node_name='catheter_mppi')
    p = stack.parameters
    assert p['estimator_backend'] == 'reduced_filter'
    assert (p['command_timeout_s'], p['marker_timeout_s'], p['plan_rate_hz'], p['planning_deadline_s']) == (.40, .25, 5.0, .05)
    assert p['controller_velocity_max'] == [5.0, 0.0, 2.25, 2.0, 12.5, 12.5]
    assert (p['task_gc_enabled'], p['task_gc_maximum_s'], p['task_gc_growth_mb']) == (True, 300.0, 256.0)
    assert p['reduced_filter_tendon_upper_mm'] == 10.8 and 'command_output_enabled' not in p
    require_filter_owned_switches_off(lambda name: p.get(name, False))


def test_every_filter_owned_switch_and_the_planner_process_are_refused():
    for name in ('backlash_compensation_enabled', 'takeup_transaction_enabled', 'engaged_gain_enabled',
                 'reversal_scheduler_enabled', 'engagement_control_enabled', 'adaptation_enabled',
                 'planner_process_enabled'):
        with pytest.raises(ValueError, match=name):
            require_filter_owned_switches_off(lambda key, name=name: key == name)


def test_the_commanding_controller_never_sends_set_zero():
    import re
    text = SOURCE.read_text(encoding='utf-8')
    assert not re.search(r'\bSET_ZERO\b', text) and not re.search(r'\bset_zero\b', text, re.IGNORECASE)
