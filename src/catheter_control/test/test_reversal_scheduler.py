import numpy as np

from catheter_control.transmission.reversal_scheduler import (
    ReversalDirectionScheduler, ReversalSchedulerConfig)


def _scheduler(**changes):
    values = dict(
        required_plans=3,
        minimum_absolute_cost_improvement=5.0,
        minimum_fractional_cost_improvement=0.0,
        minimum_terminal_error_improvement_mm=.25,
        minimum_accepted_observations=3,
        cooldown_s=1.0)
    values.update(changes)
    return ReversalDirectionScheduler(ReversalSchedulerConfig(**values))


def test_reversal_requires_margin_persistence_observations_and_cooldown():
    scheduler = _scheduler()
    scheduler.synchronize(
        ("ENGAGED", "UNKNOWN", "UNKNOWN"), [1, 0, 0], 10, 0.0)

    first = scheduler.observe_plan(
        [-1, 0, 0], [20, 0, 0], [.5, 0, 0], 11, .5)
    second = scheduler.observe_plan(
        [-1, 0, 0], [20, 0, 0], [.5, 0, 0], 13, 1.1)
    approved = scheduler.observe_plan(
        [-1, 0, 0], [20, 0, 0], [.5, 0, 0], 14, 1.2)

    assert first.approved_direction.tolist() == [0, 0, 0]
    assert second.approved_direction.tolist() == [0, 0, 0]
    assert approved.pending_count.tolist() == [3, 0, 0]
    assert approved.approved_direction.tolist() == [-1, 0, 0]
    assert approved.reason == "reversal_approved"


def test_weak_or_alternating_reversal_never_receives_approval():
    scheduler = _scheduler(required_plans=2)
    scheduler.synchronize(
        ("ENGAGED", "UNKNOWN", "UNKNOWN"), [1, 0, 0], 10, 0.0)

    weak = scheduler.observe_plan(
        [-1, 0, 0], [20, 0, 0], [.1, 0, 0], 20, 2.0)
    changed = scheduler.observe_plan(
        [1, 0, 0], [30, 0, 0], [1, 0, 0], 21, 2.1)
    restarted = scheduler.observe_plan(
        [-1, 0, 0], [30, 0, 0], [1, 0, 0], 22, 2.2)

    assert weak.reason == "reversal_cost_margin"
    assert changed.pending_count.tolist() == [0, 0, 0]
    assert restarted.pending_count.tolist() == [1, 0, 0]
    assert not np.any(restarted.approved_direction)


def test_approved_transaction_is_consumed_and_new_response_moves_lease():
    scheduler = _scheduler(
        required_plans=1, minimum_accepted_observations=0, cooldown_s=0.0)
    scheduler.synchronize(
        ("ENGAGED", "UNKNOWN", "UNKNOWN"), [1, 0, 0], 10, 0.0)
    scheduler.observe_plan(
        [-1, 0, 0], [30, 0, 0], [1, 0, 0], 10, 0.0)

    scheduler.mark_transaction_started([-1, 0, 0])
    consumed = scheduler.snapshot(10, 0.1)
    assert consumed.lease_direction.tolist() == [1, 0, 0]
    assert consumed.approved_direction.tolist() == [0, 0, 0]

    updated = scheduler.synchronize(
        ("PROVISIONAL", "UNKNOWN", "UNKNOWN"), [-1, 0, 0], 11, .2)
    assert updated.lease_direction.tolist() == [-1, 0, 0]
    assert updated.observations_since_lease.tolist() == [0, 0, 0]


def test_multi_axis_reversal_is_approved_from_per_axis_evidence():
    scheduler = _scheduler(
        required_plans=1, minimum_accepted_observations=0, cooldown_s=0.0)
    scheduler.synchronize(
        ("ENGAGED", "ENGAGED", "UNKNOWN"), [1, 1, 0], 10, 0.0)

    result = scheduler.observe_plan(
        [-1, -1, 0], [20, 20, 0], [.5, .1, 0], 10, 0.0)

    assert result.approved_direction.tolist() == [-1, 0, 0]
    assert result.terminal_error_improvement_mm.tolist() == [.5, .1, 0]
