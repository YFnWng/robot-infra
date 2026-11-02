"""Model ablation retains physical candidate eligibility and native roots."""
from types import SimpleNamespace
import numpy as np
import pytest
from runtime_supervision.engagement_qualification import compare_model_candidates
from runtime_supervision import replay_takeup_startup
from test_input_anchor_comparison import State
from catheter_control.planning.mppi import MppiConfig


def fixture(monkeypatch, mismatch=False, eligibility_difference=False):
    class Planner:
        def __init__(self, runtime, contract, config, candidates, policy):
            self.coordinate = runtime.coordinate
        def plan(self, state, position, target, **kwargs):
            ledger = {k: np.zeros(2) for k in ('requested','projected','motor_rates',
                'takeup_cost','slew_cost','boundary_cost')}
            ledger.update(eligible=np.ones(2, dtype=bool),
                total_cost=np.array([1., self.coordinate]))
            if eligibility_difference and self.coordinate > 1:
                ledger['eligible'][1] = False
            if mismatch: ledger['projected'][0] = self.coordinate
            kwargs['candidate_diagnostics'](ledger)
            state.estimator.mean[:] = -1  # caller's native root must remain owned
            return SimpleNamespace(valid=True, selected_candidate_index=int(self.coordinate < 1),
                command_logical_velocity=np.zeros(6), selected_total_cost=min(1,self.coordinate),
                zero_total_cost=1.)
    monkeypatch.setattr(replay_takeup_startup, 'FixedCandidates', Planner)
    contract = SimpleNamespace(**{k: np.zeros(6) for k in (
        'velocity_min','velocity_max','position_lower','position_upper')})
    return {str(c): (SimpleNamespace(runtime=SimpleNamespace(command_capable=False, coordinate=c),
        contract=contract, config=MppiConfig()),
        (State(), np.zeros(6), 1_000_000_000, np.ones(3), None)) for c in (2., .5)}


def test_matching_bank_changes_order_without_mutating_native_roots(monkeypatch):
    arms = fixture(monkeypatch)
    report, _ = compare_model_candidates(arms, np.zeros((2,4,3)))
    assert report['selected_candidate_changed']
    for _, snapshot in arms.values():
        np.testing.assert_array_equal(snapshot[0].mean, np.arange(32))


def test_projection_difference_rejects_comparison(monkeypatch):
    with pytest.raises(AssertionError, match='unmatched model comparison'):
        compare_model_candidates(fixture(monkeypatch, True), np.zeros((2,4,3)))


def test_model_eligibility_difference_is_reported_separately(monkeypatch):
    report, _ = compare_model_candidates(fixture(monkeypatch, eligibility_difference=True),
        np.zeros((2,4,3)))
    assert report['eligibility_difference_count'] == 1
    assert report['common_eligible_count'] == 1
    for row in report['variants'].values():
        assert row['common_eligible_cost_order'] == [0]


def test_commands_and_unmatched_times_rejected(monkeypatch):
    arms = fixture(monkeypatch)
    arms['2.0'][0].runtime.command_capable = True
    with pytest.raises(ValueError, match='non-commanding'):
        compare_model_candidates(arms, np.zeros((2,4,3)))
    arms['2.0'][0].runtime.command_capable = False
    arms['2.0'][1][0].timestamp_ns += 1
    with pytest.raises(ValueError, match='unmatched model root'):
        compare_model_candidates(arms, np.zeros((2,4,3)))
