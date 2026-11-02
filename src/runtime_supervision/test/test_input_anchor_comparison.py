"""Offline anchor ablation isolates one coordinate and preserves its owner."""
import copy
from types import SimpleNamespace
import numpy as np
import pytest

from runtime_supervision.engagement_qualification import compare_input_anchors
from runtime_supervision import replay_takeup_startup


class State:
    def __init__(self):
        self.estimator=SimpleNamespace(mean=np.arange(32,dtype=float),prediction_ready=True)
        self.timestamp_ns=1_000_000_000
        self.strain=np.zeros(24)

    @property
    def mean(self):return self.estimator.mean.copy()

    def clone(self):return copy.deepcopy(self)


def setup(monkeypatch, mismatch=False):
    original=State(); seen=[]
    class Planner:
        def __init__(self,runtime,contract,config,candidates,policy):
            self.candidates=candidates.copy()
        def plan(self,root,position,target,**kwargs):
            seen.append(root.mean.copy())
            coordinate=root.mean[30]
            ledger={key:np.array([0.,1.]) for key in (
                'tracking_cost','requested','projected','motor_rates','eligible',
                'takeup_cost','slew_cost','boundary_cost')}
            if mismatch and coordinate==3.:ledger['projected'][0]=99.
            kwargs['candidate_diagnostics'](ledger)
            return SimpleNamespace(valid=True,selected_candidate_index=int(coordinate==3.),
                command_logical_velocity=np.zeros(6),selected_total_cost=coordinate,zero_total_cost=9.)
    monkeypatch.setattr(replay_takeup_startup,'FixedCandidates',Planner)
    from catheter_control.planning.mppi import MppiConfig
    runtime=SimpleNamespace(profile=SimpleNamespace(kind='v171_static'),
        mechanics=SimpleNamespace(stationary_lambda=lambda q:3.),
        export_planning_state=lambda s:s.mean.copy())
    engine=SimpleNamespace(runtime=runtime,contract=None,config=MppiConfig())
    snapshot=(original,np.zeros(6),original.timestamp_ns,np.ones(3),None)
    return engine,snapshot,seen


def test_only_input_anchor_changes_and_owner_is_untouched(monkeypatch):
    engine,snapshot,seen=setup(monkeypatch)
    bank=np.zeros((2,4,3));before=snapshot[0].mean.copy()
    row,ledgers,roots=compare_input_anchors(engine,snapshot,bank)
    assert row['selected_candidate_changed']
    np.testing.assert_array_equal(snapshot[0].mean,before)
    np.testing.assert_array_equal(seen[0],before)
    expected=before.copy();expected[30]=3.
    np.testing.assert_array_equal(seen[1],expected)
    np.testing.assert_array_equal(roots['posterior'],before)
    np.testing.assert_array_equal(bank,np.zeros_like(bank))


def test_unmatched_projection_fails_instead_of_claiming_an_ablation(monkeypatch):
    engine,snapshot,_=setup(monkeypatch,mismatch=True)
    with pytest.raises(AssertionError,match='unmatched anchor comparison'):
        compare_input_anchors(engine,snapshot,np.zeros((2,4,3)))


def test_wrong_prior_and_stale_position_are_rejected(monkeypatch):
    engine,snapshot,_=setup(monkeypatch)
    engine.runtime.profile.kind='fitted'
    with pytest.raises(ValueError,match='static prior'):
        compare_input_anchors(engine,snapshot,np.zeros((2,4,3)))
    engine.runtime.profile.kind='v171_static'
    with pytest.raises(ValueError,match='skew'):
        compare_input_anchors(engine,(snapshot[0],snapshot[1],0,*snapshot[3:]),np.zeros((2,4,3)))


def test_unknown_planner_input_cannot_silently_change_the_ablation(monkeypatch):
    engine,snapshot,_=setup(monkeypatch)
    with pytest.raises(ValueError,match='unknown anchor comparison planner inputs'):
        compare_input_anchors(engine,snapshot,np.zeros((2,4,3)),
            planner_inputs={'disable_safety':True})


def test_bank_cli_is_separate_and_defaults_to_multiple_seeds():
    from runtime_supervision.engagement_g4 import parser
    args=parser().parse_args(['compare-banks','--manifest','manifest.json',
        '--anchor-report','anchor_comparison.json','--outdir','new-output'])
    assert args.seeds==[17,29,43]
    assert args.device=='cpu' and not args.planner_process
