"""Fail-closed suite reporting; no ROS graph, camera or device connection."""
import json
import gc
from dataclasses import asdict
from types import SimpleNamespace
import numpy as np
import pytest
from runtime_supervision.qualify_engagement_integration import (
    planner_evidence, replay_gc, sensor_fixture, verdict)
from catheter_control.orchestration.task_gc import TaskGcPolicy
from runtime_supervision.engagement_qualification import finite_json
from catheter_control.transmission.backlash import BacklashConfig


def test_confirmation_config_with_float32_widths_is_strict_json():
    config=BacklashConfig(width_rad=tuple(np.array([1.25,2.5,3.75],dtype=np.float32)))
    report=dict(confirmation_policy='shared_baseline_bending_coordinate',
                confirmation_config=asdict(config))
    saved=json.loads(json.dumps(finite_json(report),allow_nan=False))
    assert saved['confirmation_config']['width_rad']==[1.25,2.5,3.75]
    assert saved['confirmation_policy']=='shared_baseline_bending_coordinate'


def fixture_file(tmp_path, rows):
    path=tmp_path/'events.json'
    path.write_text(json.dumps({'schema':'engagement_sensor_events_v1','events':rows}))
    return path


def row(stamp,kind='encoder',values=None):
    return dict(receipt_ns=stamp,source_ns=stamp,kind=kind,
        values=[0.]*6 if values is None else values,quality=None)


def test_matched_sensor_grid_preserves_prefix_and_recorded_target(tmp_path):
    rows=[row(0,'target',[.01,.02,.03])]+[row(i*10_000_000) for i in range(11)]
    path=fixture_file(tmp_path,rows)
    events,targets,start,end=sensor_fixture(path,.04,.04)
    encoders=[e.source_ns for e in events if e.kind=='encoder']
    assert encoders==[0,20_000_000,40_000_000,60_000_000,80_000_000]
    assert start==40_000_000 and end==80_000_000
    np.testing.assert_array_equal(targets[0][1],[.01,.02,.03])
    assert [e.receipt_ns for e in events]==sorted(e.receipt_ns for e in events)
    assert any(e.receipt_ns<start for e in events)


def test_incomplete_window_and_missing_target_fail(tmp_path):
    path=fixture_file(tmp_path,[row(0),row(100_000_000)])
    with pytest.raises(ValueError,match='beyond recording'):
        sensor_fixture(path,.1,.1)
    with pytest.raises(ValueError,match='recorded target'):
        sensor_fixture(path,0,.05)


def test_receipt_order_cannot_be_repaired_by_sorting(tmp_path):
    path=fixture_file(tmp_path,[row(2),row(1)])
    with pytest.raises(ValueError,match='nonmonotonic'):
        sensor_fixture(path,0,1.)


def test_verdict_does_not_hide_missing_or_invalid_work():
    good={'valid':True,'reason':'ok'}
    corrections=[{'accepted':True}]
    assert not verdict([{'baseline':good,'experimental':good}],corrections)
    assert 'no_matched_plans' in verdict([],corrections)
    assert 'experimental_unavailable_at_matched_receipt' in verdict(
        [{'baseline':good,'experimental':None}],corrections)
    assert 'baseline_invalid_plan' in verdict(
        [{'baseline':{'valid':False},'experimental':good}],corrections)
    assert 'no_baseline_marker_correction' in verdict(
        [{'baseline':good,'experimental':good}],[])
    assert 'baseline_marker_rejection' in verdict(
        [{'baseline':good,'experimental':good}],[{'accepted':True},{'accepted':False}])


@pytest.mark.parametrize('policy_name', ['normal', 'task-boundary'])
@pytest.mark.parametrize('original_enabled', [False, True])
@pytest.mark.parametrize('fail', [False, True])
def test_replay_gc_restores_after_success_or_exception(policy_name, original_enabled, fail):
    saved = (gc.isenabled(), gc.get_threshold(), list(gc.callbacks))
    evidence = {}
    try:
        gc.enable() if original_enabled else gc.disable()
        try:
            with replay_gc(policy_name, evidence) as check:
                check(None)
                assert gc.isenabled() == (original_enabled and policy_name == 'normal')
                if fail:
                    raise RuntimeError('injected replay failure')
        except RuntimeError as error:
            assert fail and str(error) == 'injected replay failure'
        assert gc.isenabled() == original_enabled
        assert gc.get_threshold() == saved[1]
        assert gc.callbacks == saved[2]
        assert evidence['summary']['collections'] == int(policy_name == 'task-boundary')
        assert not evidence['summary']['active']
        assert not evidence['summary']['pending']
        assert not evidence['trace']['dropped']
        assert evidence['started_s'] <= evidence['work_finished_s']
        assert evidence['restored_enabled'] == evidence['original_enabled']
    finally:
        gc.set_threshold(*saved[1])
        gc.enable() if saved[0] else gc.disable()


@pytest.mark.parametrize('guard', ['memory', 'duration'])
def test_replay_gc_guard_uses_shared_owner_and_restores(monkeypatch, guard):
    import runtime_supervision.qualify_engagement_integration as runner
    state = SimpleNamespace(now=0., rss=1000)
    def factory(**kwargs):
        return TaskGcPolicy(**kwargs, clock=lambda: state.now, rss=lambda: state.rss)
    monkeypatch.setattr(runner, 'TaskGcPolicy', factory)
    enabled, thresholds, callbacks = gc.isenabled(), gc.get_threshold(), list(gc.callbacks)
    evidence = {}
    reason = 'task_gc_rss_growth_limit' if guard == 'memory' else 'task_gc_duration_limit'
    with pytest.raises(RuntimeError, match=reason):
        with replay_gc('task-boundary', evidence) as check:
            if guard == 'memory':
                state.rss += 257*1024**2
            else:
                state.now = 301.
            check(None)
    assert evidence['active_summary']['last_guard_reason'] == reason
    assert evidence['summary']['collections'] == 1
    assert (gc.isenabled(), gc.get_threshold(), gc.callbacks) == (enabled, thresholds, callbacks)


def test_planner_evidence_owns_rows_before_next_request():
    planner = SimpleNamespace(worker_diagnostics=lambda: {'planner_worker_error': ''},
        process_timings_s={'worker_root_copy': .001},
        process_stage_marks=[{'stage': 'sent', 'monotonic_s': 123.}],
        process_trace=[{'discarded': True}])
    evidence = planner_evidence(planner)
    planner.process_stage_marks[0]['stage'] = 'overwritten'
    planner.process_timings_s.clear()
    planner.process_trace[0]['discarded'] = False
    assert evidence['process_stage_marks'][0]['stage'] == 'sent'
    assert evidence['process_timings_s'] == {'worker_root_copy': .001}
    assert evidence['process_trace'][0]['discarded'] is True


def test_replay_gc_rejects_unknown_policy_without_modifying_gc():
    saved = gc.isenabled(), gc.get_threshold(), list(gc.callbacks)
    with pytest.raises(ValueError, match='unknown replay GC policy'):
        with replay_gc('unbounded', {}):
            pytest.fail('unknown policy admitted work')
    assert (gc.isenabled(), gc.get_threshold(), gc.callbacks) == saved
