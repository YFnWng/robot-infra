"""Deterministic qualification contracts, no ROS graph or device startup."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
from dataclasses import dataclass, replace
import numpy as np
import pytest

from runtime_supervision import engagement_qualification as owner
from runtime_supervision.compute_profile import ReplayEvent
from cr_meta_lnn.tests.test_engagement_runtime import initialize_proximal


def engine(monkeypatch):
    r=initialize_proximal();r.reset()
    @dataclass(frozen=True)
    class Identity:
        manifest_sha256: str = 'fixture'
    identity=Identity()
    monkeypatch.setattr(owner,'load_runtime_bundle',lambda *a,**kw:SimpleNamespace(runtime=r,identity=identity))
    return owner.EngagementQualification('fixture',
        Path(__file__).resolve().parents[2]/'control_interface/config/catheter_limits.yaml',samples=16)


def event(stamp,kind,values,quality=None,receipt=None):
    return ReplayEvent(stamp if receipt is None else receipt,stamp,kind,values,quality)


def test_future_markers_defer_then_initialize_continuously(monkeypatch):
    e=engine(monkeypatch)
    reference=initialize_proximal().current_markers()
    quality={key:np.ones(4) for key in ('confidence','source_rig_count','reprojection_error_px')}
    e.feed(event(1_000_000_000,'encoder',np.zeros(6)))
    e.feed(event(1_050_000_000,'marker',reference,quality,1_060_000_000))
    assert e.runtime.estimator is None
    e.feed(event(1_100_000_000,'encoder',np.zeros(6)))
    assert e.runtime.estimator is not None
    assert e.latest_encoder_ns==1_100_000_000
    assert e.runtime.estimator.timestamp_s==pytest.approx(1.1)
    assert e.counts['accepted_markers']==1
    e.feed(event(1_000_000_000,'encoder',np.zeros(6)))
    assert e.counts['nonmonotonic_encoder']==1


def test_bad_geometry_and_uncertainty_are_not_accepted(monkeypatch):
    with pytest.raises(ValueError):owner.marker_quality({},.001)
    with pytest.raises(ValueError):owner.EngagementQualification('none','none',marker_std_m=0.)
    quality={key:np.ones(4) for key in ('confidence','source_rig_count','reprojection_error_px')}
    quality['source_rig_count'][:2]=0
    with pytest.raises(ValueError):owner.marker_quality(quality,.001)


def test_preparation_restores_policy_memory_and_does_not_count_a_solve(monkeypatch):
    import pickle
    e=engine(monkeypatch)
    memory=pickle.dumps(e.planner.execution_memory())
    safe=e.planner._safe_result(e.planner._clock(),'deadline_missed')
    def cold(*args,**kwargs):
        e.planner._rng.random()
        e.planner._nominal.fill(1.)
        return safe
    monkeypatch.setattr(e.planner,'plan',cold)
    assert not e.prepare_planner(None)
    assert e.prepare_planner((None,np.zeros(6),0,np.zeros(3),None))
    assert pickle.dumps(e.planner.execution_memory())==memory
    assert e.counts['plans']==0 and e.counts['invalid_plans']==0
    assert e.measurements.rows[-1]['stage']=='planner_preparation'


def test_preparation_worker_failure_is_not_readiness(monkeypatch):
    e=engine(monkeypatch)
    safe=e.planner._safe_result(e.planner._clock(),'worker_unavailable')
    monkeypatch.setattr(e.planner,'plan',lambda *a,**kw:safe)
    with pytest.raises(RuntimeError,match='worker_unavailable'):
        e.prepare_planner((None,np.zeros(6),0,np.zeros(3),None))
    assert not e.prepared


def test_stale_skip_reasons_are_separate_without_relaxing_gates(monkeypatch):
    e=engine(monkeypatch)
    state=initialize_proximal().clone_state()
    snapshot=(state,np.zeros(6),0,np.zeros(3),None)
    assert e.plan_snapshot(snapshot,200_000_000) is None
    assert e.counts['plan_stale']==1
    assert e.counts['plan_stale:correction_age']==1
    assert e.counts['plan_stale:position_age']==1


def test_real_engagement_spawn_and_local_solve_conformance():
    """Opt-in real bundle smoke; CPU only, no ROS graph or actuators."""
    import os
    import torch
    manifest=os.environ.get('ENGAGEMENT_TEST_MANIFEST')
    if not manifest:
        pytest.skip('set ENGAGEMENT_TEST_MANIFEST to a reviewed G4 bundle')
    torch.set_num_threads(4)
    e=owner.EngagementQualification(manifest,
        Path(__file__).resolve().parents[2]/'control_interface/config/catheter_limits.yaml',
        samples=8,planner_process=True)
    try:
        r=e.runtime
        with torch.no_grad():
            points=r.kinematics(torch.eye(4).to(r.mechanics.mode),
                r.mechanics.natural_strain+20*r.mechanics.mode).cpu().numpy()
        r.initialize(0,np.zeros(6),points,
            tendon_confirmation=owner.tendon_confirmation(r,e.interface.snapshot()))
        assert r.estimator.external_confirmation
        assert r.estimator.phase=='UNKNOWN'
        root=r.clone_state()
        position=np.array([20.,0.,5.,0.,0.,0.])
        target=points[-1]+np.array([0.,0.,.003])
        # Conformance budget only; production configuration remains 60 ms.
        e.planner.config=replace(e.config,planning_deadline_s=5.)
        assert e.prepare_planner((root,position,0,target,None))
        local=owner.CatheterMppi(r,e.contract,e.planner.config)
        a=local.plan(root,position,target)
        b=e.planner.plan(root,position,target)
        assert a.valid and b.valid, (a.reason,b.reason)
        np.testing.assert_allclose(a.command_logical_velocity,b.command_logical_velocity,atol=1e-8)
        np.testing.assert_allclose(a.command_tip_sequence_m,b.command_tip_sequence_m,atol=1e-7)
        assert e.compute_assignment['estimator_device']=='cpu'
        assert r.command_capable is False
    finally:
        e.close()
        assert not e.planner._worker.is_alive()


def test_shadow_has_no_actuator_ros_entities():
    source=Path(owner.__file__).with_name('engagement_shadow.py').read_text()
    tree=ast.parse(source)
    calls=[node.func.attr for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)]
    assert 'create_client' not in calls
    assert 'create_service' not in calls
    publishers=[node for node in ast.walk(tree) if isinstance(node,ast.Call)
                and isinstance(node.func,ast.Attribute) and node.func.attr=='create_publisher']
    assert len(publishers)==1
    assert publishers[0].args[1].value=='status'


def test_bounded_diagnostic_report_declares_loss_and_decodes_records(monkeypatch,tmp_path):
    from runtime_supervision.compute_profile import DiagnosticRecords
    e=engine(monkeypatch)
    e.bound_diagnostics()
    e.measurements.rows=DiagnosticRecords(capacity=2)
    for stamp in range(3):
        e.measurements.rows.append(dict(stage='planner',source_ns=stamp,
            wall_ms=float(stamp),thread_cpu_ms=None))
    e.counts['plans']=3
    plans=DiagnosticRecords(capacity=2)
    plans.append(dict(source_ns=1,worker_transport={'planner_worker_error':'TimeoutError'}))
    saved=json.loads(e.report(tmp_path/'bounded',scope='test',plans=plans).read_text())
    assert saved['diagnostic_retention']['timing']['dropped']==1
    assert saved['diagnostic_evidence_complete'] is False
    assert saved['counts']['plans']==3 and saved['timing']['planner']['count']==2
    assert saved['plans'][0]['worker_transport']['planner_worker_error']=='TimeoutError'
    assert saved['diagnostic_retention']['plans']['retained']==1


def test_receipt_order_not_replaced_by_source_sort(monkeypatch,tmp_path):
    e=engine(monkeypatch)
    data=dict(schema='engagement_sensor_events_v1',events=[
        dict(receipt_ns=2,source_ns=1,kind='position',values=[0.]*6,quality=None),
        dict(receipt_ns=1,source_ns=1,kind='position',values=[0.]*6,quality=None)])
    path=tmp_path/'events.json';path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='receipt order'):owner.offline_replay(e,path)


def test_report_undefined_diagnostics_are_null_and_incomplete(monkeypatch,tmp_path):
    e=engine(monkeypatch)
    path=e.report(tmp_path/'report',scope='test',plans=[dict(cosine=float('nan'))])
    saved=json.loads(path.read_text())
    assert saved['plans'][0]['cosine'] is None
    assert saved['qualification']=='INCOMPLETE_OR_FAILED'
    assert (path.parent/'timing.csv').is_file()


def test_finite_json_preserves_numpy_scalars_and_nested_arrays():
    data=dict(width=np.float32(.125),count=np.int64(7),enabled=np.bool_(True),
              samples=np.array([[1.,np.nan],[np.inf,-np.inf]],dtype=np.float32),
              nested=(dict(value=np.float64(.5)),))
    saved=json.loads(json.dumps(owner.finite_json(data),allow_nan=False))
    assert saved==dict(width=.125,count=7,enabled=True,
                       samples=[[1.,None],[None,None]],nested=[dict(value=.5)])
    # Unsupported objects must still fail instead of silently losing evidence.
    with pytest.raises(TypeError):
        json.dumps(owner.finite_json(dict(unsupported=object())),allow_nan=False)


def test_live_report_separates_cold_and_warmed_baseline_misses(monkeypatch,tmp_path):
    e=engine(monkeypatch)
    e.baseline=[dict(source_ns=1,controller_reason='warming_planner',
                    planner_elapsed_ms=130.,plan_valid=False,plan_reason='deadline_missed'),
                dict(source_ns=2,controller_reason='planning_first_command',
                    planner_elapsed_ms=25.,plan_valid=True,plan_reason='ok'),
                dict(source_ns=3,controller_reason='post_takeup_command_ready',
                    planner_elapsed_ms=61.,plan_valid=False,plan_reason='deadline_missed'),
                dict(source_ns=4,planner_elapsed_ms=10.,plan_valid=True,plan_reason='ok')]
    report=json.loads(e.report(tmp_path/'report',scope='test',
                             live_context={'stop_reason':'duration_limit'}).read_text())
    summary=report['baseline_execution']
    assert summary['cold_rows']==1 and summary['warmed_rows']==2
    assert summary['unclassified_rows']==1
    assert summary['warmed_deadline_misses']==1
    assert summary['valid_planner_elapsed_ms']['p50_ms']==25.
    assert report['live_context']['stop_reason']=='duration_limit'


def test_deadline_result_without_prediction_is_reported_and_next_plan_recovers(monkeypatch):
    e=engine(monkeypatch)
    e.runtime=initialize_proximal()
    state=e.runtime.clone_state()
    state.estimator.phase='ENGAGED'
    state.estimator.direction=1
    state.estimator.registration_since_s=-1.
    belief=SimpleNamespace(phase=('ENGAGED','UNKNOWN','ENGAGED'))
    snapshot=(state,np.zeros(6),0,np.zeros(3),belief)
    safe=e.planner._safe_result(e.planner._clock(),'deadline_missed')
    monkeypatch.setattr(e.planner,'plan',lambda *a,**kw:safe)
    # Invalid solves cannot start a transaction or diagnostic rollout.
    e.arbiter=SimpleNamespace(begin=lambda *a:SimpleNamespace(reason='zero'))
    result=e.plan_snapshot(snapshot,0,defer_diagnostics=True)
    assert not result['valid'] and result['reason']=='deadline_missed'
    assert result['best_tip_sequence_m'] is None
    assert e.deferred_rollout_check is None
    assert e.counts['invalid_plans']==1
    good=replace(safe,valid=True,reason='ok',best_tip_sequence_m=np.zeros((4,3)))
    monkeypatch.setattr(e.planner,'plan',lambda *a,**kw:good)
    result=e.plan_snapshot(snapshot,0,defer_diagnostics=True)
    assert result['valid'] and e.deferred_rollout_check is not None


def test_callback_exception_cannot_receive_component_pass(monkeypatch,tmp_path):
    e=engine(monkeypatch)
    e.counts.update(accepted_markers=1,plans=1,callback_exceptions=1)
    report=json.loads(e.report(tmp_path/'report',scope='test').read_text())
    assert report['qualification']=='INCOMPLETE_OR_FAILED'


@pytest.mark.parametrize('defer_diagnostics', [False, True])
def test_actual_shadow_plan_does_not_execute_or_latch_transaction(monkeypatch, defer_diagnostics):
    from catheter_control.transmission.backlash import BacklashSnapshot
    e=engine(monkeypatch)
    e.runtime=initialize_proximal()
    e.planner.backend=e.runtime
    e.config=replace(e.config,planning_deadline_s=2.)
    e.planner.config=e.config
    s=e.runtime.estimator
    s.phase='ENGAGED';s.direction=1;s.registration_since_s=-1.
    state=e.runtime.clone_state()
    target=e.runtime.current_markers()[-1]+np.array([0.,0.,.003])
    belief=BacklashSnapshot(width_rad=np.ones(3),width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3),remaining_rad=np.zeros(3),
        motion_direction=np.ones(3,dtype=np.int8),engaged_direction=np.ones(3,dtype=np.int8),
        confidence=np.ones(3),confirmation_count=np.ones(3,dtype=np.int32),
        phase=('ENGAGED','UNKNOWN','ENGAGED'))
    snapshot=(state,np.array([20.,0.,5.,40.,0.,0.]),0,target,belief)
    result=e.plan_snapshot(snapshot,0,defer_diagnostics=defer_diagnostics)
    assert result['valid'], result['reason']
    assert result['command_output_enabled'] is False
    assert e.arbiter.state==e.arbiter.READY_TO_PLAN
    if defer_diagnostics:
        assert result['passive_and_coarse_fine'] is None
        assert not e.coarse_fine_checked
        assert not any(row['stage']=='coarse_fine_check' for row in e.measurements.rows)
        e.finish_deferred_diagnostics()
        assert e.post_run_rollout_check['result']['coarse_steps']==4
        assert e.post_run_rollout_check['scope']=='after_shadow_executor_shutdown'
        e.finish_deferred_diagnostics()
        assert sum(row['stage']=='post_run_coarse_fine_check' for row in e.measurements.rows)==1
    else:
        assert result['passive_and_coarse_fine']['coarse_steps']==4
        assert e.deferred_rollout_check is None


def test_deferred_diagnostic_failure_preserves_report(monkeypatch,tmp_path):
    e=engine(monkeypatch)
    e.deferred_rollout_check=(0,None,np.zeros((1,3)))
    def fail(*args):
        raise ValueError('diagnostic fixture failure')
    monkeypatch.setattr(owner,'compare_rollout_steps',fail)
    e.finish_deferred_diagnostics()
    saved=json.loads(e.report(tmp_path/'report',scope='test').read_text())
    assert saved['counts']['diagnostic_failures']==1
    assert saved['post_run_rollout_check']['error']=='diagnostic fixture failure'


def test_invalid_encoder_does_not_mutate_history(monkeypatch):
    e=engine(monkeypatch)
    with pytest.raises(ValueError,match='six-axis encoder'):
        e.feed(event(1,'encoder',[float('nan')]*6))
    assert e.latest_encoder_ns is None


def test_progress_reports_recorded_time_and_wall_time(monkeypatch,tmp_path):
    e=engine(monkeypatch)
    path=tmp_path/'events.json'
    path.write_text(json.dumps(dict(schema='engagement_sensor_events_v1',events=[
        dict(receipt_ns=1_000_000_000,source_ns=1_000_000_000,
             kind='position',values=[0.]*6,quality=None),
        dict(receipt_ns=2_000_000_000,source_ns=2_000_000_000,
             kind='position',values=[0.]*6,quality=None)])))
    logs=[]
    owner.offline_replay(e,path,progress=logs.append)
    assert logs[0]['stage']=='replay_started'
    assert logs[-1]['stage']=='replay_complete'
    assert logs[-1]['recorded_elapsed_s']==1.
    assert logs[-1]['event_index']==2
    assert logs[-1]['counts']['position']==2
    assert logs[-1]['wall_elapsed_s']>=0.
    assert 'latest_timing_ms' in logs[-1]
    with pytest.raises(ValueError,match='progress interval'):
        owner.offline_replay(e,path,progress_interval_s=0.)


def test_engagement_plot_is_generated_from_observed_evidence(monkeypatch,tmp_path):
    e=engine(monkeypatch)
    quality={key:np.ones(4) for key in ('confidence','source_rig_count','reprojection_error_px')}
    reference=initialize_proximal().current_markers()
    e.feed(event(1_000_000_000,'encoder',np.zeros(6)))
    e.feed(event(1_000_000_000,'marker',reference,quality))
    e.feed(event(1_100_000_000,'encoder',np.zeros(6)))
    e.feed(event(1_100_000_000,'marker',reference,quality))
    path=e.report(tmp_path/'report',scope='test')
    assert (path.parent/'engagement.png').stat().st_size>1000
    saved=json.loads(path.read_text())
    assert saved['trace'][-1]['confirmation']['policy']=='baseline_bending_coordinate'
    assert saved['trace'][-1]['confirmation']['threshold']==.05
    assert saved['trace'][-1]['confirmation']['observations_required']==3
    assert 'raw_tendon_motor_mm' in saved['trace'][0]
