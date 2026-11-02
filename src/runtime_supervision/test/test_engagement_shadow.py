"""Live owner scheduling contracts; no ROS graph or actuator startup."""
from collections import Counter
from types import SimpleNamespace
import threading
import time

from runtime_supervision.engagement_shadow import ShadowInputs, EngagementShadow, BaselineActivity
from runtime_supervision.compute_profile import ReplayEvent, GcTrace, ReplayGcExperiment
from runtime_supervision.compute_profile import DiagnosticRecords
from runtime_supervision.compute_profile import ObserverQuiescence
import pytest


def test_boundary_admission_preserves_parallel_work_and_recovers_after_exception():
    guard=ObserverQuiescence()
    with guard.work() as first, guard.work() as second:
        assert first and second and guard.workers==2
        with guard.boundary() as admitted:assert not admitted
    with pytest.raises(RuntimeError):
        with guard.boundary() as admitted:
            assert admitted
            with guard.work() as work:assert not work
            raise RuntimeError('maintenance failed')
    assert guard.workers==0 and not guard.maintenance
    with guard.work() as admitted:assert admitted


def test_maintenance_skips_callbacks_without_consuming_input():
    guard=ObserverQuiescence()
    node=SimpleNamespace(gc_quiescence=guard,engine=SimpleNamespace(counts=Counter()))
    with guard.boundary():
        EngagementShadow.owner_callback(node)
        EngagementShadow.plan_callback(node)
    assert node.engine.counts=={'gc_maintenance_owner_skips':1,
                               'gc_maintenance_planner_skips':1}


def test_task_gc_requires_observed_task_boundaries_before_ros_startup():
    from runtime_supervision.engagement_shadow import main
    with pytest.raises(ValueError,match='requires stop-after-targets'):
        main(['--manifest','unused.json','--outdir','unused',
              '--gc-policy','task-boundary'])


@pytest.mark.parametrize('domain,simulated,duration',
    [('214',False,60),('42',True,60),('42',False,301)])
def test_live_gc_rejects_replay_or_unbounded_use(domain,simulated,duration):
    with pytest.raises(ValueError):
        ReplayGcExperiment('live-task-boundary',duration,512,domain,simulated)


def test_live_gc_requires_isolated_planner_before_ros_startup():
    from runtime_supervision.engagement_shadow import main
    with pytest.raises(ValueError,match='isolated planner'):
        main(['--manifest','unused.json','--outdir','unused',
              '--gc-policy','live-task-boundary','--stop-after-targets','4'])


@pytest.mark.parametrize('state,source,receipt,expected', [
    ('DISARMED',950_000_000,1.,True),('ACTIVE',950_000_000,1.,False),
    ('READY',950_000_000,1.,False),('DISARMED',None,1.,False),
    ('DISARMED',800_000_000,1.,False),('DISARMED',1_100_000_000,1.,False),
    ('DISARMED',950_000_000,.5,False)])
def test_live_boundary_requires_fresh_source_and_receipt(monkeypatch,state,source,receipt,expected):
    from runtime_supervision.engagement_shadow import fresh_disarmed_status
    monkeypatch.setattr('runtime_supervision.engagement_shadow.time.monotonic',lambda:1.)
    activity=SimpleNamespace(state=state,source_ns=source,receipt_s=receipt)
    assert fresh_disarmed_status(activity,1_000_000_000)==expected


@pytest.mark.parametrize('policy',['task-boundary','live-task-boundary'])
def test_task_gc_collects_once_at_fresh_disarmed_quiescent_boundary(monkeypatch,policy):
    import gc
    calls=[];clock=[1.]
    monkeypatch.setattr('runtime_supervision.compute_profile.time.monotonic',lambda:clock[0])
    monkeypatch.setattr(gc,'collect',lambda *args:calls.append(args) or 3)
    monkeypatch.setattr(ReplayGcExperiment,'rss_bytes',staticmethod(lambda:1000))
    experiment=ReplayGcExperiment(policy,10,512,
        '42' if policy=='live-task-boundary' else '214',policy!='live-task-boundary')
    guard=ObserverQuiescence()
    original=gc.isenabled()
    try:
        experiment.start()
        assert not gc.isenabled()
        assert experiment.collect_boundary(1,'ACTIVE',1.,guard) is None
        assert experiment.collect_boundary(1,'DISARMED',.1,guard) is None
        with guard.work():
            assert experiment.collect_boundary(1,'DISARMED',1.,guard) is None
        assert experiment.collect_boundary(1,'DISARMED',1.,guard,
            planner_idle=lambda:False) is None
        row=experiment.collect_boundary(1,'DISARMED',1.,guard)
        assert row['observed_state']=='DISARMED' and row['collected']==3
        assert experiment.collect_boundary(1,'DISARMED',1.,guard) is None
        assert experiment.collect_boundary(2,'DISARMED',1.,guard) is not None
        assert calls==[(),(2,),(2,)]
    finally:
        report=experiment.stop()
    assert gc.isenabled()==original
    assert len(report['boundary_collections'])==2


def test_task_gc_collection_exception_restores_on_exit(monkeypatch):
    import gc
    experiment=ReplayGcExperiment('task-boundary',10,512,'214',True)
    guard=ObserverQuiescence();original=gc.isenabled()
    monkeypatch.setattr(ReplayGcExperiment,'rss_bytes',staticmethod(lambda:1000))
    experiment.start()
    def fail(*args):raise RuntimeError('collection failure')
    monkeypatch.setattr(gc,'collect',fail)
    try:
        with pytest.raises(RuntimeError):
            experiment.collect_boundary(1,'DISARMED',time.monotonic(),guard)
    finally:experiment.stop()
    assert not guard.maintenance and gc.isenabled()==original


def test_diagnostic_records_are_bounded_untracked_payloads_and_independent():
    import gc
    records=DiagnosticRecords(capacity=2)
    row={'curve':[[1.,2.,3.]]}
    records.append(row)
    row['curve'][0][0]=99.
    assert records[0]['curve'][0][0]==1.
    records.append({'value':2})
    records.append({'value':3})
    assert list(records)==[{'value':2},{'value':3}]
    assert all(isinstance(p,bytes) and not gc.is_tracked(p) for p in records.records)
    assert records.retention()['total']==3
    assert records.retention()['dropped']==1


def test_diagnostic_byte_limit_and_oversized_record_are_explicit():
    records=DiagnosticRecords(capacity=10,byte_limit=100)
    for n in range(10):records.append({'value':n,'payload':'x'*30})
    records.append({'oversized':'x'*200})
    meta=records.retention()
    assert meta['encoded_bytes']<=100
    assert meta['total']==11 and meta['dropped']+meta['retained']==11
    assert list(records)[-1]['value']==9


def test_engine_bounds_only_diagnostics_before_input():
    from runtime_supervision.engagement_qualification import EngagementQualification
    from runtime_supervision.compute_profile import Measurements
    engine=EngagementQualification.__new__(EngagementQualification)
    engine.measurements=Measurements()
    for name in ('trace','worker_trace','baseline','baseline_shapes'):setattr(engine,name,[])
    history=object();engine.encoders=history
    engine.bound_diagnostics()
    assert engine.encoders is history
    assert engine.measurements.rows.capacity==65536
    assert engine.trace.capacity==4096
    engine.trace.append({'source_ns':1})
    with pytest.raises(ValueError):engine.bound_diagnostics()


@pytest.mark.parametrize('domain,simulated,duration', [('42',True,60),('214',False,60),('214',True,181)])
@pytest.mark.parametrize('policy', ['deferred','task-boundary'])
def test_deferred_gc_rejects_nonisolated_or_unbounded_use(domain, simulated, duration, policy):
    with pytest.raises(ValueError):
        ReplayGcExperiment(policy,duration,512,domain,simulated)


@pytest.mark.parametrize('enabled', [True,False])
@pytest.mark.parametrize('policy', ['deferred','task-boundary','live-task-boundary'])
def test_deferred_gc_restores_policy_and_bounds_memory(monkeypatch, enabled, policy):
    import gc
    original=(gc.isenabled(),gc.get_threshold())
    clock=[0.]
    rss=[100*1024**2]
    monkeypatch.setattr('runtime_supervision.compute_profile.time.monotonic',lambda:clock[0])
    monkeypatch.setattr(ReplayGcExperiment,'rss_bytes',staticmethod(lambda:rss[0]))
    experiment=ReplayGcExperiment(policy,10,2,
        '42' if policy=='live-task-boundary' else '214',policy!='live-task-boundary')
    try:
        gc.enable() if enabled else gc.disable()
        experiment.start()
        assert not gc.isenabled()
        rss[0]+=3*1024**2
        clock[0]=1.
        assert experiment.check()=='gc_memory_growth_limit'
        clock[0]=10.
        rss[0]=100*1024**2
        assert experiment.check()=='gc_duration_limit'
        report=experiment.stop()
        assert report['restored_enabled']==enabled
        assert report['restored_thresholds']==original[1]
        assert report['max_sampled_rss_bytes']==103*1024**2
    finally:
        experiment.stop()
        gc.set_threshold(*original[1])
        gc.enable() if original[0] else gc.disable()


def test_normal_gc_reference_retains_policy(monkeypatch):
    import gc
    monkeypatch.setattr(ReplayGcExperiment,'rss_bytes',staticmethod(lambda:1024))
    original=(gc.isenabled(),gc.get_threshold())
    experiment=ReplayGcExperiment('normal',180,512,'214',True)
    try:
        experiment.start()
        assert original==(gc.isenabled(),gc.get_threshold())
    finally:
        report=experiment.stop()
    assert report['policy']=='normal'
    assert original==(gc.isenabled(),gc.get_threshold())


@pytest.mark.parametrize('growth', [0, -1, float('nan'), 1025])
def test_gc_growth_guard_validation(growth):
    with pytest.raises(ValueError):
        ReplayGcExperiment('deferred',180,growth,'214',True)


def test_gc_failure_before_disable_preserves_policy(monkeypatch):
    import gc
    original=(gc.isenabled(),gc.get_threshold())
    def unavailable():
        raise OSError('RSS unavailable')
    monkeypatch.setattr(ReplayGcExperiment,'rss_bytes',staticmethod(unavailable))
    experiment=ReplayGcExperiment('deferred',180,512,'214',True)
    with pytest.raises(OSError):
        experiment.start()
    assert experiment.stop() is None
    assert original==(gc.isenabled(),gc.get_threshold())


def test_gc_observer_is_bounded_and_preserves_policy():
    import gc
    original = (gc.isenabled(), gc.get_threshold(), list(gc.callbacks))
    observer = GcTrace(capacity=2)
    observer.start()
    try:
        gc.collect(0)
        gc.collect(0)
    finally:
        report = observer.stop()
    assert len(report['events']) == 2 and report['dropped'] >= 2
    assert [row['phase'] for row in report['events']] == ['start', 'stop']
    assert report['events'][1]['monotonic_s'] >= report['events'][0]['monotonic_s']
    assert original == (gc.isenabled(), gc.get_threshold(), list(gc.callbacks))


def event(stamp, kind='encoder', value=0.):
    return ReplayEvent(stamp, stamp, kind, [value]*6)


def test_burst_owner_work_is_bounded_and_raw_reversals_are_retained():
    inputs = ShadowInputs()
    counts = Counter()
    values = [0., 1., 2., 1., 0., -1., 0., 1., 2.]
    for stamp, value in enumerate(values, 1):
        inputs.put(stamp, event(stamp, value=value), counts)
    processed = []
    while inputs.encoders:
        batch = inputs.take()
        assert len(batch) <= 4
        processed.extend(item[1].values[0] for item in batch)
    assert processed == values  # Including both reversal extrema, without smoothing.
    assert not inputs.failed


def test_latest_samples_do_not_queue_and_future_marker_is_deferred():
    inputs = ShadowInputs()
    inputs.schedule.next_allowed_s = 0.
    counts = Counter()
    for stamp in range(1, 101):
        inputs.put(stamp, event(stamp, 'marker'), counts)
        inputs.put(stamp, event(stamp, 'position'), counts)
    assert len(inputs.latest) == 2
    assert counts['input_superseded_marker'] == 99
    assert inputs.take_marker(0., 99) is None
    assert inputs.take_marker(0., 100)[1].source_ns == 100
    inputs.put(101, event(101, 'marker'), counts)
    assert inputs.take_marker(.02, 101) is None
    assert inputs.take_marker(.05, 101)[1].source_ns == 101
    assert inputs.take()[0][1].source_ns == 100


def test_encoder_loss_latches_inhibition_instead_of_silent_eviction():
    inputs = ShadowInputs()
    counts = Counter()
    for stamp in range(1, 131):
        inputs.put(stamp, event(stamp), counts)
    assert len(inputs.encoders) == 128
    assert inputs.encoders[0][1].source_ns == 1
    assert inputs.failed and counts['input_queue_overflow'] == 2
    inputs.take()
    assert inputs.failed


def test_out_of_order_input_cannot_replace_a_newer_measurement():
    inputs = ShadowInputs()
    counts = Counter()
    for kind in ('position', 'marker', 'encoder'):
        inputs.put(2, event(2, kind), counts)
        inputs.put(3, event(1, kind), counts)
        assert counts['nonmonotonic_input_'+kind] == 1
    assert inputs.take_marker(0., 0) is None
    assert all(item[1].source_ns == 2 for item in inputs.take())


def test_owner_publishes_fresh_snapshot_after_bounded_batch_and_inhibits_on_loss():
    inputs = ShadowInputs()
    processed = []
    engine = SimpleNamespace(
        counts=Counter(), latest_encoder_ns=None,
        runtime=SimpleNamespace(diagnostics=lambda: {'health': 'test'}),
        snapshot=lambda: 'independent snapshot')
    def feed(item):
        processed.append(item)
        if item.kind == 'encoder':
            engine.latest_encoder_ns = item.source_ns
    engine.feed = feed
    node = SimpleNamespace(pending=inputs, engine=engine,
        queue_lock=threading.Lock(), snapshot_lock=threading.Lock(),
        estimator_probe=SimpleNamespace(observe=lambda now: 0.),
        record=lambda *args: None)
    for stamp in range(1, 11):
        inputs.put(stamp, event(stamp), engine.counts)
    EngagementShadow.owner_callback(node)
    assert len(processed) == 4
    assert node.snapshot == 'independent snapshot'
    assert len(inputs.encoders) == 6
    inputs.failed = True
    EngagementShadow.owner_callback(node)
    assert node.snapshot is None


def test_diagnostic_traffic_never_enters_sensor_handoff():
    seen = []
    node = SimpleNamespace(engine=SimpleNamespace(feed=seen.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)))
    fields = ('planner_elapsed_ms', 'estimator_callback_latest_ms', 'marker_total_ms',
        'marker_rewind_ms', 'marker_correction_ms', 'marker_replay_ms',
        'marker_source_age_ms', 'planner_snapshot_age_ms', 'plan_valid', 'plan_reason')
    msg = SimpleNamespace(**dict.fromkeys(fields, 0),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
    EngagementShadow.baseline_callback(node, msg)
    assert seen[0].kind == 'plan'


def test_live_planning_defers_diagnostics():
    calls=[]
    node=SimpleNamespace(snapshot='snapshot',snapshot_lock=threading.Lock(),plans=[],
        engine=SimpleNamespace(plan_snapshot=lambda *args,**kw: calls.append((args,kw))),
        planner_probe=SimpleNamespace(observe=lambda now: 0.), record=lambda *args: None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)))
    EngagementShadow.plan_callback(node)
    assert calls==[(('snapshot',1),dict(defer_diagnostics=True))]


def test_process_preparation_runs_while_inactive_but_never_counts_live_plan():
    calls=[]
    node=SimpleNamespace(snapshot='snapshot',snapshot_lock=threading.Lock(),plans=[],
        follow_baseline=True,baseline_activity=SimpleNamespace(state='DISARMED'),
        engine=SimpleNamespace(compute_assignment={'planner_process':True},prepared=False,
            prepare_planner=lambda s:calls.append(s) or True),
        planner_probe=SimpleNamespace(observe=lambda now:0.),record=lambda *a:None)
    EngagementShadow.plan_callback(node)
    assert calls==['snapshot'] and not node.plans


def test_task_monitor_ignores_home_and_counts_each_attempt_once():
    monitor = BaselineActivity(2)
    monitor.observe(1, 'DISARMED', {})
    assert not monitor.finished
    for i in range(2):
        monitor.target()
        monitor.observe(2+i*3, 'READY', {})
        monitor.observe(3+i*3, 'ACTIVE', {})
        monitor.observe(4+i*3, 'DISARMED', {})
        monitor.observe(4+i*3, 'DISARMED', {})
    assert monitor.completed_targets == 2
    assert monitor.finished
    assert monitor.report()['completed_target_attempts'] == 2


def test_task_monitor_latches_fault_without_claiming_success():
    monitor = BaselineActivity(4)
    monitor.observe(1, 'FAULTED', {'reason': 'accepted_marker_stale'})
    monitor.observe(2, 'DISARMED', {})
    assert monitor.fault_seen and monitor.finished
    assert monitor.completed_targets == 0
    assert monitor.transitions[0]['reason'] == 'accepted_marker_stale'


def test_task_following_shadow_does_not_plan_during_home_or_unknown_state():
    for state in (None, 'DISARMED', 'READY', 'FAULTED'):
        node = SimpleNamespace(follow_baseline=True,
            baseline_activity=SimpleNamespace(state=state),
            engine=SimpleNamespace(counts=Counter()),
            planner_probe=SimpleNamespace(observe=lambda now: 0.),record=lambda *a: None)
        EngagementShadow.plan_callback(node)
        assert node.engine.counts['plan_baseline_inactive'] == 1
