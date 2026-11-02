"""Matched non-actuating model replay, using existing replay/planner owners.

No ROS node is constructed. Counterfactual commands are diagnostics, never
executed. This is compute/integration evidence, not closed-loop performance.
"""
import argparse
from collections import Counter
from contextlib import contextmanager
import copy
from dataclasses import asdict
import gc
import hashlib
import importlib
import json
from pathlib import Path
import time

import numpy as np
import torch

from cr_meta_lnn.deployment import load_runtime_bundle
from catheter_control.orchestration.planner_process import ProcessPlanner
from catheter_control.orchestration.task_gc import TaskGcPolicy
from catheter_control.safety.validation import percentile_metrics
from .compute_profile import GcTrace, Measurements, ReplayEvent, replay
from .engagement_qualification import EngagementQualification, finite_json


@contextmanager
def replay_gc(policy_name, evidence):
    """Reuse production GC ownership for one serial, non-commanding replay.

    Prefix/warmup and measured work share one bounded parent interval. No ROS
    arming or DISARMED status is synthesized. Worker GC remains unchanged.
    Collection/restoration finishes before report construction, even on errors.
    """
    if policy_name not in ('normal', 'task-boundary'):
        raise ValueError('unknown replay GC policy')
    policy = TaskGcPolicy(enabled=policy_name == 'task-boundary')
    trace = GcTrace()
    original = (gc.isenabled(), gc.get_threshold())
    next_check = 0.

    def check(_event):
        nonlocal next_check
        now = time.monotonic()
        if now >= next_check:
            next_check = now + 1.
            reason = policy.guard_reason()
            if reason:
                raise RuntimeError(reason)

    trace.start()
    try:
        # Same idle preparation in both arms, outside all measured solves.
        gc.collect(2)
        if not policy.begin():
            raise RuntimeError('replay GC maintenance pending')
        evidence['started_s'] = time.monotonic()
        with policy.work() as admitted:
            if not admitted:
                raise RuntimeError('replay GC work not admitted')
            yield check
            reason = policy.guard_reason()
            if reason:
                raise RuntimeError(reason)
    finally:
        evidence['work_finished_s'] = time.monotonic()
        evidence['active_summary'] = policy.summary()
        policy.stop()
        try:
            if policy.reserve_collection():
                policy.collect_reserved()
        finally:
            evidence.update(policy=policy_name, summary=policy.summary(),
                trace=trace.stop(), original_enabled=original[0],
                original_thresholds=original[1], restored_enabled=gc.isenabled(),
                restored_thresholds=gc.get_threshold(),
                scope='serial offline parent; causal prefix and measured window; no hardware')


def planner_evidence(planner):
    """Own request-local evidence before the next solve overwrites it."""
    return copy.deepcopy(dict(worker_transport=planner.worker_diagnostics(),
        process_timings_s=planner.process_timings_s,
        process_stage_marks=planner.process_stage_marks,
        process_trace=planner.process_trace))


def installed_identity(package_root, source_root):
    """Fail if source/active site-packages masks the staged wheel, or is stale."""
    root, source = Path(package_root).resolve(), Path(source_root).resolve()
    records = {}
    for name, source_package in (
            ('cr_meta_lnn.deployment.experimental.engagement_runtime', source/'cr_meta_lnn'),
            ('cr_meta_lnn.deployment.v171_streaming_runtime', source/'cr_meta_lnn'),
            ('cr_common.kinematics', source/'cr-common/cr_common')):
        path = Path(importlib.import_module(name).__file__).resolve()
        if not path.is_relative_to(root):
            raise RuntimeError(f'package isolation failed: {name} -> {path}')
        if not path.is_file():
            raise RuntimeError(f'installed module missing: {path}')
        package = name.split('.')[0]
        staged = root/package
        directories = ('', 'deployment', 'deployment/experimental', 'deployment/legacy',
                       'networks', 'networks/hybrid') if package == 'cr_meta_lnn' else ('',)
        for directory in directories:
            for expected in (source_package/directory).glob('*.py'):
                item=staged/expected.relative_to(source_package)
                if not item.is_file():
                    raise RuntimeError(f'wheel omitted supported module: {item}')
        hashes = {}
        for item in staged.rglob('*.py'):
            relative = item.relative_to(staged)
            expected = source_package/relative
            if not expected.is_file() or item.read_bytes() != expected.read_bytes():
                raise RuntimeError(f'wheel/source mismatch: {item}')
            hashes[str(relative)] = hashlib.sha256(item.read_bytes()).hexdigest()
        records[name] = dict(path=str(path), python_files=hashes)
    return records


def sensor_fixture(path, start_offset_s, duration_s):
    """Shared native input/order, recorded targets, 15 Hz paired solve receipts."""
    payload = json.loads(Path(path).read_text())
    if payload['schema'] != 'engagement_sensor_events_v1':
        raise ValueError('unknown sensor schema')
    rows = payload['events']
    if not rows or any(b['receipt_ns'] < a['receipt_ns'] for a,b in zip(rows, rows[1:])):
        raise ValueError('empty or nonmonotonic receipt sequence')
    first = rows[0]['receipt_ns']
    start = first+int(start_offset_s*1e9)
    end = start+int(duration_s*1e9)
    if end > rows[-1]['receipt_ns']:
        raise ValueError('measurement window extends beyond recording')
    events, targets = [], []
    next_plan = first
    last_encoder = last_marker = -10**30
    for row in rows:
        if row['receipt_ns'] > end:
            break
        event = ReplayEvent(**row)
        if event.kind == 'target':
            value = np.asarray(event.values, float)
            if value.shape != (3,) or not np.isfinite(value).all():
                raise ValueError('finite XYZ recorded target required')
            targets.append((event.receipt_ns,value))
        if event.kind not in ('encoder','marker','position','target'):
            continue
        # Same recurrence/correction input grid for both estimators. Preserve
        # acquisition and receipt time; neither state is initialized from the
        # other model or from future geometry.
        if event.kind == 'encoder':
            if event.source_ns-last_encoder < 20_000_000:
                continue
            last_encoder = event.source_ns
        if event.kind == 'marker':
            if event.receipt_ns-last_marker < 50_000_000:
                continue
            last_marker = event.receipt_ns
        events.append(event)
        if event.receipt_ns >= next_plan:
            events.append(ReplayEvent(event.receipt_ns,last_encoder,'plan',None,None))
            next_plan = event.receipt_ns+66_666_667
    if not targets or targets[0][0] > start:
        raise ValueError('window requires a recorded target before its start')
    return events, targets, start, end


def installed_ros_identity(source_root):
    """Verify the sourced overlay's core owners and record exact loaded bytes."""
    repo=Path(source_root).resolve()/'robot-infra'
    records={}
    for name in ('catheter_control.node','catheter_control.planning.mppi',
                 'catheter_control.planning.engagement',
                 'catheter_control.orchestration.planner_process',
                 'catheter_control.orchestration.task_gc',
                 'catheter_control.transmission.backlash',
                 'runtime_supervision.compute_profile',
                 'runtime_supervision.engagement_qualification',
                 'runtime_supervision.qualify_engagement_integration'):
        path=Path(importlib.import_module(name).__file__).absolute()
        if not any(path.is_relative_to(repo/d) for d in ('build','install')):
            raise RuntimeError(f'ROS overlay isolation failed: {name} -> {path}')
        resolved=path.resolve()
        if not resolved.is_relative_to(repo/'src'):
            raise RuntimeError(f'ROS symlink/source identity failed: {name} -> {resolved}')
        records[name]=dict(loaded_path=str(path),resolved_path=str(resolved),
            sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return records


def verdict(pairs, corrections):
    failures = []
    if not pairs:
        failures.append('no_matched_plans')
    if not corrections or not any(row['accepted'] for row in corrections):
        failures.append('no_baseline_marker_correction')
    if any(not row['accepted'] for row in corrections):
        failures.append('baseline_marker_rejection')
    for family in ('baseline','experimental'):
        if any(row[family] is None for row in pairs):
            failures.append(f'{family}_unavailable_at_matched_receipt')
        if any(row[family] is not None and not row[family]['valid'] for row in pairs):
            failures.append(f'{family}_invalid_plan')
    return failures


def run(args):
    torch.set_num_threads(4)
    torch.set_num_interop_threads(1)
    if args.device == 'cuda' and not torch.cuda.is_available():
        raise RuntimeError('CUDA requested but unavailable; no CPU fallback')
    identity = installed_identity(args.package_root,args.source_root)
    ros_identity = installed_ros_identity(args.source_root)
    args.output.mkdir(parents=True,exist_ok=False)
    events,targets,start,end = sensor_fixture(args.events,args.start_offset_s,args.duration_s)
    experimental = EngagementQualification(args.manifest,args.limits,
        samples=args.samples,device=args.device,planner_process=True)
    baseline = None
    pairs, cold = [], []
    meter = Measurements()
    gc_evidence = {}
    try:
        bundle = load_runtime_bundle(args.baseline_manifest,device='cpu',
            options={'marker_estimator':'ukf','adaptation_enabled':False})
        config = experimental.config
        baseline = ProcessPlanner(bundle.runtime,experimental.contract,config,
            manifest=str(args.baseline_manifest),device=args.device,
            options={'marker_estimator':'ukf','adaptation_enabled':False},
            identity=bundle.identity.manifest_sha256)
        prepared = False

        def solve(root,position,target,event,measurements):
            nonlocal prepared
            snapshot = experimental.snapshot()
            if snapshot is not None:
                state,joints,stamp,_,belief = snapshot
                snapshot = (state,joints,stamp,np.asarray(target).copy(),belief)
            if not prepared and snapshot is not None:
                # Lazy allocation/transfer is discarded, preserving solver
                # memory. Cold deadline misses stay visible, not executable.
                saved = baseline.execution_memory()
                warm = baseline.plan(root,position,target)
                baseline.restore_execution_memory(saved)
                if warm.reason in ('worker_unavailable','worker_error','worker_state_changed'):
                    raise RuntimeError(f'baseline preparation failed: {warm.reason}')
                experimental.prepare_planner(snapshot)
                cold.append(dict(receipt_ns=event.receipt_ns,
                    baseline_reason=warm.reason,elapsed_ms=warm.elapsed_s*1000,
                    discarded=True,executed=False))
                prepared = True
            if event.receipt_ns < start:
                return baseline._safe_result(baseline._clock(),'prefix_not_measured')
            # Matched frozen proposal policy: seed/nominal reset for each paired
            # solve, never reset estimator or physical transmission history.
            baseline.reset(); experimental.planner.reset()
            belief = None if snapshot is None else snapshot[-1]
            a_started = time.monotonic()
            a = measurements.call('planner',event.source_ns,baseline.plan,
                root,position,target,transmission_state=belief)
            a_finished = time.monotonic()
            a_evidence = planner_evidence(baseline)
            b_started = time.monotonic()
            b = experimental.plan_snapshot(snapshot,event.receipt_ns,defer_diagnostics=True)
            b_finished = time.monotonic()
            if b is not None:
                b.update(planner_evidence(experimental.planner),
                    call_started_s=b_started,call_finished_s=b_finished)
            pairs.append(dict(receipt_ns=event.receipt_ns,source_ns=event.source_ns,
                position=np.asarray(position).tolist(),target_m=np.asarray(target).tolist(),
                baseline=dict(valid=a.valid,reason=a.reason,elapsed_ms=a.elapsed_s*1000,
                    call_started_s=a_started,call_finished_s=a_finished,**a_evidence,
                    command=a.command_logical_velocity.tolist(),
                    tip_sequence_m=None if a.command_tip_sequence_m is None else a.command_tip_sequence_m.tolist()),
                experimental=b))
            if len(pairs)%15 == 0:
                print(json.dumps(dict(stage='matched_replay',pairs=len(pairs),
                    recorded_window_s=(event.receipt_ns-start)*1e-9)),flush=True)
            return a

        with replay_gc(args.gc_policy, gc_evidence) as check_gc:
            def feed(event):
                check_gc(event)
                if event.kind != 'plan':
                    experimental.feed(event)
            _,corrections = replay(bundle.runtime,experimental.contract,config,events,targets,
                start_ns=start,end_ns=end,measured=meter,
                before_event=feed,plan_function=solve)
        failures = verdict(pairs,corrections)
        if gc_evidence['trace']['dropped']:
            failures.append('gc_trace_incomplete')
        if (gc_evidence['restored_enabled'] != gc_evidence['original_enabled']
                or gc_evidence['restored_thresholds'] != gc_evidence['original_thresholds']):
            failures.append('gc_policy_not_restored')
        if len(pairs) < max(1,int(args.duration_s*15*.8)):
            failures.append('insufficient_matched_plan_coverage')
        if not experimental.counts['accepted_markers']:
            failures.append('no_experimental_marker_correction')
        if experimental.counts['rejected_markers'] or experimental.counts['marker_queue_overflow']:
            failures.append('experimental_marker_rejection_or_overflow')
        if (experimental.runtime.estimator is None
                or not experimental.runtime.estimator.external_confirmation):
            failures.append('shared_confirmation_owner_missing')
        summaries = {}
        for family in ('baseline','experimental'):
            rows = [p[family] for p in pairs if p[family] is not None]
            summaries[family] = dict(count=len(rows),
                timing_ms=None if not rows else percentile_metrics([p['elapsed_ms'] for p in rows]),
                deadline_misses=sum(p['reason']=='deadline_missed' for p in rows),
                invalid_plans=sum(not p['valid'] for p in rows),
                reason_counts=dict(Counter(p['reason'] for p in rows)))
        report = dict(schema='engagement_integration_replay_v1',passed=not failures,
            failures=failures,executed_commands=0,package_identity=identity,
            ros_identity=ros_identity,
            baseline_identity=asdict(bundle.identity),
            experimental_identity=asdict(experimental.bundle.identity),
            events_sha256=hashlib.sha256(Path(args.events).read_bytes()).hexdigest(),
            scope='sequential matched counterfactual solves; not ROS scheduling or closed loop',
            matching='same sensor grid, receipt/target/joints/seed/config/projection; common experimental-derived transmission snapshot isolates distal/belief state effects',
            assignment={'estimator':'cpu','planner':args.device,'isolated_worker':True},
            start_offset_s=args.start_offset_s,duration_s=args.duration_s,
            config=asdict(config),cold_discarded=cold,summary=summaries,
            gc=gc_evidence,
            diagnostic_retention='complete offline rows; no eviction; task-GC RSS/time guards bound deferred work',
            plan_coverage=dict(actual=len(pairs),nominal=args.duration_s*15,
                minimum=max(1,int(args.duration_s*15*.8))),
            baseline_timing=meter.summaries(),baseline_corrections=corrections,
            experimental_prefix_and_window_timing=experimental.measurements.summaries(),
            experimental_counts=dict(experimental.counts),
            confirmation_policy='shared_baseline_bending_coordinate',
            distal_input_prior=experimental.runtime.diagnostics()['distal_input_prior'],
            confirmation_config=asdict(experimental.interface.config),
            experimental_observed_phase_counts=dict(Counter(row['phase'] for row in experimental.trace)),
            pairs=pairs)
        (args.output/'report.json').write_text(json.dumps(finite_json(report),indent=2,allow_nan=False)+'\n')
        print(json.dumps(dict(report=str(args.output/'report.json'),passed=report['passed'])),flush=True)
        return 0 if report['passed'] else 1
    finally:
        if baseline is not None:
            baseline.close()
        experimental.close()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('manifest','baseline-manifest','events','limits','package-root','source-root','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--device',choices=('cpu','cuda'),default='cuda')
    parser.add_argument('--samples',type=int,default=512)
    parser.add_argument('--start-offset-s',type=float,required=True)
    parser.add_argument('--duration-s',type=float,default=4.)
    parser.add_argument('--gc-policy',choices=('normal','task-boundary'),default='task-boundary',
        help='Shared bounded parent GC policy for replay only; normal is the diagnostic control arm')
    args=parser.parse_args(argv)
    if (args.samples<8 or not np.isfinite([args.start_offset_s,args.duration_s]).all()
            or args.start_offset_s<0 or not 0<args.duration_s<=30):
        parser.error('samples>=8, finite nonnegative offset and duration in (0,30] required')
    raise SystemExit(run(args))


if __name__ == '__main__':
    main()
