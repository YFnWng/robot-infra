"""Source-time G4 observer core shared by recorded replay and ROS shadow.

No publisher, service client, arm/mode API, or device command exists here.
The independent joint estimator owns its history; recorded baseline latents
are never used to initialize it. Offline wall durations are not live scheduling.
"""
from collections import Counter, deque
import copy
from dataclasses import asdict, replace
from pathlib import Path
import csv
import json
import time

import numpy as np

from cr_meta_lnn.deployment import load_runtime_bundle
from cr_meta_lnn.deployment.experimental.engagement_rollout_check import compare_rollout_steps
from catheter_control.planning.mppi import CatheterMppi, MppiConfig
from catheter_control.planning.engagement import (reconcile_engagement, tendon_confirmation,
    confirmation_from_checkpoint, confirm_at_source)
from cr_meta_lnn.deployment.v171_streaming_runtime import ENCODER_RADIANS_PER_COUNT
from catheter_control.safety.hardware_contract import load_hardware_contract
from catheter_control.safety.validation import percentile_metrics
from catheter_control.transmission.backlash import (
    BacklashConfig, BacklashStateEstimator, BacklashFeedforwardCompensator, TakeupTransactionArbiter)
from .compute_profile import Measurements, ReplayEvent, read_events


def marker_quality(raw, std_m):
    """Uncertainty is an explicit trial assumption, not confidence-as-variance."""
    if raw is None or any(key not in raw or np.asarray(raw[key]).shape != (4,) for key in (
            'confidence', 'source_rig_count', 'reprojection_error_px')):
        raise ValueError('four marker quality values required')
    visible = ((np.asarray(raw['confidence']) > 0)
               & (np.asarray(raw['source_rig_count']) >= 1)
               & (np.asarray(raw['reprojection_error_px']) >= 0))
    if visible.sum() < 3:
        raise ValueError('insufficient supported markers')
    return dict(visible=visible, point_std_m=np.full(4, std_m))


class EngagementQualification:
    """Single mutable estimator owner; planning uses independent deep snapshots."""

    def __init__(self, manifest, limits, *, catheter='imricor_test', device='cpu',
                 samples=512, grouped=True, marker_std_m=.0005, planner_process=False):
        if not np.isfinite(marker_std_m) or marker_std_m <= 0:
            raise ValueError('positive marker uncertainty in metres required')
        self.bundle = load_runtime_bundle(manifest, device='cpu' if planner_process else device)
        self.runtime = self.bundle.runtime
        self.runtime.use_external_tendon_confirmation()
        if self.runtime.command_capable is not False or self.runtime.proximal is None:
            raise ValueError('G4 requires the non-commanding G2/G3 manifest')
        self.marker_std_m = marker_std_m
        contract = load_hardware_contract(limits, catheter)
        minimum, maximum = contract.velocity_min.copy(), contract.velocity_max.copy()
        minimum[1] = maximum[1] = 0.
        self.contract = replace(contract, velocity_min=minimum, velocity_max=maximum)
        self.config = MppiConfig(samples=samples, horizon_steps=4, step_s=.04,
            point_rollout_step_s=.2, point_rollout_coarse_steps=True,
            grouped_mode_sampling=grouped, planning_deadline_s=.060,
            capture_radius_mm=2.5, capture_minimum_terminal_improvement_mm=.25,
            capture_hold_s=.3)
        self.planner = CatheterMppi(self.runtime, self.contract, self.config)
        if planner_process:
            from catheter_control.orchestration.planner_process import ProcessPlanner
            self.planner = ProcessPlanner(self.runtime, self.contract, self.config,
                manifest=str(manifest),device=device,options={},
                identity=self.bundle.identity.manifest_sha256)
        self.compute_assignment = dict(estimator_device=str(self.runtime.device),
            planner_device=device,planner_process=planner_process)
        self.prepared = False
        p = self.runtime.proximal.transmission
        widths = ((p.positive_play+p.negative_play)/p.axis_scale.abs()).cpu().numpy()
        config = BacklashConfig(tuple(widths), takeup_velocity=(1.,2.5,.5),
            engagement_confirmation_observations=3)
        # One shared baseline observation policy for every shaft. In this
        # non-commanding replay, motion ownership is a counterfactual assumption.
        self.interface = BacklashStateEstimator(config)
        self.confirmation_history = deque(maxlen=512)
        self.arbiter = TakeupTransactionArbiter(BacklashFeedforwardCompensator(config))
        self.measurements = Measurements()
        self.counts = Counter()
        self.encoders = deque(maxlen=512)
        self.pending_markers = deque(maxlen=64)
        self.latest_encoder_ns = None
        self.position = self.position_ns = None
        self.last_pose = None
        self.target = None
        self.target_source = 'home_plus_5mm_timing_probe'
        self.trace = []
        self.first_observed_source_ns = None
        self.worker_trace = []
        self.baseline = []
        self.baseline_shapes = []
        self.coarse_fine_checked = False
        self.deferred_rollout_check = None
        self.post_run_rollout_check = None
        self.started = time.monotonic()

    def bound_diagnostics(self):
        """ROS observer only; offline analysis and model history stay unchanged."""
        from .compute_profile import DiagnosticRecords
        if self.measurements.rows or any(getattr(self, name) for name in
                ('trace','worker_trace','baseline','baseline_shapes')):
            raise ValueError('bound diagnostics before feeding any input')
        self.measurements.rows = DiagnosticRecords(capacity=65536)
        for name in ('trace','worker_trace','baseline','baseline_shapes'):
            setattr(self, name, DiagnosticRecords())

    def feed(self, event):
        """Events enter in receipt order, not sorted by source time."""
        self.counts[event.kind] += 1
        self.measurements.rows.append(dict(stage=event.kind+'_source_to_receipt_age',source_ns=event.source_ns,
            wall_ms=(event.receipt_ns-event.source_ns)/1e6,thread_cpu_ms=None))
        if event.kind == 'baseline_shape':
            self.baseline_shapes.append(dict(source_ns=event.source_ns, **event.values))
            return
        if event.kind == 'target':
            target = np.asarray(event.values, dtype=float)
            if target.shape != (3,) or not np.isfinite(target).all():
                raise ValueError('invalid recorded target')
            self.target = target.copy()
            self.target_source = 'recorded_target'
            return
        if event.kind == 'plan':
            if event.values is not None:
                self.baseline.append(dict(source_ns=event.source_ns, **event.values))
            return
        if event.kind == 'position':
            values = np.asarray(event.values, dtype=float)
            if values.shape != (6,) or not np.isfinite(values).all():
                raise ValueError('invalid six-axis position')
            self.position = values.copy()
            self.position_ns = event.source_ns
            return
        if event.kind == 'marker':
            if len(self.pending_markers) == self.pending_markers.maxlen:
                self.counts['marker_queue_overflow'] += 1
            self.pending_markers.append(event)
        elif event.kind == 'encoder':
            values = np.asarray(event.values, dtype=float)
            if values.shape != (6,) or not np.isfinite(values).all():
                raise ValueError('invalid six-axis encoder')
            if self.latest_encoder_ns is not None and event.source_ns <= self.latest_encoder_ns:
                self.counts['nonmonotonic_encoder'] += 1
                return
            self.encoders.append((event.source_ns, np.asarray(event.values).copy()))
            self.latest_encoder_ns = event.source_ns
            raw = np.asarray(event.values)[:3]*ENCODER_RADIANS_PER_COUNT
            self.interface.advance_motor(raw)
            self.confirmation_history.append([event.source_ns,raw,self.interface.clone_state(),True])
            if self.runtime.estimator is not None:
                self.measurements.call('encoder_prediction', event.source_ns,
                    self.runtime.advance_encoder, event.source_ns, event.values,
                    return_snapshot=False,
                    tendon_confirmation=tendon_confirmation(self.runtime,self.interface.snapshot()))
        self._drain_markers()

    def _drain_markers(self):
        while self.pending_markers and self.latest_encoder_ns is not None:
            event = self.pending_markers[0]
            if event.source_ns > self.latest_encoder_ns:
                self.counts['marker_deferred'] += 1
                break
            self.pending_markers.popleft()
            try:
                quality = marker_quality(event.quality, self.marker_std_m)
                if self.runtime.estimator is None:
                    earlier = [item for item in self.encoders if item[0] <= event.source_ns]
                    later = [item for item in self.encoders if item[0] >= event.source_ns]
                    if not earlier or not later:
                        self.counts['initialization_unbracketed'] += 1
                        continue
                    a, b = earlier[-1], later[0]
                    if b[0]-a[0] > 250_000_000:
                        raise ValueError('initialization_encoder_gap')
                    counts = a[1] if a[0] == b[0] else a[1]+(event.source_ns-a[0])/(b[0]-a[0])*(b[1]-a[1])
                    self.measurements.call('spatial_initialization', event.source_ns,
                        self.runtime.initialize, event.source_ns, counts, event.values, quality=quality,
                        tendon_confirmation=confirmation_from_checkpoint(self.runtime,
                            next(entry[2] for entry in reversed(self.confirmation_history) if entry[0]<=event.source_ns)))
                    for stamp, encoder in self.encoders:
                        if stamp > event.source_ns:
                            self.measurements.call('encoder_prediction', stamp,
                                self.runtime.advance_encoder, stamp, encoder,
                                return_snapshot=False,
                                tendon_confirmation=confirmation_from_checkpoint(self.runtime,
                                    next(entry[2] for entry in reversed(self.confirmation_history) if entry[0]<=stamp)))
                    correction = None
                else:
                    correction = self.measurements.call('joint_correction_rewind_replay', event.source_ns,
                        self.runtime.observe_markers, event.source_ns, event.values, quality,
                        confirmation_observer=lambda estimator:self._confirm_at_source(event,estimator))
                    for stage, milliseconds in self.runtime.estimator.last_update_timing_ms.items():
                        self.measurements.rows.append(dict(stage='joint_'+stage,
                            source_ns=event.source_ns,wall_ms=milliseconds,thread_cpu_ms=None))
                state = self.measurements.call('snapshot_transfer', event.source_ns,
                    self.runtime.clone_state_at_or_before, event.source_ns,
                    include_history=False)
                if state is None:
                    raise ValueError('source_snapshot_missing')
                pose = state.estimator.poses(state.mean[None])[0]
                # Joint posterior orientation with source-time marker-0
                # translation, matching the production response observer.
                self.last_pose = pose
                observed = np.asarray(event.values)
                predicted = self.runtime.markers_for_state(state)
                if self.target is None:
                    self.target = observed[-1].copy()+np.array([0.,0.,.005])
                if self.first_observed_source_ns is None:
                    self.first_observed_source_ns = event.source_ns
                self.trace.append(dict(source_ns=event.source_ns, receipt_ns=event.receipt_ns,
                    phase=state.estimator.phase, direction=state.estimator.direction,
                    raw_tendon_motor_mm=float(state.estimator.raw_encoder),
                    u=float(state.mean[30]), u_std=float(np.sqrt(state.covariance[30,30])),
                    pose_covariance_diagonal=np.diag(state.covariance)[24:30].tolist(),
                    marker_max_error_mm=float(1000*np.max(np.linalg.norm(predicted[quality['visible']]-observed[quality['visible']],axis=1))),
                    predicted_markers_m=predicted.tolist(), observed_markers_m=observed.tolist(),
                    confirmation=dict(policy='baseline_bending_coordinate',
                        response=next(entry[2]['distal_bending_increment'] for entry in reversed(self.confirmation_history)
                                      if entry[0]<=event.source_ns),
                        threshold=self.interface.config.minimum_distal_bending_increment,
                        observations_required=self.interface.config.engagement_confirmation_observations),
                    correction=correction))
                self.counts['accepted_markers'] += 1
            except ValueError as error:
                self.counts['rejected_markers'] += 1
                self.counts['rejection:'+str(error)] += 1

    def _confirm_at_source(self, event, estimator):
        _,history,decisions = confirm_at_source(self.runtime,self.interface,
            self.confirmation_history,event.source_ns,estimator,np.asarray(event.values),control_owned=True)
        self.confirmation_history = deque(history,maxlen=512)
        return decisions

    def snapshot(self):
        if self.runtime.estimator is None or self.position is None:
            return None
        state = self.measurements.call('planner_snapshot_clone', self.latest_encoder_ns,
            self.runtime.clone_state, include_history=False)
        return state, self.position.copy(), self.position_ns, self.target.copy(), reconcile_engagement(
            self.runtime, self.interface.snapshot(), backlash_config=self.interface.config)

    def plan_snapshot(self, snapshot, now_ns, *, defer_diagnostics=False):
        if snapshot is None:
            self.counts['plan_uninitialized'] += 1
            return None
        state, position, position_ns, target, belief = snapshot
        self.measurements.rows.append(dict(stage='planner_snapshot_age',source_ns=now_ns,
            wall_ms=(now_ns-int(state.estimator.timestamp_s*1e9))/1e6,thread_cpu_ms=None))
        last = state.estimator.last_correction_s
        if last is not None:
            self.measurements.rows.append(dict(stage='planner_correction_age',source_ns=now_ns,
                wall_ms=(now_ns-int(last*1e9))/1e6,thread_cpu_ms=None))
        stale_reasons = []
        if last is None or now_ns-int(last*1e9) > 150_000_000:
            stale_reasons.append('correction_age')
        if now_ns-position_ns > 150_000_000:
            stale_reasons.append('position_age')
        if now_ns < position_ns:
            stale_reasons.append('position_from_future')
        if stale_reasons:
            self.counts['plan_stale'] += 1
            for reason in stale_reasons:
                self.counts['plan_stale:'+reason] += 1
            return None
        if not state.estimator.prediction_ready:
            self.counts['plan_invalid_posterior'] += 1
            return None
        result = self.measurements.call('planner', now_ns, self.planner.plan,
            state, position, target, transmission_state=belief)
        for stage, seconds in getattr(self.planner,'process_timings_s',{}).items():
            self.measurements.rows.append(dict(stage=stage,source_ns=now_ns,
                wall_ms=seconds*1000,thread_cpu_ms=None))
        for trace in getattr(self.planner, 'process_trace', []):
            self.worker_trace.append(dict(source_ns=now_ns, **trace))
        self.counts['plans'] += 1
        self.counts['deadline_misses'] += int(result.reason == 'deadline_missed')
        self.counts['invalid_plans'] += int(not result.valid)
        rates = result.motor_radians_per_second_sequence[:1,:3]
        check = None
        if result.valid and not self.coarse_fine_checked and self.deferred_rollout_check is None:
            if defer_diagnostics:
                # The exchanged state is already an independent, history-free
                # snapshot. Retain one job, never a backlog of diagnostic work.
                self.deferred_rollout_check = (now_ns, state, rates.copy())
            else:
                check = self.measurements.call('coarse_fine_check', now_ns,
                    compare_rollout_steps, self.runtime, state, rates)
                self.coarse_fine_checked = True
        # Deliberately return diagnostics only. No command is forwarded.
        # A counterfactual intent was NOT executed. Never persist its transaction
        # into the next observed snapshot or infer its engagement from baseline motion.
        decision=copy.deepcopy(self.arbiter).begin(
            result.command_logical_velocity if result.valid else np.zeros(6),position,self.contract,belief)
        return dict(source_ns=now_ns, reason=result.reason, valid=result.valid,
            target_m=target.tolist(), target_source=self.target_source,
            elapsed_ms=result.elapsed_s*1000,rollout_ms=result.rollout_ms,
            sample_projection_ms=result.sample_projection_ms,
            cost_weighting_ms=result.cost_weighting_ms,
            update_projection_ms=result.update_projection_ms,
            best_tip_sequence_m=(None if result.best_tip_sequence_m is None
                                 else result.best_tip_sequence_m.tolist()),
            passive_and_coarse_fine=check, tendon_phase=state.estimator.phase,
            shadow_transaction=decision.reason,
            worker_transport=getattr(self.planner,'worker_diagnostics',lambda:None)(),
            command_output_enabled=False)

    def prepare_planner(self, snapshot):
        """Discard one bounded cold solve without consuming policy/RNG history.

        Called outside the estimator owner, using an independent observed state.
        A cold deadline result is evidence of preparation, not executable work.
        Worker failures are never treated as readiness or replaced by local code.
        """
        if snapshot is None or self.prepared:
            return self.prepared
        state, position, position_ns, target, belief = snapshot
        memory = self.planner.execution_memory()
        try:
            result = self.measurements.call('planner_preparation',position_ns,
                self.planner.plan,state,position,target,transmission_state=belief)
        finally:
            self.planner.restore_execution_memory(memory)
        if result.reason not in ('ok','deadline_missed'):
            raise RuntimeError('shadow planner preparation failed: '+result.reason)
        self.prepared = True
        return True

    def close(self):
        close = getattr(self.planner,'close',None)
        if close is not None:
            close()

    def finish_deferred_diagnostics(self):
        """Run only after live executor shutdown; offline replay stays unchanged."""
        job = self.deferred_rollout_check
        if job is None:
            return
        self.deferred_rollout_check = None
        stamp, state, rates = job
        try:
            check = self.measurements.call('post_run_coarse_fine_check', stamp,
                compare_rollout_steps, self.runtime, state, rates)
            self.post_run_rollout_check = dict(source_ns=stamp, result=check,
                scope='after_shadow_executor_shutdown')
            self.coarse_fine_checked = True
        except Exception as error:
            # Keep the acquired timing evidence even if an optional check fails.
            self.counts['diagnostic_failures'] += 1
            self.post_run_rollout_check = dict(source_ns=stamp,
                scope='after_shadow_executor_shutdown', error=str(error))

    def report(self, path, *, scope, plans=(), live_context=None):
        path = Path(path)
        path.mkdir(parents=True, exist_ok=False)
        # Executor is already stopped. Rehydrate once, outside the measured
        # loop; distributions/plots explicitly describe retained samples.
        stores = dict(timing=self.measurements.rows, plans=plans,
            **{name:getattr(self,name) for name in
               ('trace','worker_trace','baseline','baseline_shapes')})
        retention = {name:store.retention() for name,store in stores.items()
                     if hasattr(store,'retention')}
        rows = list(self.measurements.rows)
        for name in ('trace','worker_trace','baseline','baseline_shapes'):
            setattr(self,name,list(getattr(self,name)))
        with (path/'timing.csv').open('w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=['stage','source_ns','wall_ms','thread_cpu_ms'])
            writer.writeheader(); writer.writerows(rows)
        timing={}
        for stage in sorted({row['stage'] for row in rows}):
            group=[row for row in rows if row['stage']==stage]
            cpu=[row['thread_cpu_ms'] for row in group if row['thread_cpu_ms'] is not None]
            wall=[row['wall_ms'] for row in group]
            timing[stage]=dict(count=len(group),wall=percentile_metrics(wall),
                thread_cpu=None if not cpu else percentile_metrics(cpu),
                deadline_misses=sum(v>60. for v in wall) if stage in ('planner','planner_callback') else None,
                period_overruns=sum(v>20. for v in wall) if stage=='estimator_callback' else None)
        baseline_timing={}
        for key in sorted({key for row in self.baseline for key in row if key.endswith('_ms')}):
            values=[row[key] for row in self.baseline if row.get(key) is not None and np.isfinite(row[key])]
            if values:baseline_timing[key]=percentile_metrics(values)
        # A cold discarded rollout is emitted by the baseline's trace topic.
        # Missing lifecycle information remains unknown (older replay exports).
        cold=[row for row in self.baseline if row.get('controller_reason')=='warming_planner']
        warmed=[row for row in self.baseline if row.get('controller_reason') is not None
                and row.get('controller_reason')!='warming_planner']
        valid_times=[row['planner_elapsed_ms'] for row in warmed if row['plan_valid']]
        baseline_execution=dict(cold_rows=len(cold),warmed_rows=len(warmed),
            unclassified_rows=len(self.baseline)-len(cold)-len(warmed),
            warmed_deadline_misses=sum(row['plan_reason']=='deadline_missed' for row in warmed),
            warmed_invalid_plans=sum(not row['plan_valid'] for row in warmed),
            valid_planner_elapsed_ms=None if not valid_times else percentile_metrics(valid_times))
        matching={row['source_ns']:row for row in self.baseline_shapes}
        comparison=[]
        for row in self.trace:
            reference=matching.get(row['source_ns'])
            if reference is None:continue
            baseline_error=1000*np.linalg.norm(np.asarray(reference['estimated_tip_m'])-reference['observed_tip_m'])
            experimental_error=1000*np.linalg.norm(np.asarray(row['predicted_markers_m'])[-1]-np.asarray(row['observed_markers_m'])[-1])
            comparison.append(dict(source_ns=row['source_ns'],baseline_tip_error_mm=float(baseline_error),
                experimental_tip_error_mm=float(experimental_error),baseline_phase=reference['phase'],experimental_phase=row['phase']))
        errors = [row['marker_max_error_mm'] for row in self.trace]
        residual_summary = None if not errors else dict(count=len(errors),
            median_mm=float(np.median(errors)),p95_mm=float(np.percentile(errors,95)),
            max_mm=float(np.max(errors)))
        report = dict(schema='engagement_g4_v1', scope=scope,
            confirmation_policy='shared_baseline_bending_coordinate',
            confirmation_config=asdict(self.interface.config),
            confirmation_ownership='counterfactual owned motion; no actuation',
            runtime_identity=asdict(self.bundle.identity), command_capable=False,
            compute_assignment=self.compute_assignment,
            configuration=asdict(self.config), marker_std_m=self.marker_std_m,
            counts=dict(self.counts), timing=timing,baseline_timing_summary=baseline_timing,
            diagnostic_retention=retention,
            diagnostic_evidence_complete=all(not meta['dropped'] for meta in retention.values()),
            first_observed_source_ns=self.first_observed_source_ns,
            timing_scope='retained diagnostic samples; counts are lifetime counters',
            baseline_execution=baseline_execution,live_context=live_context,
            target_source=self.target_source,
            posterior_marker_max_error=residual_summary,
            observed_phase_counts=dict(Counter(row['phase'] for row in self.trace)),
            same_source_comparison=comparison,
            post_run_rollout_check=self.post_run_rollout_check,
            qualification='PASS_COMPONENTS_ONLY' if self.counts['accepted_markers'] and self.counts['plans']
                and not self.counts['invalid_plans'] and not self.counts['rejected_markers']
                and not self.counts['input_queue_overflow']
                and not self.counts['callback_exceptions']
                and not self.counts['diagnostic_failures'] else 'INCOMPLETE_OR_FAILED',
            full_stack_qualification='requires representative tracking/recording load and baseline traces',
            elapsed_wall_s=time.monotonic()-self.started, baseline_timing=self.baseline,
            trace=self.trace, worker_trace=self.worker_trace, plans=list(plans))
        report['worker_timing_summary'] = {
            key: percentile_metrics([row[key] * scale for row in self.worker_trace
                                     if key in row])
            for key, scale in (('root_transfer_wall_s', 1000),
                               ('root_transfer_cpu_s', 1000), ('solve_wall_s', 1000),
                               ('solve_cpu_s', 1000), ('decode_wall_s', 1000),
                               ('decode_cpu_s', 1000), ('request_to_worker_ms', 1),
                               ('reply_ready_to_decoded_ms', 1))
            if any(key in row for row in self.worker_trace)}
        (path/'report.json').write_text(json.dumps(finite_json(report), indent=2, allow_nan=False)+'\n')
        if self.trace:
            self.write_engagement_plot(path/'engagement.png')
        return path/'report.json'

    def write_engagement_plot(self, path):
        """Plot the actual shared confirmation evidence and its declared units."""
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg

        times = (np.array([row['source_ns'] for row in self.trace], dtype=np.int64)
                 - self.trace[0]['source_ns']) * 1e-9
        motor = [row['raw_tendon_motor_mm'] for row in self.trace]
        responses, thresholds = [], []
        for row in self.trace:
            evidence = row['confirmation']
            responses.append(abs(evidence['response']))
            thresholds.append(evidence['threshold'])
        modes = {'UNKNOWN': 0, 'TAKEUP': 1, 'PROVISIONAL': 2, 'ENGAGED': 3, 'FAILED': 4}
        figure = Figure(figsize=(11, 8), constrained_layout=True)
        FigureCanvasAgg(figure)
        axes = figure.subplots(3, 1, sharex=True)
        axes[0].plot(times, motor)
        axes[0].set_ylabel('Raw tendon motor (mm)')
        axes[1].plot(times, responses, label='Marker-corrected bending response')
        axes[1].plot(times, thresholds, label='Confirmation threshold')
        axes[1].set_ylabel('Bending-mode strain coordinate')
        axes[1].legend()
        axes[2].step(times, [modes[row['phase']] for row in self.trace], where='post')
        axes[2].set_yticks(list(modes.values()), list(modes))
        axes[2].set_xlabel('Time since first accepted marker (s)')
        for axis in axes:
            axis.grid(alpha=.25)
        figure.suptitle('Shared baseline tendon confirmation (no command output)')
        figure.savefig(path, dpi=160)


def prepare_events(session, output, prefix=''):
    """Keep native encoder and marker events, source AND receipt timestamps."""
    events = read_events(Path(session), prefix, require_plan=False, include_comparison=True)
    destination = Path(output)
    if destination.exists():
        raise ValueError('refusing to overwrite prepared replay')
    payload = [dict(receipt_ns=int(e.receipt_ns), source_ns=int(e.source_ns), kind=e.kind,
        values=None if e.values is None else e.values.tolist() if isinstance(e.values,np.ndarray) else e.values,
        quality=None if e.quality is None else {key:np.asarray(value).tolist() for key,value in e.quality.items()}) for e in events]
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(finite_json(dict(schema='engagement_sensor_events_v1',
        session=str(Path(session).resolve()), events=payload)), allow_nan=False)+'\n')
    return destination


def finite_json(value):
    """Convert NumPy diagnostics to JSON types; undefined values are null."""
    if isinstance(value, np.ndarray):
        return finite_json(value.tolist())
    if isinstance(value, np.generic):
        return finite_json(value.item())
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def compare_model_candidates(arms, candidates, *, planner_inputs=None):
    """Offline matched action bank on independently owned, native model roots.

    Each arm is (engine, snapshot). Do not rebind cross-model snapshots. Keep
    the task, hardware contract and scoring configuration identical. This
    reports model-sensitive ordering, not unobserved counterfactual outcomes.
    """
    from .replay_takeup_startup import FixedCandidates
    if len(arms) < 2:
        raise ValueError('model comparison needs at least two arms')
    candidates = np.asarray(candidates, dtype=float)
    if not np.isfinite(candidates).all():
        raise ValueError('nonfinite candidate bank')
    if set(planner_inputs or {})-{'previous_logical_velocity','takeup_motor_radians_per_second',
            'blocked_motor_direction','direction_lease','approved_reversal_direction'}:
        raise ValueError('unknown model comparison planner inputs')
    rows, ledgers = {}, {}
    first_engine, first_snapshot = next(iter(arms.values()))
    for label, (engine, snapshot) in arms.items():
        if engine.runtime.command_capable:
            raise ValueError('model comparison requires non-commanding runtime')
        state, position, position_ns, target, belief = snapshot
        if (state.timestamp_ns != first_snapshot[0].timestamp_ns
                or position_ns != first_snapshot[2]
                or engine.config != first_engine.config):
            raise ValueError('unmatched model root time or planner configuration')
        np.testing.assert_array_equal(position, first_snapshot[1])
        np.testing.assert_array_equal(target, first_snapshot[3])
        if abs(state.timestamp_ns-position_ns) > 150_000_000:
            raise ValueError('model comparison root/position skew exceeds 150ms')
        for key in ('velocity_min', 'velocity_max', 'position_lower', 'position_upper'):
            np.testing.assert_array_equal(getattr(engine.contract, key),
                getattr(first_engine.contract, key))
        if not state.estimator.prediction_ready:
            raise ValueError('model comparison root is not prediction ready')
        planner = FixedCandidates(engine.runtime, engine.contract,
            replace(engine.config, planning_deadline_s=60.), candidates, 'bounded')
        captured = []
        result = planner.plan(state.clone(), position.copy(), target.copy(),
            transmission_state=copy.deepcopy(belief), candidate_diagnostics=captured.append,
            **copy.deepcopy(planner_inputs or {}))
        if not result.valid or len(captured) != 1:
            raise RuntimeError(f'{label} diagnostic solve failed: {result.reason}')
        ledger = captured[0]
        ledgers[label] = ledger
        rows[label] = dict(selected_candidate=result.selected_candidate_index,
            command=result.command_logical_velocity.tolist(),
            total_cost=result.selected_total_cost, hold_cost=result.zero_total_cost,
            eligible_cost_order=np.flatnonzero(ledger['eligible'])[
                np.argsort(ledger['total_cost'][ledger['eligible']])].tolist())
    reference = next(iter(ledgers.values()))
    for ledger in ledgers.values():
        for key in ('requested', 'projected', 'motor_rates',
                    'takeup_cost', 'slew_cost', 'boundary_cost'):
            np.testing.assert_array_equal(reference[key], ledger[key],
                err_msg='unmatched model comparison: '+key)
    common = np.logical_and.reduce([l['eligible'] for l in ledgers.values()])
    indices = np.flatnonzero(common)
    for label, ledger in ledgers.items():
        rows[label]['common_eligible_cost_order'] = indices[
            np.argsort(ledger['total_cost'][common])].tolist()
        rows[label]['eligible_count'] = int(np.count_nonzero(ledger['eligible']))
    return dict(root_ns=first_snapshot[0].timestamp_ns, variants=rows,
        common_eligible_count=int(common.sum()),
        eligibility_difference_count=int(np.count_nonzero(np.logical_or.reduce([
            l['eligible'] != reference['eligible'] for l in ledgers.values()]))),
        selected_candidate_changed=len({r['selected_candidate'] for r in rows.values()}) > 1), ledgers


def compare_input_anchors(engine, snapshot, candidates, *, planner_inputs=None):
    """Offline only: identical posterior/proposals, varying coordinate 30 only.

    The equilibrium branch is a counterfactual, never fed back into inference.
    Returned packed roots and ledgers support subsequent matched-root replay.
    Intrusive diagnostic timing is not a control deadline qualification.
    """
    from .replay_takeup_startup import FixedCandidates
    if getattr(engine.runtime.profile, 'kind', None) != 'v171_static':
        raise ValueError('anchor comparison requires the explicit v171 static prior')
    if snapshot is None:
        raise ValueError('missing initialized replay snapshot')
    state, position, position_ns, target, belief = snapshot
    if not state.estimator.prediction_ready:
        raise ValueError('replay posterior is not prediction ready')
    if abs(state.timestamp_ns-position_ns) > 150_000_000:
        raise ValueError('replay root/position skew exceeds 150ms')
    equilibrium = float(engine.runtime.mechanics.stationary_lambda(state.strain))
    if not np.isfinite(equilibrium):
        raise ValueError('nonfinite equilibrium anchor')
    candidates = np.asarray(candidates, dtype=float).copy()
    if not np.isfinite(candidates).all():
        raise ValueError('nonfinite candidate bank')
    rows, ledgers, roots = {}, {}, {}
    planner_inputs = {} if planner_inputs is None else copy.deepcopy(planner_inputs)
    if set(planner_inputs)-{'previous_logical_velocity','takeup_motor_radians_per_second',
                          'blocked_motor_direction','direction_lease','approved_reversal_direction'}:
        raise ValueError('unknown anchor comparison planner inputs')
    for label, coordinate in (('posterior', float(state.mean[30])),
                              ('equilibrium', equilibrium)):
        root = state.clone()
        root.estimator.mean[30] = coordinate
        roots[label] = engine.runtime.export_planning_state(root)
        # A fresh planner for each branch prevents warm-start/RNG contamination.
        planner = FixedCandidates(engine.runtime, engine.contract,
            replace(engine.config, planning_deadline_s=60.), candidates, 'bounded')
        captured = []
        result = planner.plan(root, position.copy(), target.copy(),
            transmission_state=copy.deepcopy(belief),
            candidate_diagnostics=captured.append, **copy.deepcopy(planner_inputs))
        if not result.valid or len(captured) != 1:
            raise RuntimeError(f'{label} diagnostic solve failed: {result.reason}')
        ledger = captured[0]
        ledgers[label] = ledger
        rows[label] = dict(coordinate=coordinate,
            selected_candidate=result.selected_candidate_index,
            command=result.command_logical_velocity.tolist(),
            total_cost=result.selected_total_cost, hold_cost=result.zero_total_cost)
    for key in ('requested', 'projected', 'motor_rates', 'eligible',
                'takeup_cost', 'slew_cost', 'boundary_cost'):
        np.testing.assert_array_equal(ledgers['posterior'][key], ledgers['equilibrium'][key],
                                      err_msg='unmatched anchor comparison: '+key)
    # Save the exact input roots, not planner-mutated output snapshots.
    return dict(root_ns=state.timestamp_ns, position_ns=position_ns,
        position=position.tolist(), target_m=target.tolist(), variants=rows,
        selected_candidate_changed=rows['posterior']['selected_candidate'] !=
            rows['equilibrium']['selected_candidate']), ledgers, roots


def offline_replay(engine, path, *, maximum_seconds=60., plan_rate_hz=15.,
                   progress=None, progress_interval_s=5.):
    """Replay continuously; optional progress reports wall and recorded time separately."""
    if not np.isfinite(progress_interval_s) or progress_interval_s <= 0:
        raise ValueError('positive progress interval in wall seconds required')
    saved = json.loads(Path(path).read_text())
    if saved['schema'] != 'engagement_sensor_events_v1':
        raise ValueError('unknown sensor event schema')
    plans = []; start = None; next_plan = None; previous_receipt = None
    wall_start = time.monotonic()
    last_log = -float('inf')
    event_index = 0
    recorded_s = 0.

    def emit(stage, *, force=False):
        nonlocal last_log
        now = time.monotonic()
        if progress is None or not force and now-last_log < progress_interval_s:
            return
        last_log = now
        estimator = engine.runtime.estimator
        latest = {}
        for row in engine.measurements.rows[-100:]:
            latest[row['stage']] = row['wall_ms']
        progress(dict(stage=stage, event_index=event_index,
            total_file_events=len(saved['events']), recorded_elapsed_s=round(recorded_s,3),
            requested_recorded_s=maximum_seconds, wall_elapsed_s=round(now-wall_start,3),
            counts=dict(engine.counts),
            tendon_phase=None if estimator is None else estimator.phase,
            latest_timing_ms=latest))

    emit('replay_started', force=True)
    for row in saved['events']:
        event = ReplayEvent(**row)
        if previous_receipt is not None and event.receipt_ns < previous_receipt:
            raise ValueError('replay must preserve monotonic receipt order')
        previous_receipt = event.receipt_ns
        start = event.receipt_ns if start is None else start
        if event.receipt_ns-start > maximum_seconds*1e9:
            break
        event_index += 1
        recorded_s = (event.receipt_ns-start)*1e-9
        emit('before_'+event.kind)
        engine.feed(event)
        emit('after_'+event.kind)
        if next_plan is None or event.receipt_ns >= next_plan:
            emit('snapshot_and_plan')
            snapshot = engine.snapshot()
            if engine.compute_assignment['planner_process'] and not engine.prepared:
                engine.prepare_planner(snapshot)
                result = None
            else:
                result = engine.plan_snapshot(snapshot, event.receipt_ns)
            if result is not None: plans.append(result)
            next_plan = event.receipt_ns+int(1e9/plan_rate_hz)
            emit('after_plan')
    emit('replay_complete', force=True)
    return plans
