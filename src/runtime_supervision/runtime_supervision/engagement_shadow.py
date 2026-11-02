"""Bounded G4 ROS observer. No control publisher, client, or arm service.

Subscription callbacks retain bounded encoder history and latest-value samples. One estimator timer owns
all mutable model state and exchanges deep snapshots with a separate planning
group. A separate diagnostic group publishes only private String diagnostics.
"""
import argparse
from contextlib import nullcontext
from collections import deque
import json
import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PointStamped
from diagnostic_msgs.msg import DiagnosticArray
from control_interface.msg import DeviceStream, ControlCycleTiming, EstimatorStateTrace

from catheter_control.orchestration.estimator_owner import marker_measurement, stamp_ns
from catheter_control.orchestration.timing import PeriodicTimerProbe
from .compute_profile import ReplayEvent, GcTrace, ReplayGcExperiment, DiagnosticRecords, ObserverQuiescence
from .engagement_qualification import finite_json
from .engagement_g4 import add_runtime_arguments, make_engine
from catheter_control.orchestration.causal_schedule import CausalMarkerSchedule


class BaselineActivity:
    """Observe task activity only; never arm, disarm or control the baseline."""

    def __init__(self, expected_targets=0):
        self.expected_targets = expected_targets
        self.targets = 0
        self.state = None
        self.active_target = False
        self.completed_targets = 0
        self.fault_seen = False
        self.identity = {}
        self.transitions = []
        self.receipt_s = None
        self.source_ns = None
        self.disarmed_revision = 0

    def target(self):
        self.targets += 1
        self.active_target = False

    def observe(self, stamp, state, values):
        self.receipt_s = time.monotonic()
        self.source_ns = stamp
        if state != self.state:
            if state == 'DISARMED':
                self.disarmed_revision += 1
            self.transitions.append(dict(source_ns=stamp, state=state,
                observed_monotonic_s=self.receipt_s, reason=values.get('reason')))
        self.state = state
        self.identity = {key: values[key] for key in (
            'mppi_samples', 'planner_process_enabled', 'planner_compute_device',
            'estimator_compute_device', 'estimator_compute_dtype') if key in values}
        if state == 'FAULTED':
            self.fault_seen = True
        if state == 'ACTIVE' and self.targets:
            self.active_target = True
        if state == 'DISARMED' and self.active_target:
            self.completed_targets += 1
            self.active_target = False

    @property
    def finished(self):
        return self.fault_seen or (self.expected_targets > 0
                                  and self.completed_targets >= self.expected_targets)

    def report(self):
        return dict(expected_targets=self.expected_targets, targets=self.targets,
                    completed_target_attempts=self.completed_targets,
                    fault_seen=self.fault_seen, final_state=self.state,
                    identity=self.identity, transitions=self.transitions,
                    note='Disarm counts completed attempts, not successful reaches; inspect baseline bag results.')


class ShadowInputs:
    """Bounded handoff; never silently discard raw encoder reversals.

    The caller holds its input lock. POS, targets and images are latest-value
    streams, as in the baseline. Raw encoders remain ordered (at most four are
    processed per owner tick). Losing encoder history latches planning inhibition.
    """

    def __init__(self):
        self.encoders = deque()
        self.latest = {}
        self.last_encoder_ns = None
        self.failed = False
        self.schedule = CausalMarkerSchedule(20., time.monotonic())

    def put(self, arrival, event, counts):
        if event.kind == 'encoder':
            if self.last_encoder_ns is not None and event.source_ns <= self.last_encoder_ns:
                counts['nonmonotonic_input_encoder'] += 1
                return
            self.last_encoder_ns = event.source_ns
            if len(self.encoders) >= 128:
                counts['input_queue_overflow'] += 1
                self.failed = True
                return
            self.encoders.append((arrival, event))
        else:
            previous = self.latest.get(event.kind)
            if previous is not None:
                if event.source_ns <= previous[1].source_ns:
                    counts['nonmonotonic_input_'+event.kind] += 1
                    return
                counts['input_superseded_'+event.kind] += 1
            self.latest[event.kind] = (arrival, event)

    def take(self):
        events = [self.latest.pop(kind) for kind in ('position', 'target')
                  if kind in self.latest]
        events.extend(self.encoders.popleft() for _ in range(min(4, len(self.encoders))))
        return events

    def take_marker(self, now, encoder_ns):
        item = self.latest.get('marker')
        if item is None:
            return None
        if self.schedule.decision(now, item[1].source_ns, encoder_ns) != 'ready':
            return None
        self.schedule.commit(now)
        return self.latest.pop('marker')


class EngagementShadow(Node):
    def __init__(self,args):
        super().__init__('engagement_shadow',namespace='/experimental_engagement')
        self.engine=make_engine(args)
        self.engine.bound_diagnostics()
        self.gc_quiescence = (ObserverQuiescence() if args.gc_policy in
            ('task-boundary','live-task-boundary') else None)
        self.pending=ShadowInputs()
        self.queue_lock=threading.Lock()
        self.snapshot_lock=threading.Lock()
        self.snapshot=None
        self.model_status={}
        self.plans=DiagnosticRecords()
        self.follow_baseline = args.stop_after_targets > 0
        self.baseline_activity = BaselineActivity(args.stop_after_targets)
        self.input_group=MutuallyExclusiveCallbackGroup()
        self.owner_group=MutuallyExclusiveCallbackGroup()
        self.plan_group=MutuallyExclusiveCallbackGroup()
        self.io_group=MutuallyExclusiveCallbackGroup()
        self.estimator_probe=PeriodicTimerProbe(.02,time.monotonic())
        self.planner_probe=PeriodicTimerProbe(1/15.,time.monotonic())
        self.create_subscription(DeviceStream,args.prefix+'/device/state',self.device_callback,
            qos_profile_sensor_data,callback_group=self.input_group)
        self.create_subscription(PointCloud,args.prefix+'/shape_tracking/markers',self.marker_callback,
            qos_profile_sensor_data,callback_group=self.input_group)
        self.create_subscription(ControlCycleTiming,args.prefix+'/catheter_mppi/control_cycle_timing',
            self.baseline_callback,qos_profile_sensor_data,callback_group=self.io_group)
        self.create_subscription(EstimatorStateTrace,args.prefix+'/catheter_mppi/estimator_trace',
            self.baseline_shape_callback,qos_profile_sensor_data,callback_group=self.io_group)
        self.create_subscription(DiagnosticArray,args.prefix+'/catheter_mppi/status',
            self.baseline_status_callback,10,callback_group=self.io_group)
        self.create_subscription(PointStamped,args.prefix+'/catheter_mppi/target_tip',
            self.target_callback,10,callback_group=self.io_group)
        self.status=self.create_publisher(String,'status',10)
        self.create_timer(.02,self.owner_callback,callback_group=self.owner_group)
        self.create_timer(1/15.,self.plan_callback,callback_group=self.plan_group)
        self.create_timer(1.,self.status_callback,callback_group=self.io_group)

    def enqueue(self,event):
        with self.queue_lock:
            self.pending.put(time.monotonic_ns(),event,self.engine.counts)

    def device_callback(self,message):
        kind={ord('E'):'encoder',ord('P'):'position'}.get(message.predicate)
        if kind is None:return
        stamp=stamp_ns(message.header)
        if stamp<=0 or len(message.data)!=6:
            self.engine.counts['invalid_device_message']+=1;return
        self.enqueue(ReplayEvent(self.get_clock().now().nanoseconds,stamp,kind,list(message.data)))

    def marker_callback(self,message):
        try:
            stamp,points,quality=marker_measurement(message,'robot_base')
            self.enqueue(ReplayEvent(self.get_clock().now().nanoseconds,stamp,'marker',points,quality))
        except ValueError:
            self.engine.counts['invalid_marker_message']+=1

    def baseline_callback(self,message):
        values={name:getattr(message,name) for name in ('planner_elapsed_ms',
            'estimator_callback_latest_ms','marker_total_ms','marker_rewind_ms',
            'marker_correction_ms','marker_replay_ms','marker_source_age_ms',
            'planner_snapshot_age_ms','plan_valid','plan_reason')}
        values['controller_reason'] = getattr(message, 'controller_reason', None)
        values['controller_state'] = getattr(message, 'controller_state', None)
        # Diagnostic ingestion is lightweight and never invokes estimator work.
        self.engine.feed(ReplayEvent(self.get_clock().now().nanoseconds,stamp_ns(message.header),'plan',values))

    def baseline_status_callback(self, message):
        for status in message.status:
            if status.name == 'catheter_control/mppi':
                self.baseline_activity.observe(stamp_ns(message.header), status.message,
                    {item.key: item.value for item in status.values})

    def baseline_shape_callback(self,message):
        self.engine.feed(ReplayEvent(self.get_clock().now().nanoseconds,
            int(message.observation_timestamp_ns),'baseline_shape',
            dict(estimated_tip_m=list(message.estimated_tip_m),
                 observed_tip_m=list(message.observed_tip_m),phase=list(message.backlash_phase))))

    def target_callback(self,message):
        if message.header.frame_id!='robot_base':
            self.engine.counts['invalid_target_frame']+=1
            return
        # Share the diagnostic callback group so target and state observations
        # cannot interleave updates to the activity record.
        self.baseline_activity.target()
        self.enqueue(ReplayEvent(self.get_clock().now().nanoseconds,stamp_ns(message.header),
            'target',[message.point.x,message.point.y,message.point.z]))

    def owner_callback(self):
        guard = getattr(self,'gc_quiescence',None)
        with guard.work() if guard is not None else nullcontext(True) as admitted:
            if not admitted:
                self.engine.counts['gc_maintenance_owner_skips'] += 1
                return
            EngagementShadow._owner_callback(self)

    def _owner_callback(self):
        started=time.monotonic()
        self.record('estimator_timer_lateness',self.estimator_probe.observe(started))
        with self.queue_lock:
            events=self.pending.take()
        for arrival,event in events:
            self.record('input_owner_queue_wait',(time.monotonic_ns()-arrival)*1e-9,event.source_ns)
            try:self.engine.feed(event)
            except ValueError as error:self.engine.counts['invalid_input:'+str(error)]+=1
        # One causal, rate-limited correction per tick; no correction backlog.
        with self.queue_lock:
            marker=self.pending.take_marker(time.monotonic(),self.engine.latest_encoder_ns)
            failed=self.pending.failed
        if marker is not None:
            arrival,event=marker
            self.record('input_owner_queue_wait',(time.monotonic_ns()-arrival)*1e-9,event.source_ns)
            try:self.engine.feed(event)
            except ValueError as error:self.engine.counts['invalid_input:'+str(error)]+=1
        snapshot=self.engine.snapshot()
        model_status=self.engine.runtime.diagnostics()
        with self.snapshot_lock:
            self.snapshot=None if failed else snapshot
            self.model_status=model_status
        self.record('estimator_callback',time.monotonic()-started)

    def plan_callback(self):
        guard = getattr(self,'gc_quiescence',None)
        with guard.work() if guard is not None else nullcontext(True) as admitted:
            if not admitted:
                self.engine.counts['gc_maintenance_planner_skips'] += 1
                return
            EngagementShadow._plan_callback(self)

    def _plan_callback(self):
        started=time.monotonic()
        self.record('planner_timer_lateness',self.planner_probe.observe(started))
        if getattr(self.engine,'compute_assignment',{}).get('planner_process') and not self.engine.prepared:
            with self.snapshot_lock:
                snapshot = self.snapshot
            if self.engine.prepare_planner(snapshot):
                print(json.dumps(dict(stage='shadow_ready',command_capable=False,
                    **self.engine.compute_assignment)),flush=True)
            return
        if getattr(self, 'follow_baseline', False) and self.baseline_activity.state != 'ACTIVE':
            self.engine.counts['plan_baseline_inactive'] += 1
            return
        with self.snapshot_lock:snapshot=self.snapshot
        result=self.engine.plan_snapshot(snapshot,self.get_clock().now().nanoseconds,
            defer_diagnostics=True)
        if result is not None:self.plans.append(result)
        self.record('planner_callback',time.monotonic()-started)

    def record(self,stage,seconds,stamp=0):
        self.engine.measurements.rows.append(dict(stage=stage,source_ns=int(stamp),
            wall_ms=seconds*1000,thread_cpu_ms=None))

    def status_callback(self):
        with self.snapshot_lock:model_status=self.model_status
        with self.queue_lock:
            input_failed=self.pending.failed
            encoder_pending=len(self.pending.encoders)
        message=String()
        message.data=json.dumps(finite_json(dict(runtime_family='experimental_joint_engagement',
            command_capable=False,counts=dict(self.engine.counts),
            input_history_lost=input_failed,encoder_pending=encoder_pending,
            model=model_status)),allow_nan=False)
        self.status.publish(message)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    add_runtime_arguments(parser)
    parser.add_argument('--prefix',default='')
    parser.add_argument('--duration-s',type=float,default=60.)
    parser.add_argument('--outdir',required=True)
    parser.add_argument('--gc-policy',choices=('normal','deferred','task-boundary','live-task-boundary'),default='normal',
        help='Observer-only GC intervention; replay policies require domain 214/sim time; live policy domain 42/real time')
    parser.add_argument('--gc-max-growth-mb',type=float,default=512.,
        help='Sampled RSS growth guard in MiB for GC interventions (not a hard memory cap)')
    parser.add_argument('--stop-after-targets',type=int,default=0,
        help='Opt-in task-following test: plan only while baseline ACTIVE and stop after N active/disarm cycles')
    args,ros_args=parser.parse_known_args(argv)
    if not 0<args.duration_s<=3600:raise ValueError('bounded duration must be in (0,3600] seconds')
    if args.stop_after_targets < 0:raise ValueError('stop-after-targets must be nonnegative')
    if args.gc_policy in ('task-boundary','live-task-boundary') and args.stop_after_targets <= 0:
        raise ValueError('task-boundary GC requires stop-after-targets > 0')
    if args.gc_policy == 'live-task-boundary' and not args.planner_process:
        raise ValueError('live boundary GC requires the isolated planner process')
    if Path(args.outdir).exists():raise ValueError('use a new report directory')
    gc_experiment=ReplayGcExperiment(args.gc_policy,args.duration_s,args.gc_max_growth_mb,
        os.environ.get('ROS_DOMAIN_ID'),simulated_time=args.gc_policy != 'live-task-boundary')
    rclpy.init(args=ros_args)
    node=EngagementShadow(args)
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    started=time.monotonic()
    stop_reason='duration_limit'
    callback_error=None
    gc_trace=GcTrace()
    trace_started=False
    try:
        simulated_time = node.get_parameter('use_sim_time').value
        if args.gc_policy in ('deferred','task-boundary') and not simulated_time:
            raise ValueError('deferred GC requires use_sim_time=true')
        if args.gc_policy == 'live-task-boundary' and simulated_time:
            raise ValueError('live boundary GC requires use_sim_time=false')
        gc_experiment.start()
        gc_trace.start()
        trace_started=True
        started=time.monotonic()
        print(json.dumps(dict(stage='shadow_waiting_for_observed_preparation' if args.planner_process else 'shadow_ready',command_capable=False,
            device=args.device,samples=args.samples,stop_after_targets=args.stop_after_targets,
            gc_policy=args.gc_policy)),flush=True)
        while rclpy.ok() and time.monotonic()-started<args.duration_s:
            guard_reason=gc_experiment.check()
            if guard_reason:
                stop_reason=guard_reason
                break
            executor.spin_once(timeout_sec=.1)
            if args.gc_policy in ('task-boundary','live-task-boundary'):
                activity = node.baseline_activity
                def planner_idle():
                    if args.gc_policy == 'live-task-boundary':
                        if not fresh_disarmed_status(activity,node.get_clock().now().nanoseconds):
                            return False
                    if not args.planner_process:
                        return True
                    diagnostics = node.engine.planner.worker_diagnostics()
                    return diagnostics['planner_worker_pending_sequence'] == 'None'
                row = gc_experiment.collect_boundary(activity.disarmed_revision,
                    activity.state, activity.receipt_s, node.gc_quiescence,
                    planner_idle=planner_idle)
                if row is not None:
                    row['state_after_collection'] = activity.state
                    print(json.dumps(dict(stage='gc_boundary_collection',**row)),flush=True)
            if node.follow_baseline and node.baseline_activity.finished:
                stop_reason='baseline_fault' if node.baseline_activity.fault_seen else 'target_attempts_complete'
                break
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        stop_reason='interrupted'
    except Exception as error:
        stop_reason='callback_exception'
        callback_error=dict(type=type(error).__name__,message=str(error))
        node.engine.counts['callback_exceptions'] += 1
        raise
    finally:
        try:
            executor.shutdown()
        finally:
            gc_experiment_report=gc_experiment.stop()
        gc_report=gc_trace.stop() if trace_started else None
        try:
            # Never run optional expensive work while the commanding baseline
            # is active, including an early timeout or Ctrl+C of the observer.
            if node.baseline_activity.state == 'DISARMED':
                node.engine.finish_deferred_diagnostics()
            context=node.baseline_activity.report()
            context['stop_reason']=stop_reason
            context['callback_error']=callback_error
            context['gc_trace']=gc_report
            context['gc_experiment']=gc_experiment_report
            context['gc_qualification_scope'] = ('live_noncommanding_observer' if
                args.gc_policy == 'live-task-boundary' else 'default_or_replay_observer')
            report=node.engine.report(args.outdir,scope='ROS_shadow_observer_load_requires_baseline_and_tracking_evidence',plans=node.plans,
                live_context=context)
            print(json.dumps({'report':str(report)}))
        finally:
            node.engine.close()
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()


def fresh_disarmed_status(activity, now_ns):
    """Live maintenance evidence only; never a claim of actuator authority."""
    return (activity.state == 'DISARMED' and activity.source_ns is not None
            and 0 <= now_ns-activity.source_ns <= 150_000_000
            and activity.receipt_s is not None
            and 0 <= time.monotonic()-activity.receipt_s <= .15)


if __name__=='__main__':main()
