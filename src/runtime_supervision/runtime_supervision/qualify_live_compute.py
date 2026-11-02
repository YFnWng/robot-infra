"""Bounded stationary live estimator + isolated planner timing; never commands.

Replaces the ordinary disarmed planner no-op with discarded solves. No task,
mode claim, arming, actuator link, or command publication is permitted here.
"""
from __future__ import annotations

import argparse
import copy
from collections import deque
from dataclasses import asdict, replace
import json
import hashlib
import math
import os
from pathlib import Path
import threading
import time

import numpy as np

from .compare_ros import plan_coverage, probe_parameters
from .compute_profile import Measurements, GcTrace, ReplayGcExperiment
from catheter_control.safety.lifecycle import readiness
from catheter_control.safety.validation import percentile_metrics


class CallbackMeasurements(Measurements):
    """Existing duration measurements plus bounded host-clock callback spans."""

    def __init__(self):
        super().__init__()
        self.spans = deque(maxlen=32768)
        self.dropped_spans = 0

    def call(self, stage, stamp, function, *args, **kwargs):
        started, thread = time.monotonic(), threading.get_native_id()
        try:
            return super().call(stage, stamp, function, *args, **kwargs)
        finally:
            ended = time.monotonic()
            if len(self.spans) == self.spans.maxlen:
                self.dropped_spans += 1
            self.spans.append((stage, started, ended, thread))


def parent_pause_evidence(plans, gc_capture, spans, dropped_spans=0):
    """Correlate intervals, not causes; no collection/scheduling intervention."""
    active, collections = {}, []
    for event in gc_capture['events']:
        key = (event['native_thread_id'], event['generation'])
        if event['phase'] == 'start':
            active[key] = event
        elif event['phase'] == 'stop' and key in active:
            start = active.pop(key)
            collections.append(dict(start_s=start['monotonic_s'], end_s=event['monotonic_s'],
                native_thread_id=key[0], generation=key[1],
                wall_ms=1000*(event['monotonic_s']-start['monotonic_s']),
                thread_cpu_ms=1000*(event['thread_cpu_s']-start['thread_cpu_s'])))
    invalid = []
    for index, plan in enumerate(plans):
        if plan['valid']:
            continue
        start = plan['plan_started_s']
        end = start + plan['elapsed_ms']/1000
        marks = plan.get('process_stage_marks', [])
        gaps = [dict(before=a['stage'], after=b['stage'], start_s=a['monotonic_s'],
            end_s=b['monotonic_s'], wall_ms=1000*(b['monotonic_s']-a['monotonic_s']),
            thread_cpu_ms=1000*(b['thread_cpu_s']-a['thread_cpu_s']))
            for a, b in zip(marks, marks[1:])]
        overlapping_gc = [row for row in collections if row['start_s'] < end and row['end_s'] > start]
        callbacks = [dict(stage=stage, start_s=a, end_s=b, native_thread_id=thread)
            for stage, a, b, thread in spans if a < end and b > start]
        invalid.append(dict(plan_index=index, stage_intervals=gaps,
            overlapping_gc=overlapping_gc, overlapping_callbacks=callbacks,
            gc_evidence_complete=(gc_capture['dropped'] == 0 and not active)))
    return dict(gc=gc_capture, collection_intervals=collections,
        unmatched_gc_starts=list(active.values()), callback_spans_retained=len(spans),
        callback_spans_dropped=dropped_spans, invalid_plans=invalid,
        interpretation='Overlap is evidence of concurrent work, not proof of GC/GIL/OS causation. '
            'Dropped or incomplete events make absence inconclusive. No GC policy changed.')


def live_parameters(recorded, manifest, limits, device):
    parameters = probe_parameters(recorded, manifest, limits, device, "cpu")
    parameters.update(planner_process_enabled=True, adaptation_enabled=False,
                      simulation_state_reset_enabled=False)
    return parameters


def controller_parameters(controller_manifest=None, stack_config=None):
    """Reuse launch's authoritative stack composition, or a recorded session."""
    if stack_config is not None:
        from ament_index_python.packages import get_package_share_directory
        from catheter_control.orchestration.configuration import (
            ACTIVE_CONTROLLER_PARAMETERS, locate_stack, resolve_stack)
        root = Path(get_package_share_directory("catheter_control"))/"config"
        return dict(resolve_stack(locate_stack(stack_config, root), config_root=root,
            allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS),
            expected_node_name="catheter_mppi").parameters)
    return json.loads(Path(controller_manifest).read_text())["controller_parameters"]


def loaded_module_identity():
    """Record actual imports, not an assumed active installation."""
    import importlib
    import sys
    names = ("cr_meta_lnn.deployment.v171_streaming_runtime",
             "cr_meta_lnn.deployment.experimental.engagement_runtime",
             "cr_common.kinematics", "catheter_control.node",
             "catheter_control.planning.mppi",
             "runtime_supervision.qualify_live_compute",
             "catheter_control.orchestration.planner_process")
    result = {}
    for name in names:
        if name not in sys.modules:
            continue
        path = Path(importlib.import_module(name).__file__).resolve()
        result[name] = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return result


def worker_timing_evidence(plans):
    """Join late replies to their original solve, never to the recovery tick.

    All timestamps use host monotonic seconds. Wall minus thread CPU is an
    off-thread/wait indicator, not proof of an OS or GIL scheduling cause.
    An expired final request may have no decoded reply before probe shutdown.
    """
    traces = {}
    for plan in plans:
        for trace in plan['process_trace']:
            traces[trace['sequence']] = trace
    fields = {
        'request_to_worker_ms': ('request_to_worker_ms', 1.),
        'root_transfer_wall_ms': ('root_transfer_wall_s', 1000.),
        'root_transfer_cpu_ms': ('root_transfer_cpu_s', 1000.),
        'solve_wall_ms': ('solve_wall_s', 1000.),
        'solve_cpu_ms': ('solve_cpu_s', 1000.),
        'reply_ready_to_decoded_ms': ('reply_ready_to_decoded_ms', 1.),
        'parent_decode_wall_ms': ('decode_wall_s', 1000.),
        'parent_decode_cpu_ms': ('decode_cpu_s', 1000.),
    }
    metrics = {}
    for name, (field, scale) in fields.items():
        values = [trace[field]*scale for trace in traces.values() if field in trace]
        if values:
            metrics[name] = dict(count=len(values), **percentile_metrics(values))
    by_request = {trace['request_started_s']: trace for trace in traces.values()}
    invalid = []
    for index, plan in enumerate(plans):
        if plan['valid']:
            continue
        trace = by_request.get(plan['plan_started_s'])
        invalid.append(dict(plan_index=index, reason=plan['reason'],
            plan_started_offset_s=plan['plan_started_offset_s'],
            elapsed_ms=plan['elapsed_ms'],
            process_timings_ms=plan['process_timings_ms'],
            transport_failure=plan['transport_failure'], worker_trace=trace,
            worker_trace_status='decoded' if trace is not None else 'unavailable_at_shutdown'))
    return dict(decoded_requests=len(traces),
                discarded_replies=sum(bool(t['discarded']) for t in traces.values()),
                metrics=metrics, invalid_plans=invalid)


def observation_ready(inputs, limits):
    """Reuse sensor gates only; no manager authorization is sought for compute."""
    return readiness(replace(inputs, manager_ready=True, manager_age_s=0.,
                             target_available=True), limits)[1]


def initialize_sensor_evidence(node):
    # Fixed bounds cover startup plus the maximum 120 s measurement. No
    # filesystem writes or console logging are performed in sensor callbacks.
    node.probe_marker_callbacks = 0
    node.probe_marker_arrival = node.probe_marker_source_ns = None
    node.probe_marker_updates = deque(maxlen=4096)
    node.probe_sensor_snapshots = deque(maxlen=4096)
    node.probe_evidence_origin = time.monotonic()
    node.probe_evidence_time = node.probe_evidence_reason = None


def sensor_evidence_locked(node, now, reason, snapshot_source_ns):
    """Called with the production state lock; reuse authoritative diagnostics."""
    evidence = node._fault_snapshot_locked(reason)
    arrival = node.probe_marker_arrival
    pending = node.pending_marker_timestamp_ns
    evidence.update(
        offset_s=now-node.probe_evidence_origin,
        measurement_offset_s=(None if node.probe_started is None
                              else now-node.probe_started),
        marker_callback_count=node.probe_marker_callbacks,
        marker_callback_age_ms=None if arrival is None else 1000*max(0., now-arrival),
        marker_callback_source_ns=node.probe_marker_source_ns,
        marker_callback_source_age_ms=(None if node.probe_marker_source_ns is None
            else (node._now_ns()-node.probe_marker_source_ns)*1e-6),
        accepted_observations=node.accepted_observations,
        estimator_health=node.estimator_health,
        pending_marker_source_ns=pending,
        pending_marker_age_ms=(None if node.pending_marker_arrival is None
            else 1000*max(0., now-node.pending_marker_arrival)),
        planner_snapshot_source_ns=snapshot_source_ns,
        pending_marker_ahead_of_snapshot_ms=(
            None if pending is None or snapshot_source_ns is None
            else (pending-snapshot_source_ns)*1e-6))
    return evidence


def make_probe_type(base):
    """Compose production callbacks with a strictly non-commanding plan tick."""
    class LiveProbe(base):
        def _arm_cb(self, request, response):
            response.success = False
            response.message = "live_compute_probe_never_arms"
            return response

        def _publish_velocity_locked(self, velocity):
            return  # Independent of mutable ROS parameters.

        def _send_mode_locked(self, mode):
            return

        def _marker_cb(self, message):
            arrival = time.monotonic()
            stamp = message.header.stamp
            with self._lock:
                self.probe_marker_callbacks += 1
                self.probe_marker_arrival = arrival
                self.probe_marker_source_ns = int(stamp.sec)*10**9+int(stamp.nanosec)
            return super()._marker_cb(message)

        def _estimator_tick(self):
            function = super()._estimator_tick
            return (self.probe_meter.call("ros_estimator_callback", 0, function)
                    if self.probe_started is not None else function())

        def _process_encoder_sample(self, sample):
            function = super()._process_encoder_sample
            return (self.probe_meter.call("ros_encoder_owner", sample[0], function, sample)
                    if sample is not None and self.probe_started is not None else function(sample))

        def _process_marker_sample(self, sample):
            function = super()._process_marker_sample
            started = time.monotonic()
            before = self.last_marker_result
            result = (self.probe_meter.call("ros_marker_owner", sample[0], function, sample)
                      if sample is not None and self.probe_started is not None else function(sample))
            if sample is not None:
                ended = time.monotonic()
                with self._lock:
                    update = self.last_marker_result
                    changed = update is not None and update is not before
                    self.probe_marker_updates.append({
                        "offset_s": ended-self.probe_evidence_origin,
                        "measurement_offset_s": (None if self.probe_started is None
                            else ended-self.probe_started),
                        "source_ns": int(sample[0]),
                        "elapsed_ms": 1000*(ended-started),
                        "accepted": bool(changed and update.accepted),
                        "reason": str(update.reason) if changed else "no_update_result",
                        "accepted_observations": self.accepted_observations,
                        "consecutive_rejections": self.consecutive_rejections})
            return result

        def _record_sensor_evidence(self, now, reason, snapshot_source_ns):
            if (reason != self.probe_evidence_reason
                    or self.probe_evidence_time is None
                    or now-self.probe_evidence_time >= 1.):
                with self._lock:
                    evidence = sensor_evidence_locked(self, now, reason, snapshot_source_ns)
                self.probe_sensor_snapshots.append(evidence)
                self.probe_evidence_time, self.probe_evidence_reason = now, reason

        def _plan_tick(self):
            started = time.monotonic()
            if not self._plan_lock.acquire(blocking=False):
                return
            try:
                if self.probe_stop.is_set():
                    return
                if (self.probe_started is not None
                        and started-self.probe_started >= self.probe_duration_s):
                    return
                if self.armed or self.mode_claimed or self.command_output_enabled:
                    raise RuntimeError("non-commanding interlock violated")
                with self._lock:
                    reason = observation_ready(self._gate_inputs_locked(started), self.limits)
                    if self.fault_latched:
                        reason = "estimator_fault_latched"
                    position = self.position.copy() if self.position_valid else None
                    tip = None if self.observed_tip is None else self.observed_tip.copy()
                with self._snapshot_lock:
                    root = self._planner_snapshot
                    source = self._planner_snapshot_source_time
                    belief = copy.deepcopy(self._planner_transmission_snapshot)
                if root is None or source is None or position is None or tip is None:
                    reason = "missing_live_snapshot"
                elif started-source > self.limits.feedback_s:
                    reason = "planner_snapshot_stale"
                self.probe_wait_reason = reason
                self._record_sensor_evidence(started, reason,
                    None if root is None else int(root.timestamp_ns))
                if reason != "ready":
                    if self.probe_started is not None:
                        self.probe_gate_failures.append({"offset_s": started-self.probe_started,
                                                         "reason": reason})
                    return
                if self.probe_target is None:
                    # Private compute-only target, never published to the robot.
                    self.probe_target = tip + self.probe_offset_mm*.001
                kwargs = dict(observed_tip_base_m=tip, transmission_state=belief,
                              deadline_started_s=started)
                if self.probe_started is None:
                    plan = self.planner.plan(root, position, self.probe_target, **kwargs)
                    self.probe_warmup = {"valid": plan.valid, "reason": plan.reason,
                                         "elapsed_ms": 1000*(time.monotonic()-started)}
                    self.planner.reset()
                    if plan.reason not in ("ok", "deadline_missed"):
                        raise RuntimeError(f"warmup failed: {plan.reason}")
                    self.probe_started = time.monotonic()
                    return
                plan = self.probe_meter.call("ros_probe_planner", root.timestamp_ns,
                    self.planner.plan, root, position, self.probe_target, **kwargs)
                ended = time.monotonic()
                # Discard everything, including plans finishing with stale evidence.
                valid = plan.valid and ended-source <= self.limits.feedback_s
                self.probe_plans.append({
                    "plan_started_s": started,
                    "plan_started_offset_s": started-self.probe_started,
                    "elapsed_ms": 1000*(ended-started), "valid": valid,
                    "reason": plan.reason if not plan.valid or valid else "snapshot_stale_at_end",
                    "source_ns": root.timestamp_ns,
                    "snapshot_arrival_age_at_plan_end_ms": 1000*(ended-source),
                    "process_timings_ms": {k: v*1000 for k, v in self.planner.process_timings_s.items()},
                    # Existing IPC evidence only; no extra solve, wait, or I/O.
                    # A recovery tick may include an earlier discarded reply.
                    "process_trace": copy.deepcopy(self.planner.process_trace),
                    "process_stage_marks": copy.deepcopy(getattr(self.planner, 'process_stage_marks', [])),
                    "transport_failure": copy.deepcopy(self.planner.last_transport_failure)
                        if not plan.valid else None})
            except Exception as error:
                self.probe_errors.append(repr(error))
                self.probe_stop.set()
            finally:
                self._plan_lock.release()
    return LiveProbe


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    controller = parser.add_mutually_exclusive_group(required=True)
    controller.add_argument("--controller-manifest", type=Path)
    controller.add_argument("--stack-config", help="installed reviewed stack, as in control.launch.py")
    parser.add_argument("--model-manifest", required=True)
    parser.add_argument("--limits-file", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--duration-s", type=float, default=30.)
    parser.add_argument("--startup-timeout-s", type=float, default=60.)
    parser.add_argument('--gc-policy', choices=('normal', 'live-stationary'), default='normal',
                        help='bounded parent-probe A/B only; never changes production GC')
    parser.add_argument('--gc-max-growth-mb', type=float, default=256.)
    parser.add_argument("--target-offset-mm", type=float, nargs=3, default=[0., 0., 5.])
    args = parser.parse_args(argv)
    if not (math.isfinite(args.duration_s) and 1 <= args.duration_s <= 120
            and math.isfinite(args.startup_timeout_s) and 1 <= args.startup_timeout_s <= 120
            and np.isfinite(args.target_offset_mm).all()):
        parser.error("finite duration/startup in [1,120] s and finite target offset required")
    import rclpy
    import yaml
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from catheter_control.node import CatheterControlNode
    recorded = controller_parameters(args.controller_manifest, args.stack_config)
    parameters = live_parameters(recorded, args.model_manifest, args.limits_file, args.device)
    gc_experiment = ReplayGcExperiment(args.gc_policy, args.duration_s+args.startup_timeout_s,
        args.gc_max_growth_mb, os.environ.get('ROS_DOMAIN_ID', '0'),
        bool(parameters.get('use_sim_time', False)))
    args.output.mkdir(parents=True, exist_ok=False)
    params = args.output/"parameters.yaml"
    params.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
    rclpy.init(args=["--ros-args", "--params-file", str(params),
                    "-r", "__node:=live_compute_qualification",
                    "-r", "/teleop/control:=/live_compute_probe/blocked_control",
                    "-r", "/teleop/event:=/live_compute_probe/blocked_event",
                    "-r", "/catheter_mppi/set_armed:=/live_compute_probe/reject_arm"])
    node = executor = gc_trace = None
    errors = []
    try:
        guard = Node("live_compute_preflight", use_global_arguments=False)
        try:
            rclpy.spin_once(guard, timeout_sec=1.)
            if {"catheter_mppi", "live_compute_qualification"} & set(guard.get_node_names()):
                raise RuntimeError("stop the existing controller/qualification probe first")
        finally:
            guard.destroy_node()
        node = make_probe_type(CatheterControlNode)()
        node.probe_meter = CallbackMeasurements()
        node.probe_plans, node.probe_errors, node.probe_gate_failures = [], [], []
        node.probe_stop = threading.Event()
        node.probe_started = node.probe_target = node.probe_warmup = None
        node.probe_duration_s = args.duration_s
        node.probe_offset_mm = np.asarray(args.target_offset_mm)
        initialize_sensor_evidence(node)
        node.probe_wait_reason = "waiting_for_live_feedback"
        # Experimental parent holds a CPU backend; CUDA exists only in the
        # production ProcessPlanner child. Do not mistake parent storage for
        # the configured and initialized child execution device.
        if str(node.runtime.device) != "cpu" or node.compute_device.device.type != args.device:
            raise RuntimeError("requested CPU-estimator/planner assignment not established")
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        # Existing bounded observer intervention; both arms collect once before
        # executor callbacks start. Production and child GC remain unchanged.
        gc_experiment.start()
        gc_trace = GcTrace()
        gc_trace.start()
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        waiting = time.monotonic()
        next_log = waiting
        while rclpy.ok() and not node.probe_stop.is_set():
            now = time.monotonic()
            guard_reason = gc_experiment.check()
            if guard_reason is not None:
                errors.append(guard_reason)
                break
            if node.probe_started is None:
                if now-waiting >= args.startup_timeout_s:
                    errors.append(f"startup_timeout:{node.probe_wait_reason}")
                    break
            elif now-node.probe_started >= args.duration_s:
                break
            if now >= next_log:
                print(json.dumps({"phase": "waiting" if node.probe_started is None else "measuring",
                                  "reason": node.probe_wait_reason,
                                  "plans": len(node.probe_plans)}), flush=True)
                next_log = now+5
            node.probe_stop.wait(.1)
    except KeyboardInterrupt:
        errors.append("interrupted")
    except Exception as error:
        errors.append(repr(error))
    finally:
        if node is not None:
            node.probe_stop.set()
        try:
            if executor is not None:
                executor.shutdown(timeout_sec=35.)
        finally:
            gc_capture = (gc_trace.stop() if gc_trace is not None else
                          dict(events=[], dropped=0, enabled=None, thresholds=None))
            gc_experiment_report = gc_experiment.stop()
        if node is not None:
            errors.extend(node.probe_errors)
            plans = node.probe_plans
            coverage = plan_coverage(plans, 0., args.duration_s,
                                     float(node.get_parameter("plan_rate_hz").value))
            report = {"scope": __doc__, "ever_armed": node.armed,
                "command_output_enabled": node.command_output_enabled,
                "model_manifest_sha256": hashlib.sha256(Path(args.model_manifest).read_bytes()).hexdigest(),
                "planner_config": asdict(node.planner.config),
                "runtime_devices": {"planner": str(node.compute_device.device), "estimator": str(node.runtime.device)},
                "runtime_identity": asdict(node.runtime_bundle.identity),
                "loaded_modules": loaded_module_identity(),
                "controller_parameters": parameters,
                "warmup": node.probe_warmup, "plans": plans, "plan_coverage": coverage,
                "worker_timing": worker_timing_evidence(plans),
                "parent_pause_evidence": parent_pause_evidence(plans, gc_capture,
                    node.probe_meter.spans, node.probe_meter.dropped_spans),
                "gc_experiment": gc_experiment_report,
                "gate_failures": node.probe_gate_failures, "failures": errors,
                "sensor_snapshots": list(node.probe_sensor_snapshots),
                "marker_update_events": list(node.probe_marker_updates),
                "sensor_evidence_limits": {"rows_per_series": 4096,
                    "snapshot_period_s": 1., "snapshot_on_gate_change": True},
                "stages": node.probe_meter.summaries(),
                "deadline_misses": sum(p["reason"] == "deadline_missed" for p in plans),
                "target_m": None if node.probe_target is None else node.probe_target.tolist(),
                "limitations": ["Stationary live compute, not closed-loop trajectory/control accuracy",
                    "No active takeup arbitration, mode claims, command heartbeat or actuator path",
                    "Manager authorization excluded for compute only; production sensor gates unchanged",
                    "Marker callback arrivals are subscriber delivery, not camera acquisition",
                    "Pending timestamp comparison uses the replace-only planner snapshot, not a concurrent runtime read",
                    "Worker wall/CPU timing cannot by itself identify OS, GIL or CUDA scheduling; final pending replies may be unavailable",
                    "Bounded in-memory diagnostics add observer overhead; no per-frame logging or extra subscriptions"]}
            valid_ms = [p["elapsed_ms"] for p in plans if p["valid"]]
            report["valid_plan_elapsed_ms"] = percentile_metrics(valid_ms) if valid_ms else {}
            report["passed"] = coverage["passed"] and not errors and not node.probe_gate_failures
            node.shutdown()
            node.destroy_node()
        else:
            report = {"passed": False, "failures": errors}
        rclpy.shutdown()
        path = args.output/"report.json"
        path.write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps({"report": str(path), "passed": report["passed"]}), flush=True)
    if not report["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
