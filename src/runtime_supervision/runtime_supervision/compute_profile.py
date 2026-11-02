"""Bounded, non-actuating recorded-input compute profiling (optimization O0).

No node, executor, publisher, device link, or hardware command is constructed.
This is an algorithm workload replay, not a reproduction of ROS scheduling or
the online transmission posterior. Timing and intrusive profiler runs differ.
"""
from __future__ import annotations

import argparse
from contextlib import nullcontext, contextmanager
from collections import deque
import gc
import cProfile
import csv
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
import os
import pickle
from pathlib import Path
import pstats
import time
import threading
import tracemalloc

import numpy as np
import torch

from catheter_control.planning.mppi import CatheterMppi, MppiConfig
from catheter_control.safety.hardware_contract import load_hardware_contract
from catheter_control.safety.validation import percentile_metrics
from catheter_control.transmission.backlash import BacklashSnapshot
from catheter_control.transmission.engaged_gain import EngagedGainConfig, EngagedGainEstimator


class DiagnosticRecords:
    """Bounded trusted in-process records stored as GC-untracked bytes.

    Decode only for post-run analysis. Never use for estimator/command history.
    Limits apply to encoded payloads, not total RSS; oversized rows are counted
    and dropped. No files, background writer, or external pickle input exist.
    """

    def __init__(self, capacity=4096, byte_limit=8*1024**2):
        if capacity <= 0 or byte_limit <= 0:
            raise ValueError('positive diagnostic record and byte limits required')
        self.capacity, self.byte_limit = capacity, byte_limit
        self.records = deque()
        self.bytes = self.total = self.dropped = 0
        self.lock = threading.Lock()

    def append(self, value):
        payload = pickle.dumps(value, protocol=5)
        with self.lock:
            self.total += 1
            if len(payload) > self.byte_limit:
                self.dropped += 1
                return
            while self.records and (len(self.records) >= self.capacity or
                                     self.bytes+len(payload) > self.byte_limit):
                self.bytes -= len(self.records.popleft())
                self.dropped += 1
            self.records.append(payload)
            self.bytes += len(payload)

    def __len__(self):
        return len(self.records)

    def __iter__(self):
        with self.lock:
            payloads = tuple(self.records)
        return (pickle.loads(payload) for payload in payloads)

    def __getitem__(self, index):
        return pickle.loads(self.records[index])

    def retention(self):
        return dict(total=self.total, retained=len(self), dropped=self.dropped,
                    encoded_bytes=self.bytes, capacity=self.capacity,
                    byte_limit=self.byte_limit, selection='latest_bounded_records')


class GcTrace:
    """Bounded observer-only GC events; never change collection policy.

    Monotonic timestamps share the worker-trace clock. A dropped event makes
    absence inconclusive. No filesystem work occurs in the GC callback.
    """

    def __init__(self, capacity=4096):
        self.events = deque(maxlen=capacity)
        self.dropped = 0
        self.callback = self.record

    def record(self, phase, info):
        if len(self.events) == self.events.maxlen:
            self.dropped += 1
        self.events.append(dict(phase=phase, generation=info['generation'],
            monotonic_s=time.monotonic(), thread_cpu_s=time.thread_time(),
            native_thread_id=threading.get_native_id()))

    def start(self):
        gc.callbacks.append(self.callback)

    def stop(self):
        gc.callbacks.remove(self.callback)
        return dict(events=list(self.events), dropped=self.dropped,
                    enabled=gc.isenabled(), thresholds=gc.get_threshold())


class ObserverQuiescence:
    """Non-blocking maintenance admission; estimator and planner still overlap."""

    def __init__(self):
        self.lock = threading.Lock()
        self.workers = 0
        self.maintenance = False

    @contextmanager
    def work(self):
        with self.lock:
            admitted = not self.maintenance
            if admitted:
                self.workers += 1
        try:
            yield admitted
        finally:
            if admitted:
                with self.lock:
                    self.workers -= 1

    @contextmanager
    def boundary(self):
        with self.lock:
            admitted = not self.maintenance and self.workers == 0
            if admitted:
                self.maintenance = True
        try:
            yield admitted
        finally:
            if admitted:
                with self.lock:
                    self.maintenance = False


class ReplayGcExperiment:
    """Bounded replay/live parent-observer intervention, not production GC.

    Reference counting remains active. RSS is sampled at most once per second
    outside executor callbacks; the growth guard is sampled, not a hard cap.
    Both arms collect before measurement. Call stop after callbacks quiesce,
    before optional diagnostics, including on interruption or callback errors.
    """

    def __init__(self, policy, duration_s, growth_mb, domain, simulated_time):
        if policy not in ('normal', 'deferred', 'task-boundary', 'live-task-boundary', 'live-stationary'):
            raise ValueError('unknown GC policy')
        if not math.isfinite(growth_mb) or not 0 < growth_mb <= 1024:
            raise ValueError('GC memory growth guard must be in (0,1024] MiB')
        if policy in ('deferred', 'task-boundary') and (str(domain) != '214' or not simulated_time
                                    or not 0 < duration_s <= 180):
            raise ValueError('deferred GC requires domain 214, simulated time, and duration <=180s')
        if policy == 'live-task-boundary' and (str(domain) != '42' or simulated_time
                                               or not 0 < duration_s <= 300):
            raise ValueError('live boundary GC requires domain 42, real time, and duration <=300s')
        if policy == 'live-stationary' and (str(domain) != '42' or simulated_time
                                             or not 0 < duration_s <= 240):
            raise ValueError('stationary observer GC requires domain 42, real time, and duration <=240s')
        self.policy = policy
        self.duration_s = duration_s
        self.growth_bytes = int(growth_mb * 1024**2)
        self.samples = []
        self.original = None
        self.boundaries = []
        self.last_boundary = None

    @staticmethod
    def rss_bytes():
        # Linux resident pages, not ru_maxrss (which is a lifetime high-water).
        return int(Path('/proc/self/statm').read_text().split()[1]) * os.sysconf('SC_PAGE_SIZE')

    def start(self):
        gc.collect()
        self.baseline_bytes = self.rss_bytes()
        self.started_s = time.monotonic()
        self.next_sample_s = self.started_s
        self.check()
        self.original = (gc.isenabled(), gc.get_threshold())
        if self.policy != 'normal':
            gc.disable()

    def collect_boundary(self, revision, state, receipt_s, quiescence, *, planner_idle=None):
        """Collect once per fresh observed DISARMED transition, never wait.

        The main replay loop calls this outside executor callbacks. An incoming
        baseline transition can still race observation; this is not a hardware
        idle guarantee. Both evidence and admission failure remain reportable.
        """
        if (self.policy not in ('task-boundary', 'live-task-boundary') or revision == self.last_boundary
                or state != 'DISARMED' or receipt_s is None
                or not 0 <= time.monotonic()-receipt_s <= .15):
            return None
        with quiescence.boundary() as admitted:
            if not admitted or (planner_idle is not None and not planner_idle()):
                return None
            started, cpu = time.monotonic(), time.thread_time()
            collected = gc.collect(2)
            row = dict(revision=revision, observed_state=state,
                observed_receipt_s=receipt_s, start_s=started, end_s=time.monotonic(),
                thread_cpu_ms=1000*(time.thread_time()-cpu), collected=collected)
            row['wall_ms'] = 1000*(row['end_s']-started)
            self.boundaries.append(row)
            self.last_boundary = revision
            return row

    def check(self):
        now = time.monotonic()
        if now >= self.next_sample_s:
            self.samples.append(dict(monotonic_s=now, rss_bytes=self.rss_bytes()))
            self.next_sample_s = now + 1.
        if self.policy != 'normal':
            if self.samples[-1]['rss_bytes'] - self.baseline_bytes > self.growth_bytes:
                return 'gc_memory_growth_limit'
            if now - self.started_s >= self.duration_s:
                return 'gc_duration_limit'
        return None

    def stop(self):
        if self.original is None:
            return None
        enabled, thresholds = self.original
        gc.set_threshold(*thresholds)
        if enabled:
            gc.enable()
        else:
            gc.disable()
        return dict(policy=self.policy, original_enabled=enabled,
            original_thresholds=thresholds, restored_enabled=gc.isenabled(),
            restored_thresholds=gc.get_threshold(), baseline_rss_bytes=self.baseline_bytes,
            max_sampled_rss_bytes=max(row['rss_bytes'] for row in self.samples),
            growth_limit_bytes=self.growth_bytes, samples=self.samples,
            boundary_collections=self.boundaries,
            duration_limit_s=self.duration_s, stopped_s=time.monotonic())


@dataclass(frozen=True)
class ReplayEvent:
    receipt_ns: int
    source_ns: int
    kind: str
    values: object
    quality: dict | None = None


def read_events(session: Path, prefix: str = "", *, include_status=False, require_plan=True, include_comparison=False,
                sensors_only=False, invalid_samples=None):
    """Reuse the storage/CDR and marker conversion used by existing audits."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from catheter_control.orchestration.estimator_owner import marker_measurement

    bag = session / "robot_bag"
    if not (bag / "metadata.yaml").is_file():
        raise ValueError("session must contain a finalized robot_bag")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    selected = {prefix + "/device/state": "device",
                prefix + "/shape_tracking/markers": "marker",
                prefix + "/catheter_mppi/control_cycle_timing": "plan"}
    if sensors_only:
        selected.pop(prefix + "/catheter_mppi/control_cycle_timing")
    if include_status:
        selected[prefix + "/catheter_mppi/status"] = "status"
    if include_comparison:
        selected[prefix + "/catheter_mppi/estimator_trace"] = "baseline_shape"
        selected[prefix + "/catheter_mppi/target_tip"] = "target"
    classes = {x.name: get_message(x.type)
               for x in reader.get_all_topics_and_types() if x.name in selected}
    required = set(selected)
    if not require_plan:
        required.discard(prefix + "/catheter_mppi/control_cycle_timing")
    if include_comparison:
        required.discard(prefix + "/catheter_mppi/estimator_trace")
        required.discard(prefix + "/catheter_mppi/target_tip")
    if required-set(classes):
        raise ValueError(f"required topics missing: {required-set(classes)}")
    events = []
    while reader.has_next():
        topic, payload, receipt = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize_message(payload, classes[topic])
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp <= 0:
            if invalid_samples is not None:
                invalid_samples.append(dict(topic=topic, source_ns=int(stamp), receipt_ns=int(receipt),
                                            reason='invalid_source_timestamp'))
                continue
            raise ValueError(f"invalid source timestamp: {topic}")
        kind = selected[topic]
        if kind == "target":
            if message.header.frame_id != "robot_base":
                raise ValueError("target must be expressed in robot_base")
            events.append(ReplayEvent(receipt, stamp, kind,
                [message.point.x, message.point.y, message.point.z]))
        elif kind == "baseline_shape":
            values=dict(estimated_tip_m=list(message.estimated_tip_m),
                observed_tip_m=list(message.observed_tip_m),phase=list(message.backlash_phase))
            events.append(ReplayEvent(receipt,int(message.observation_timestamp_ns),kind,values))
        elif kind == "status":
            values = {item.key: item.value for status in message.status for item in status.values}
            events.append(ReplayEvent(receipt, stamp, kind, values))
        elif kind == "marker":
            try:
                stamp, points, quality = marker_measurement(message, "robot_base")
            except ValueError as error:
                if invalid_samples is None:
                    raise
                invalid_samples.append(dict(topic=topic, source_ns=int(stamp), receipt_ns=int(receipt),
                                            reason=str(error)))
                continue
            events.append(ReplayEvent(receipt, stamp, kind, points, quality))
        elif kind == "plan":
            values = {name: getattr(message, name) for name in (
                "planner_elapsed_ms", "estimator_callback_latest_ms", "marker_total_ms",
                "marker_rewind_ms", "marker_correction_ms", "marker_replay_ms",
                "marker_source_age_ms", "planner_snapshot_age_ms", "plan_valid", "plan_reason")}
            events.append(ReplayEvent(receipt, stamp, kind, values))
        elif message.predicate in (ord("E"), ord("P")):
            values = np.asarray(message.data, dtype=float)
            if values.shape != (6,) or not np.isfinite(values).all():
                if invalid_samples is not None:
                    invalid_samples.append(dict(topic=topic, source_ns=int(stamp), receipt_ns=int(receipt),
                                                reason='invalid_six_axis_feedback'))
                    continue
                raise ValueError("invalid six-axis device sample")
            events.append(ReplayEvent(receipt, stamp,
                                      "encoder" if message.predicate == ord("E")
                                      else "position", values))
    return events


class Measurements:
    """Independent wall/thread-CPU measurements; no artificial CUDA fences."""
    def __init__(self, intrusive=False):
        self.rows = []
        self.intrusive = intrusive

    def call(self, stage, stamp, function, *args, **kwargs):
        scope = (torch.profiler.record_function(stage) if self.intrusive
                 else nullcontext())
        with scope:
            wall, cpu = time.perf_counter_ns(), time.thread_time_ns()
            result = function(*args, **kwargs)
            self.rows.append({"stage": stage, "source_ns": int(stamp),
                              "wall_ms": (time.perf_counter_ns()-wall)/1e6,
                              "thread_cpu_ms": (time.thread_time_ns()-cpu)/1e6})
        return result

    def summaries(self):
        result = {}
        for stage in sorted({row["stage"] for row in self.rows}):
            rows = [row for row in self.rows if row["stage"] == stage]
            cpu = [r["thread_cpu_ms"] for r in rows if r["thread_cpu_ms"] is not None]
            result[stage] = {"count": len(rows),
                             "wall": percentile_metrics([r["wall_ms"] for r in rows]),
                             "thread_cpu": percentile_metrics(cpu) if cpu else None}
        return result


def replay(runtime, contract, config, events, targets, *, start_ns, end_ns,
           measured: Measurements, encoder_period_ns=20_000_000,
           marker_period_ns=50_000_000, transmission_state=None,
           direction_lease=None, on_window_start=None, on_estimator_event=None,
           before_event=None, plan_function=None):
    """Sequential causal workload, preserving prefix history before the window.

    Encoders are thinned at the configured source-time recurrence period;
    newest pending markers are causally deferred and rate-limited. Recorded
    solve-completion receipts trigger offline solves, not callback-entry times.
    """
    last_encoder = last_marker = -10**30
    pending = None
    position = None
    target = None
    target_index = 0
    plans, corrections = [], []
    quiet = Measurements()
    window_started = False
    for event in events:
        if event.receipt_ns > end_ns:
            break
        meter = measured if event.receipt_ns >= start_ns else quiet
        if meter is measured and not window_started:
            window_started = True
            if on_window_start is not None:
                on_window_start()
        if before_event is not None:
            before_event(event)
        while target_index < len(targets) and targets[target_index][0] <= event.receipt_ns:
            target = targets[target_index][1]
            target_index += 1
        if event.kind == "position":
            position = event.values
        elif event.kind == "marker":
            pending = event
        elif event.kind == "encoder" and event.source_ns-last_encoder >= encoder_period_ns:
            last_encoder = event.source_ns
            method = runtime.initialize if runtime.state is None else runtime.advance_encoder
            meter.call("encoder_propagation", event.source_ns, method,
                       event.source_ns, event.values)
            meter.call("diagnostics", event.source_ns, runtime.diagnostics)
            meter.call("snapshot_clone", event.source_ns, runtime.clone_state)
            if on_estimator_event is not None:
                on_estimator_event("encoder", event, None)
        elif event.kind == "plan" and runtime.state is not None and position is not None and target is not None:
            # Fresh seed + zero nominal freezes proposal noise for each case.
            # Do not reuse a warm start that depends on machine-specific misses.
            planner = CatheterMppi(runtime, contract, config)
            root = meter.call("plan_snapshot_clone", event.source_ns, runtime.clone_state)
            if plan_function is None:
                plan = meter.call("planner", event.source_ns, planner.plan,
                                  root, position, target,
                                  transmission_state=transmission_state,
                                  direction_lease=direction_lease)
            else:
                # Qualification can reuse a bounded process planner without
                # copying the estimator's causal replay implementation.
                plan = plan_function(root, position, target, event, meter)
            if meter is measured:
                plans.append({"source_ns": event.source_ns, "reason": plan.reason,
                              "valid": plan.valid, "elapsed_ms": plan.elapsed_s*1000,
                              "sample_projection_ms": plan.sample_projection_ms,
                              "rollout_ms": plan.rollout_ms,
                              "cost_weighting_ms": plan.cost_weighting_ms,
                              "update_projection_ms": plan.update_projection_ms})
        if (pending is not None and runtime.state is not None
                and pending.source_ns <= runtime.state.timestamp_ns
                and event.receipt_ns-last_marker >= marker_period_ns):
            marker, pending = pending, None
            last_marker = event.receipt_ns
            if on_estimator_event is not None:
                on_estimator_event("before_marker", marker, None)
            # Read-only workload accounting of the exact retained replay grid.
            replay_times = [entry.timestamp_ns for entry in runtime._rewind
                            if entry.timestamp_ns > marker.source_ns]
            previous = marker.source_ns
            replay_substeps = 0
            for stamp in replay_times:
                replay_substeps += max(1, math.ceil(
                    (stamp-previous)*1e-9/runtime.maximum_step_s))
                previous = stamp
            result = meter.call("marker_update", marker.source_ns,
                                runtime.observe_markers, marker.source_ns,
                                marker.values, marker.quality)
            meter.call("diagnostics", marker.source_ns, runtime.diagnostics)
            meter.call("snapshot_clone", marker.source_ns, runtime.clone_state)
            if on_estimator_event is not None:
                on_estimator_event("marker", marker, result)
            if meter is measured:
                early_rejection = result.reason in {
                    "observation_from_future", "observation_too_old",
                    "observation_before_rewind_buffer"}
                corrections.append({"source_ns": marker.source_ns,
                                    "accepted": result.accepted, "reason": result.reason,
                                    "observable_rank": getattr(result, "observable_rank", None),
                                    "replay_entries": len(replay_times) if result.accepted else 0,
                                    "replay_substeps": replay_substeps if result.accepted else 0,
                                    "timing_ms": ({} if early_rejection else
                                                  dict(runtime.last_marker_timing_ms))})
    return plans, corrections


def require_controller_idle():
    """Refuse profiling alongside a detected local controller; never stop it."""
    for process in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            command = process.read_bytes().decode(errors="replace").split("\0")
        except (OSError, PermissionError):
            continue
        if any(Path(token).name == "catheter_mppi" or token == "catheter_control.node"
               for token in command):
            raise RuntimeError("stop the controller before compute profiling; no process was stopped")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--model-manifest", type=Path, required=True)
    parser.add_argument("--limits-file", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--prefix", default="")
    parser.add_argument("--samples", type=int, default=512)
    parser.add_argument("--active-axes", type=int, choices=(2, 3), default=2)
    parser.add_argument("--gain-scenarios", type=int, choices=(1, 3), default=3)
    parser.add_argument("--start-offset-s", type=float, default=0)
    parser.add_argument("--duration-s", type=float, default=10)
    parser.add_argument("--deadline-s", type=float, default=.06)
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--profiler", action="store_true")
    parser.add_argument("--capture-states", action="store_true",
                        help="intrusive numerical qualification; save trusted local snapshots, not timing evidence")
    args = parser.parse_args(argv)
    if (args.samples < 8 or args.duration_s <= 0 or args.start_offset_s < 0
            or args.deadline_s <= 0 or not np.isfinite(
                [args.duration_s, args.start_offset_s, args.deadline_s]).all()):
        parser.error("require samples >= 8, finite positive duration/deadline and nonnegative offset")
    if args.profiler and args.duration_s > 2:
        parser.error("intrusive traces are limited to 2 s; use a separate longer timing run")
    # Avoid accidentally overwriting an earlier measurement or trace.
    args.output.mkdir(parents=True, exist_ok=False)
    require_controller_idle()
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    from cr_meta_lnn.deployment import load_runtime_bundle
    bundle = load_runtime_bundle(args.model_manifest, device=args.device,
                                 options={"marker_estimator": "ukf",
                                          "adaptation_enabled": False})
    runtime = bundle.runtime
    contract = load_hardware_contract(args.limits_file, "imricor_test")
    velocity_max = contract.velocity_max.copy()
    if args.active_axes == 2:
        velocity_max[1] = 0.
    contract = replace(contract, velocity_max=velocity_max,
                       velocity_min=np.minimum(contract.velocity_min, velocity_max))
    lease = np.array([1, 0 if args.active_axes == 2 else 1, 1], dtype=np.int8)
    gain = (EngagedGainEstimator(EngagedGainConfig(enabled=True)).snapshot()
            if args.gain_scenarios == 3 else None)
    transmission = BacklashSnapshot(
        width_rad=np.zeros(3), width_positive_rad=np.zeros(3),
        width_negative_rad=np.zeros(3), remaining_rad=np.zeros(3),
        motion_direction=lease.copy(), engaged_direction=lease.copy(),
        confidence=np.ones(3), confirmation_count=np.ones(3, dtype=np.int32),
        phase=("ENGAGED", "UNKNOWN" if args.active_axes == 2 else "ENGAGED", "ENGAGED"),
        engaged_gain=gain)
    config = MppiConfig(samples=args.samples, horizon_steps=4,
                        point_rollout_step_s=.2, point_rollout_coarse_steps=True,
                        engaged_gain_scenarios=args.gain_scenarios == 3,
                        planning_deadline_s=args.deadline_s, seed=args.seed)
    proposals, groups, _ = CatheterMppi(runtime, contract, config)._grouped_candidates(
        lease)
    np.savez(args.output/"proposal_fixture.npz", requested=proposals, groups=groups)
    events = read_events(args.session, args.prefix)
    if not events:
        raise ValueError("empty replay")
    journal = [json.loads(line) for line in
               (args.session/"trials.jsonl").read_text().splitlines()]
    targets = [(e["ros_time_ns"], np.asarray(e["target_m"]))
               for e in journal if e["event"] == "trial_started"]
    start = events[0].receipt_ns + int(args.start_offset_s*1e9)
    end = start + int(args.duration_s*1e9)
    if start > events[-1].receipt_ns:
        raise ValueError("measurement window starts after recording ends")
    meter = Measurements(args.profiler)
    activities = [torch.profiler.ProfilerActivity.CPU]
    if args.device == "cuda":
        activities.append(torch.profiler.ProfilerActivity.CUDA)
    profile = (torch.profiler.profile(activities=activities,
               record_shapes=True, profile_memory=True) if args.profiler else None)
    calls = cProfile.Profile()
    profiling_started = False
    captured_states, cartesian_rows = [], []

    def capture_state(stage, event, result):
        if not args.capture_states or stage == "before_marker" or not start <= event.receipt_ns <= end:
            return
        state = runtime.clone_state()
        captured_states.append(state)
        cartesian_rows.append({"stage": stage, "receipt_ns": event.receipt_ns,
                               "source_ns": event.source_ns,
                               "state_ns": state.timestamp_ns,
                               "markers_m": runtime.markers_for_state(state).detach().cpu().tolist(),
                               "centerline_m": runtime.centerline_from_state(state).detach().cpu().tolist()})

    def begin_window():
        nonlocal profiling_started
        if profile is not None:
            profile.start()
            tracemalloc.start()
            calls.enable()
            profiling_started = True

    try:
        plans, corrections = replay(runtime, contract, config, events, targets,
                                    start_ns=start, end_ns=end, measured=meter,
                                    transmission_state=transmission, direction_lease=lease,
                                    on_window_start=begin_window,
                                    on_estimator_event=capture_state if args.capture_states else None)
    finally:
        if profiling_started:
            calls.disable()
            profile.stop()
    allocations = None
    if profiling_started:
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        allocations = {"python_current_bytes": current, "python_peak_bytes": peak,
                       "scope": "Python allocations only; tensor/native allocations in Torch trace"}
        calls.dump_stats(str(args.output/"python_calls.prof"))
        with (args.output/"python_calls.txt").open("w") as stream:
            pstats.Stats(calls, stream=stream).sort_stats("cumulative").print_stats(100)
        profile.export_chrome_trace(str(args.output/"torch_trace.json"))
        function_rows = []
        for (filename, line, name), (primitive, total, own, cumulative, _) in pstats.Stats(calls).stats.items():
            if any(part in filename for part in ("catheter_control", "cr_meta_lnn", "cr_common")):
                function_rows.append({"file": filename, "line": line, "function": name,
                                      "calls": total, "primitive_calls": primitive,
                                      "own_seconds": own, "cumulative_seconds": cumulative})
        (args.output/"function_costs.json").write_text(json.dumps(
            sorted(function_rows, key=lambda r: r["cumulative_seconds"], reverse=True), indent=2)+"\n")
    with (args.output/"timing_rows.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=["stage", "source_ns", "wall_ms", "thread_cpu_ms"])
        writer.writeheader()
        writer.writerows(meter.rows)
    report = {"schema_version": 1, "scope": "OFFLINE_NON_ACTUATING_ALGORITHM_PROFILE",
              "intrusive_profiler": args.profiler, "session": str(args.session.resolve()),
              "intrusive_state_capture": args.capture_states,
              "runtime_identity": asdict(bundle.identity), "configuration": asdict(config),
              "device": args.device, "torch_version": torch.__version__,
              "active_axes": args.active_axes, "gain_scenario_count": args.gain_scenarios,
              "transmission_fixture": "fixed engaged direction, zero gap, prior gain uncertainty",
              "window": {"start_ns": start, "end_ns": end},
              "stages": meter.summaries(), "plans": plans, "marker_updates": corrections,
              "deadline_misses": sum(p["reason"] == "deadline_missed" for p in plans),
              "allocations": allocations,
              "input_sha256": hashlib.sha256(
                  (args.session/"trials.jsonl").read_bytes()).hexdigest(),
              "proposal_fixture_sha256": hashlib.sha256(
                  (args.output/"proposal_fixture.npz").read_bytes()).hexdigest(),
              "proposal_arrays_sha256": hashlib.sha256(
                  proposals.tobytes()+groups.tobytes()).hexdigest(),
              "input_hash_scope": "trial journal only; bag path is recorded, bag not hashed",
              "profiler_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "runtime_configuration": {"marker_estimator": "ukf", "adaptation_enabled": False},
              "limitations": ["Sequential algorithm replay; no executor/GIL contention measurement",
                  "No online backlash/gain posterior replay; raw encoder drives both model input streams",
                  "Fixed engaged transmission fixture is for workload comparison, not decision attribution",
                  "Seeded fresh zero-nominal proposals, not recorded online warm starts",
                  "No arm/home resets at trial boundaries; continuous prefix history retained",
                  "Initialization included; no cold-start exclusion unless selected window follows it",
                  "Thread CPU excludes native worker threads and GPU; wall time can include waits",
                  "Prefix is replayed without profiling; only selected window is instrumented",
                  "Asynchronous GPU per-stage times need Torch trace; no new per-stage barriers added",
                  "No ROS callbacks, transmission belief update, heartbeat or full-stack qualification"]}
    if args.capture_states:
        fixture = args.output/"captured_states.pt"
        # Pickled dataclasses are a trusted, locally generated fixture only.
        # Consumers must verify this digest and never accept arbitrary snapshots.
        torch.save(captured_states, fixture)
        report["captured_states_sha256"] = hashlib.sha256(fixture.read_bytes()).hexdigest()
        report["cartesian_trajectory"] = cartesian_rows
        report["limitations"].append("State/Cartesian capture perturbs work; these timings are not qualification evidence")
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({"output": str(args.output), "stage_rows": len(meter.rows),
                      "plans": len(plans), "deadline_misses": report["deadline_misses"]}))


if __name__ == "__main__":
    main()
