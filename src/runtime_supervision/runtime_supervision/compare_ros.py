"""Disarmed production ROS estimator/diagnostics plus a non-commanding MPPI probe.

No lifecycle arming, manager/device process, command replay or camera SDK.
The probe measures ROS execution overhead, not full active-control attribution.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
from dataclasses import asdict
import json
from pathlib import Path
import threading
import time

import numpy as np

from .compute_profile import Measurements, require_controller_idle
from .qualify_paced_compute import recorded_belief
from .path_commit_evidence import plan_outcomes


INPUT_TOPICS = ("/device/state", "/shape_tracking/markers", "/shape_tracking/marker_status")
ABLATIONS = ("combined", "no-diagnostics", "estimator-only", "planner-only")


def measurement_work(ablation):
    """Work enabled after the common prefix; idle timers/subscriptions remain."""
    if ablation not in ABLATIONS:
        raise ValueError(f"unknown ROS ablation: {ablation}")
    return {"estimator": ablation != "planner-only",
            "planner": ablation != "estimator-only",
            "diagnostics": ablation != "no-diagnostics"}


def require_measurement_work(ablation, stages, plans, failures, *, allow_path_transitions=False):
    work = measurement_work(ablation)
    required = ["ros_estimator_callback"] if work["estimator"] else []
    if work["planner"]:
        required.append("ros_probe_planner")
    missing = [name for name in required if not stages.get(name, {}).get("count", 0)]
    invalid = plan_outcomes(plans, allow_path_transitions=allow_path_transitions)["unexpected_invalid_count"]
    if failures or missing or (work["planner"] and (not plans or invalid)):
        raise RuntimeError(f"ROS comparison incomplete: missing={missing}, plans={len(plans)}, invalid={invalid}, failures={failures}")


def save_plan_fixture(path, inputs, contract):
    """Persist an owned local snapshot; never overwrite an existing fixture."""
    import torch
    metadata = path.with_suffix(".json")
    if path.exists() or metadata.exists():
        raise FileExistsError(path)
    torch.save(inputs, path)
    metadata.write_text(json.dumps({"sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                                    "contract": contract}, indent=2)+"\n")


def load_plan_fixture(path, contract, device):
    import torch
    metadata = json.loads(path.with_suffix(".json").read_text())
    if metadata["contract"] != contract:
        raise ValueError("planner fixture model/configuration mismatch")
    if metadata["sha256"] != hashlib.sha256(path.read_bytes()).hexdigest():
        raise ValueError("planner fixture hash mismatch")
    # Only our own locally captured, hash-checked dataclasses; not downloaded pickle.
    return torch.load(path, map_location=device, weights_only=False), metadata["sha256"]


def input_allowed(topic, message):
    """Only encoder/POS feedback and perception; never replay device commands."""
    return topic in INPUT_TOPICS and (topic != "/device/state" or message.predicate in (ord("E"), ord("P")))


def probe_parameters(recorded, model_manifest, limits_file, device, estimator_device=None):
    parameters = dict(recorded)
    parameters.update(command_output_enabled=False, use_sim_time=False,
                      model_manifest=model_manifest, limits_file=limits_file,
                      device=device, estimator_device=estimator_device or device, estimator_dtype="float32")
    return parameters


def coordinated_epoch(directory, role, timeout_s=60.):
    """Wait outside measurement for the parent-owned two-process start barrier."""
    import math
    (directory/(role+".ready")).touch(exist_ok=False)
    deadline = time.monotonic()+timeout_s
    while time.monotonic() < deadline:
        path = directory/"epoch.json"
        if path.exists():
            epoch = json.loads(path.read_text())["epoch_monotonic_s"]
            if not math.isfinite(epoch) or epoch <= time.monotonic():
                raise RuntimeError("invalid or expired coordinated replay epoch")
            return epoch
        time.sleep(.01)
    raise TimeoutError("coordinated replay start barrier expired")


def plan_coverage(plans, offset_s, duration_s, rate_hz, *, allow_path_transitions=False):
    """Offline cadence qualification, including both measurement boundaries."""
    times = sorted(row['plan_started_offset_s'] for row in plans)
    gaps = np.diff([offset_s, *times, offset_s+duration_s])
    expected = duration_s*rate_hz
    maximum_gap = float(max(gaps, default=duration_s))
    outcomes = plan_outcomes(plans, allow_path_transitions=allow_path_transitions)
    return {'expected_count': expected, 'actual_count': len(times),
            **outcomes,
            'maximum_gap_ms': 1000*maximum_gap,
            'passed': outcomes['passed']
                      and len(times) >= max(1, int(np.floor(expected))-1)
                      and maximum_gap <= 2/rate_hz}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--model-manifest", required=True)
    parser.add_argument("--limits-file", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--estimator-device", choices=("cpu", "cuda"),
                        help="defaults to planner device; estimator precision remains float32")
    parser.add_argument("--start-offset-s", type=float, default=32.)
    parser.add_argument("--duration-s", type=float, default=2.)
    parser.add_argument("--ros-ablation", choices=ABLATIONS, default="combined")
    parser.add_argument("--coordination-dir", type=Path)
    parser.add_argument("--process-role", choices=("planner", "estimator"))
    parser.add_argument("--ros-evolving-snapshots", action="store_true")
    parser.add_argument("--planner-process", action="store_true",
                        help="Exercise opt-in production socket worker; CPU estimator, combined replay")
    parser.add_argument("--path-reference-commit", action="store_true",
                        help="Replay path previews and record production commit decisions; offset is relative to path task start")
    parser.add_argument("--exchange-dir", type=Path)
    fixture_args = parser.add_mutually_exclusive_group()
    fixture_args.add_argument("--ros-plan-fixture", type=Path)
    fixture_args.add_argument("--write-plan-fixture", type=Path,
                              help="capture inputs at measurement entry and exit; not a timing run")
    args = parser.parse_args(argv)
    if args.path_reference_commit and not args.planner_process:
        parser.error("path-reference commit replay requires --planner-process")
    if args.planner_process and (args.estimator_device != "cpu"
            or args.ros_ablation != "combined" or args.ros_evolving_snapshots
            or args.ros_plan_fixture or args.write_plan_fixture or args.process_role):
        parser.error("production worker requires combined CPU-estimator replay without fixture/split roles")
    import math
    if (not math.isfinite(args.duration_s) or not 0 < args.duration_s <= 10
            or not math.isfinite(args.start_offset_s) or args.start_offset_s < 0):
        parser.error("require finite nonnegative offset and duration in (0,10] seconds")
    if args.write_plan_fixture and args.ros_ablation != "combined":
        parser.error("fixture capture requires combined mode")
    if args.ros_plan_fixture and args.ros_ablation not in ("combined", "planner-only"):
        parser.error("fixed planner fixture requires combined or planner-only")
    work = measurement_work(args.ros_ablation)
    if bool(args.coordination_dir) != bool(args.process_role):
        parser.error("coordination directory and process role must be supplied together")
    if args.ros_evolving_snapshots != bool(args.exchange_dir) or (args.exchange_dir and not args.process_role):
        parser.error("evolving snapshots require a coordinated role and private exchange directory")
    if args.process_role:
        expected = "planner-only" if args.process_role == "planner" else "estimator-only"
        if (args.ros_ablation != expected or args.estimator_device != "cpu"
                or args.device != ("cuda" if args.process_role == "planner" else "cpu")
                or bool(args.ros_plan_fixture) != (args.process_role == "planner" and not args.ros_evolving_snapshots)
                or args.write_plan_fixture):
            parser.error("split roles require CPU estimation and fixed-fixture GPU planning")
        work["diagnostics"] = args.process_role == "estimator"
    require_controller_idle()
    import os
    domain = 214 if args.process_role == "estimator" else 213
    if os.environ.get("ROS_DOMAIN_ID") != str(domain):
        raise RuntimeError(f"ROS workload comparison requires isolated domain {domain}")
    import rclpy
    import rosbag2_py
    import yaml
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from catheter_control.node import CatheterControlNode
    from catheter_control.orchestration.diagnostics import instrument_timer
    from catheter_control.planning.mppi import CatheterMppi
    from catheter_control.safety.validation import percentile_metrics
    parameters = probe_parameters(
        json.loads((args.session/"controller_manifest.json").read_text())["controller_parameters"],
        args.model_manifest, args.limits_file, args.device, args.estimator_device)
    parameters["planner_process_enabled"] = args.planner_process
    args.output.mkdir(parents=True, exist_ok=False)
    params_file = args.output/"parameters.yaml"
    params_file.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.session/"robot_bag"), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    replay_topics = INPUT_TOPICS + (("/catheter_mppi/reference_horizon",)
                                    if args.path_reference_commit else ())
    classes = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()
               if item.name in replay_topics or item.name == "/catheter_mppi/status"}
    if any(topic not in classes for topic in replay_topics):
        raise ValueError("bag lacks required replay input topics")
    events = []
    while reader.has_next():
        topic, payload, receipt = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize_message(payload, classes[topic])
        if (topic == "/catheter_mppi/status" or input_allowed(topic, message)
                or (args.path_reference_commit and topic == "/catheter_mppi/reference_horizon")):
            events.append((receipt, topic, message))
    if not events:
        raise ValueError("no recorded feedback")
    first = next(row[0] for row in events if row[1] in INPUT_TOPICS)
    requested_task_offset_s = args.start_offset_s if args.path_reference_commit else None
    if args.path_reference_commit:
        task = json.loads((args.session/"path_result.json").read_text())
        args.start_offset_s += (task["started_ros_ns"]-first)*1e-9
        if args.start_offset_s < 0:
            raise ValueError("path task precedes replay feedback")
    start, end = first+int(args.start_offset_s*1e9), first+int((args.start_offset_s+args.duration_s)*1e9)
    if end > events[-1][0]:
        raise ValueError("ROS replay window outside recording")
    targets = [] if args.path_reference_commit else [(row["ros_time_ns"], np.asarray(row["target_m"])) for row in
               map(json.loads, (args.session/"trials.jsonl").read_text().splitlines()) if row["event"] == "trial_started"]
    stop = threading.Event()
    meter, plans, failures, producer_lateness, warmup_rows = Measurements(), [], [], [], []
    sampled_node_timing = []
    epoch = None
    frozen_inputs = None
    fixture_sha256 = None
    fixture_written = False
    exchange_sequence = 0
    sent_snapshots = []
    qualification_pairs = {}  # First per two-second bucket plus latest; checked after timing.

    def measured():
        return epoch is not None and args.start_offset_s <= time.monotonic()-epoch <= args.start_offset_s+args.duration_s

    class Probe(CatheterControlNode):
        # Keep the actual callbacks and timing instruments; only replace the
        # disarmed planner's no-op with a pure, discarded plan computation.
        def _arm_cb(self, request, response):
            response.success = False
            response.message = "offline_probe_never_arms"
            return response

        def _publish_planner_snapshot(self, state, source_time, transmission_state=None):
            nonlocal exchange_sequence
            super()._publish_planner_snapshot(state, source_time, transmission_state)
            if not (args.ros_evolving_snapshots and args.process_role == "estimator" and epoch is not None):
                return
            from .replay_exchange import ReplaySnapshot, publish
            try:
                with self._lock:
                    position = self.position.copy() if self.position_valid else None
                    tip = None if self.observed_tip is None else self.observed_tip.copy()
                with evidence_lock:
                    target = None if evidence['target'] is None else evidence['target'].copy()
                    belief = copy.deepcopy(evidence['belief'])
                if state is None or source_time is None or any(v is None for v in (position, tip, target, belief)):
                    return
                exchange_sequence += 1
                snapshot = ReplaySnapshot(exchange_sequence, time.monotonic(), source_time,
                    contract_digest, (state, position, tip, target, belief))
                if measured():
                    size = meter.call('snapshot_encode_publish', state.timestamp_ns,
                                      publish, args.exchange_dir, snapshot)
                    sent_snapshots.append({'sequence': exchange_sequence, 'bytes': size,
                                           'source_ns': state.timestamp_ns})
                else:
                    publish(args.exchange_dir, snapshot)
            except Exception as error:
                failures.append(repr(error))
                stop.set()

        def _estimator_tick(self):
            if args.ros_evolving_snapshots and args.process_role == "planner":
                return
            if measured() and not work["estimator"]:
                return
            function = super()._estimator_tick
            return meter.call("ros_estimator_callback", 0, function) if measured() else function()

        def _process_encoder_sample(self, sample):
            function = super()._process_encoder_sample
            return (meter.call("ros_encoder_owner", sample[0], function, sample)
                    if sample is not None and measured() else function(sample))

        def _process_marker_sample(self, sample):
            function = super()._process_marker_sample
            return (meter.call("ros_marker_owner", sample[0], function, sample)
                    if sample is not None and measured() else function(sample))

        def _diagnostic_tick(self):
            if measured() and not work["diagnostics"]:
                return
            function = super()._diagnostic_tick
            if not measured():
                return function()
            result = meter.call("ros_diagnostic_callback", 0, function)
            sampled_node_timing.append(self._timing.latest())
            return result

        def _heartbeat_tick(self):
            function = super()._heartbeat_tick
            return meter.call("ros_idle_heartbeat_callback", 0, function) if measured() else function()

        @instrument_timer("plan")
        def _plan_tick(self):
            nonlocal frozen_inputs, fixture_written
            if args.process_role == "estimator":
                return  # CPU-only load process never performs an unused planner warmup.
            in_window = measured()
            plan_started_offset_s = None if epoch is None else time.monotonic()-epoch
            if in_window and not work["planner"]:
                return
            if epoch is None or (not in_window and warmup_rows):
                return
            if self.armed or self.mode_claimed or self.command_output_enabled:
                failures.append("non-actuating interlock violated")
                stop.set()
                return
            with self._lock:
                position = self.position.copy() if self.position_valid else None
                tip = None if self.observed_tip is None else self.observed_tip.copy()
                captured_reference = self.path_reference if args.path_reference_commit else None
            with self._snapshot_lock:
                root = self._planner_snapshot
                source_time = self._planner_snapshot_source_time
            with evidence_lock:
                target, belief = evidence["target"], evidence["belief"]
            tangent = None
            if args.path_reference_commit:
                if captured_reference is None or root is None:
                    return
                try:
                    target, tangent = captured_reference.sample(
                        root.timestamp_ns, self.planner.config.horizon_steps,
                        self.planner.config.step_s, self._steady())
                    if captured_reference.final_hold:
                        tangent = None
                except ValueError as error:
                    if in_window:
                        failures.append("path_sampling:"+str(error))
                    return
            ipc_ms, publication_age_ms, state_age_ms, snapshot_sequence = 0., None, None, None
            header_age_ms = None
            if args.planner_process and source_time is not None:
                state_age_ms = 1000*(time.monotonic()-source_time)
                header_age_ms = (self.get_clock().now().nanoseconds-root.timestamp_ns)*1e-6 if root is not None else None
            if args.ros_evolving_snapshots:
                from .replay_exchange import receive
                try:
                    ipc_started = time.monotonic()
                    snapshot = receive(args.exchange_dir, contract_digest)
                    ipc_ms = 1000*(time.monotonic()-ipc_started)
                    if snapshot is None:
                        if in_window:
                            raise RuntimeError("missing evolving estimator snapshot")
                        return
                    publication_age_ms = 1000*(time.monotonic()-snapshot.published_monotonic_s)
                    state_age_ms = 1000*(time.monotonic()-snapshot.source_monotonic_s)
                    if state_age_ms > 210:
                        raise RuntimeError("evolving estimator snapshot older than 210 ms test bound")
                    snapshot_sequence = snapshot.sequence
                    root, position, tip, target, belief = snapshot.inputs
                    header_age_ms = (self.get_clock().now().nanoseconds-root.timestamp_ns)*1e-6
                except Exception as error:
                    failures.append(repr(error))
                    stop.set()
                    return
            if root is None or position is None or tip is None or target is None or belief is None:
                return
            if in_window and args.write_plan_fixture:
                save_plan_fixture(args.write_plan_fixture,
                                  (root, position, tip, target, copy.deepcopy(belief)), fixture_contract)
                fixture_written = True
                stop.set()
                return
            if args.ros_plan_fixture or (in_window and args.ros_ablation == "planner-only" and not args.ros_evolving_snapshots):
                if frozen_inputs is None:
                    # Replace-only snapshots are immutable to the estimator.
                    # Freeze all plan evidence, not merely the model state.
                    frozen_inputs = (root, position.copy(), tip.copy(), target.copy(), copy.deepcopy(belief))
                root, position, tip, target, belief = frozen_inputs
            # Reconstruct the small planner wrapper outside timing to ensure
            # identical seeded proposals and no accumulated warm-start/capture
            # state. The warmed rollout backend remains the same object.
            planner = (CatheterMppi(self.planner_runtime, self.contract, self.planner.config)
                       if args.ros_plan_fixture else probe_planner)
            try:
                started = time.monotonic()-ipc_ms*.001
                transfer_started = time.monotonic()
                cpu_root = root
                if not args.ros_plan_fixture and not args.planner_process:
                    root = self._planner_root_on_device(root)
                    if args.ros_evolving_snapshots:
                        import torch
                        torch.cuda.synchronize(self.planner_runtime.device)
                snapshot_transfer_ms = 1000*(time.monotonic()-transfer_started)
                # Production snapshot publication handles cross-device copies.
                # A loaded fixed fixture already resides on the planner device.
                kwargs = dict(observed_tip_base_m=tip, transmission_state=belief, deadline_started_s=started)
                if args.path_reference_commit:
                    kwargs["target_tangent_base"] = tangent
                if in_window:
                    plan = meter.call("ros_probe_planner", root.timestamp_ns, planner.plan,
                                      root, position, target, **kwargs)
                else:
                    plan = planner.plan(root, position, target, **kwargs)
                row = {"source_ns": root.timestamp_ns, "elapsed_ms": 1000*(time.monotonic()-started),
                              "plan_started_offset_s": plan_started_offset_s,
                              "position_mm_deg_mm": position.tolist(),
                              "snapshot_transfer_ms": snapshot_transfer_ms,
                              "snapshot_receive_ms": ipc_ms, "snapshot_sequence": snapshot_sequence,
                              "snapshot_publication_age_ms": publication_age_ms,
                              "snapshot_source_age_ms": state_age_ms,
                              "snapshot_header_age_ms": header_age_ms,
                              "snapshot_arrival_age_at_plan_end_ms": (None if state_age_ms is None
                                  else state_age_ms + 1000*(time.monotonic()-started)),
                              "reason": plan.reason, "valid": plan.valid,
                              "rollout_ms": plan.rollout_ms, "sample_projection_ms": plan.sample_projection_ms,
                              "cost_weighting_ms": plan.cost_weighting_ms}
                if args.planner_process:
                    row["process_timings_ms"] = {
                        name: seconds*1000 for name, seconds
                        in planner.process_timings_s.items()}
                if args.path_reference_commit:
                    from .path_commit_evidence import commit_evidence
                    # Exact immutable versions under the production exchange lock.
                    # Evaluate the same production predicate, but never commit a command.
                    with self._lock:
                        current_reference = self.path_reference
                        gate_now = self._steady()
                        gate_error = None
                        try:
                            compatible = self._planner_reference_matches_locked(
                                "PATH", target, captured_reference, root.timestamp_ns, gate_now)
                        except ValueError as error:
                            compatible, gate_error = False, str(error)
                        candidate_decisions = {}
                        for name, tangent_tolerance in (
                                ("strict", 1e-4), ("bounded", 1e-3),
                                ("tangent_relaxed", 2e-3)):
                            error_text = None
                            try:
                                accepted = (
                                    self.reference_source == "PATH"
                                    and captured_reference.compatible_with(
                                        current_reference, root.timestamp_ns,
                                        self.planner.config.horizon_steps,
                                        self.planner.config.step_s, gate_now,
                                        bounded_refresh=name != "strict",
                                        bounded_tangent_tolerance=tangent_tolerance))
                            except ValueError as error:
                                accepted, error_text = False, str(error)
                            candidate_decisions[name] = (accepted, error_text)
                    row["path_commit"] = commit_evidence(
                        captured_reference, current_reference, root.timestamp_ns,
                        self.planner.config.horizon_steps, self.planner.config.step_s,
                        gate_now, compatible, gate_error)
                    for name, (accepted, error_text) in candidate_decisions.items():
                        row[f"{name}_path_commit"] = commit_evidence(
                            captured_reference, current_reference, root.timestamp_ns,
                            self.planner.config.horizon_steps, self.planner.config.step_s,
                            gate_now, accepted, error_text)
                        row[f"{name}_would_pass_reference_and_plan_validity"] = bool(
                            accepted and error_text is None and plan.valid)
                    row["proposed_command"] = plan.command_logical_velocity.tolist()
                    row["would_pass_reference_and_plan_validity"] = bool(
                        compatible and gate_error is None and plan.valid)
                    row["command_executed"] = False
                if in_window:
                    plans.append(row)
                    if args.ros_evolving_snapshots:
                        pair = (cpu_root, root)
                        bucket = int((plan_started_offset_s-args.start_offset_s)/2)
                        qualification_pairs.setdefault(bucket, pair)
                        qualification_pairs['latest'] = pair
                else:
                    warmup_rows.append(row)
                    probe_planner.reset()
            except Exception as error:
                failures.append(repr(error))
                stop.set()

    evidence_lock = threading.Lock()
    evidence = {"target": None, "belief": None}
    # Command publications are additionally remapped to local blocked topics;
    # no output consumer or manager is started on this isolated DDS domain.
    rclpy.init(args=["--ros-args", "--params-file", str(params_file),
                    "-r", "/teleop/control:=/offline_compute_probe/blocked_control",
                    "-r", "/teleop/event:=/offline_compute_probe/blocked_event"])
    node = feed = executor = None
    producer = None
    try:
        node = Probe()
        runtime_devices = {"planner_device": str(node.planner_runtime.device),
                           "planner_dtype": str(node.planner_runtime.dtype),
                           "estimator_device": str(node.runtime.device),
                           "estimator_dtype": str(node.runtime.dtype)}
        if (node.planner_runtime.device.type != args.device
                or node.runtime.device.type != (args.estimator_device or args.device)
                or str(node.runtime.dtype) != "torch.float32"):
            raise RuntimeError(f"requested runtime assignment not established: {runtime_devices}")
        fixture_contract = {"model_sha256": hashlib.sha256(Path(args.model_manifest).read_bytes()).hexdigest(),
                            "limits_sha256": hashlib.sha256(Path(args.limits_file).read_bytes()).hexdigest(),
                            "planner_config": json.loads(json.dumps(asdict(node.planner.config)))}
        contract_digest = hashlib.sha256(json.dumps(fixture_contract, sort_keys=True).encode()).hexdigest()
        if args.ros_plan_fixture:
            frozen_inputs, fixture_sha256 = load_plan_fixture(
                args.ros_plan_fixture, fixture_contract, args.device)
        probe_planner = (node.planner if args.planner_process else
                         CatheterMppi(node.planner_runtime, node.contract, node.planner.config))
        feed = Node("offline_compute_feed", use_global_arguments=False)
        publishers = {topic: feed.create_publisher(classes[topic], topic,
                      10 if topic.endswith(("marker_status", "reference_horizon")) else qos_profile_sensor_data)
                      for topic in replay_topics}
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        executor.add_node(feed)
        # Bounded discovery time, no actuator link or service readiness.
        executor.spin_once(timeout_sec=.5)
        epoch = (coordinated_epoch(args.coordination_dir, args.process_role)
                 if args.process_role else time.monotonic())
        now = time.monotonic()
        base_ns = feed.get_clock().now().nanoseconds+round((epoch-now)*1e9)

        def produce():
            try:
                target_index = 0
                for receipt, topic, original in events:
                    if receipt > end or stop.wait(max(0., epoch+(receipt-first)*1e-9-time.monotonic())):
                        break
                    if measured():
                        producer_lateness.append(max(0., 1000*(time.monotonic()-epoch)-(receipt-first)*1e-6))
                    with evidence_lock:
                        while target_index < len(targets) and targets[target_index][0] <= receipt:
                            evidence["target"] = targets[target_index][1]
                            target_index += 1
                        if topic == "/catheter_mppi/status":
                            values = {value.key: value.value for status in original.status for value in status.values}
                            if "engaged_gain_mean" in values:
                                evidence["belief"] = recorded_belief(values)
                            continue
                    message = copy.deepcopy(original)
                    source = message.header.stamp.sec*10**9+message.header.stamp.nanosec
                    message.header.stamp.sec, message.header.stamp.nanosec = divmod(base_ns+source-first, 10**9)
                    publishers[topic].publish(message)
            except Exception as error:
                failures.append(repr(error))
                stop.set()

        producer = threading.Thread(target=produce)
        if args.path_reference_commit:
            print(json.dumps({"stage": "path_commit_replay_ready",
                              "prefix_s": args.start_offset_s,
                              "measurement_s": args.duration_s,
                              "command_output_enabled": False}), flush=True)
        producer.start()
        while rclpy.ok() and not stop.is_set() and time.monotonic()-epoch < args.start_offset_s+args.duration_s:
            executor.spin_once(timeout_sec=.05)
    finally:
        stop.set()
        if producer is not None:
            producer.join(5)
        if executor is not None:
            executor.shutdown(timeout_sec=10)
        if feed is not None:
            feed.destroy_node()
        if node is not None:
            if args.planner_process:
                node.planner.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    qualification = {'passed': False, 'rows': []}
    if args.ros_evolving_snapshots and args.process_role == 'planner':
        from .benchmark_compute_isolation import state_record
        for cpu_root, gpu_root in qualification_pairs.values():
            same = (json.dumps(state_record(cpu_root), sort_keys=True)
                    == json.dumps(state_record(gpu_root.clone_to('cpu')), sort_keys=True))
            a = node.runtime.markers_for_state(cpu_root).detach().cpu().numpy()
            b = node.planner_runtime.markers_for_state(gpu_root).detach().cpu().numpy()
            error_mm = float(1000*np.linalg.norm(a-b, axis=-1).max())
            rates = np.zeros((3, 4, 3), dtype=np.float32)
            rates[1, :, 0], rates[2, :, 2] = .15, .15
            dt = np.full(4, .2, dtype=np.float32)
            a_tip = node.runtime.predict_control_sequence_coarse(cpu_root, rates, dt).tip_base_m.detach().cpu().numpy()
            b_tip = node.planner_runtime.predict_control_sequence_coarse(gpu_root, rates, dt).tip_base_m.detach().cpu().numpy()
            rollout_error_mm = float(1000*np.linalg.norm(a_tip-b_tip, axis=-1).max())
            qualification['rows'].append({'source_ns': cpu_root.timestamp_ns,
                'state_roundtrip_equal': same, 'marker_max_error_mm': error_mm,
                'rollout_tip_max_error_mm': rollout_error_mm})
        qualification['passed'] = bool(qualification['rows']) and all(
            row['state_roundtrip_equal'] and row['marker_max_error_mm'] <= .01
            and row['rollout_tip_max_error_mm'] <= .05 for row in qualification['rows'])
    report = {"scope": __doc__, "command_output_enabled": False, "ever_armed": False,
              "window": {'start_offset_s': args.start_offset_s, 'duration_s': args.duration_s},
              "evolving_snapshots": args.ros_evolving_snapshots,
              "production_planner_process": args.planner_process,
              "path_reference_commit_replay": args.path_reference_commit,
              "requested_path_task_offset_s": requested_task_offset_s,
              "handoff_transport": ('private single-inflight socketpair' if args.planner_process else
                                    'private latest-only atomic file in /tmp' if args.exchange_dir else None),
              "sent_snapshots": sent_snapshots, "handoff_qualification": qualification,
              "runtime_devices": runtime_devices,
              "ros_ablation": args.ros_ablation, "measurement_work": work,
              "plan_fixture_sha256": fixture_sha256, "plan_fixture_written": fixture_written,
              "planner_fixture_contract": fixture_contract,
              "frozen_planner_source_ns": None if frozen_inputs is None else frozen_inputs[0].timestamp_ns,
              "domain_id": domain, "process_role": args.process_role,
              "replay_epoch_monotonic_s": epoch,
              "stages": meter.summaries(), "plans": plans, "failures": failures,
              "pre_window_warmup": warmup_rows,
              "sampled_node_latest_timing_ms": sampled_node_timing,
              "deadline_misses": sum(row["elapsed_ms"] > 1000*probe_planner.config.planning_deadline_s for row in plans),
              "producer_lateness_ms": percentile_metrics(producer_lateness) if producer_lateness else {},
              "limitations": ["No active takeup arbitration, heartbeat command work or mode gates",
                              "Recorded gain bounds with an all-engaged zero-gap planning fixture",
                              "Feedback source stamps rebased preserving source/receipt offsets",
                              "Node latest-timing rows are sampled cached values, not independent callback durations",
                              "No camera SDK, serial driver or manager process; DDS feed is in-process"]}
    if args.path_reference_commit:
        report["path_commit_summary"] = {
            "compared": len(plans),
            "compatible": sum(row["path_commit"]["reference_compatible"] for row in plans),
            "reference_rejected": sum(not row["path_commit"]["reference_compatible"] for row in plans),
            "gate_errors": sum(row["path_commit"]["gate_error"] is not None for row in plans),
            "executed_commands": 0,
        }
        for name, tangent_tolerance in (("strict", 1e-4), ("bounded", 1e-3), ("tangent_relaxed", 2e-3)):
            report[f"{name}_path_commit_summary"] = {
                "compared": len(plans),
                "compatible": sum(row[f"{name}_path_commit"]["reference_compatible"] for row in plans),
                "reference_rejected": sum(not row[f"{name}_path_commit"]["reference_compatible"] for row in plans),
                "gate_errors": sum(row[f"{name}_path_commit"]["gate_error"] is not None for row in plans),
                "rescued_valid_plans": sum(
                    row[f"{name}_would_pass_reference_and_plan_validity"]
                    and not row["strict_would_pass_reference_and_plan_validity"] for row in plans),
                "executed_commands": 0,
                "moving_position_coordinate_tolerance_mm": .001 if name == "strict" else .01,
                "moving_tangent_component_tolerance": tangent_tolerance,
                "maximum_progress_advance_mm": None if name == "strict" else .1,
                "maximum_progress_rewind_mm": None if name == "strict" else .001,
            }
        report["tangent_relaxed_path_commit_summary"]["additional_rescued_valid_plans"] = sum(
            row["tangent_relaxed_would_pass_reference_and_plan_validity"]
            and not row["bounded_would_pass_reference_and_plan_validity"] for row in plans)
        report["limitations"].append(
            "path_commit uses the shared production bounded policy (moving tangent bound 0.002). "
            "strict_path_commit preserves the historical predicate; rescue counts compare against it. "
            "All predicates use the same captured/current pair and evaluation time; "
            "tangent-relaxed changes only the moving tangent component bound from 0.001 to 0.002")
        report["limitations"].append(
            "Exact replay captured/current references, not reconstruction of historical callback scheduling; "
            "would-pass includes only reference compatibility and plan validity, not all production readiness gates")
    report["limitations"].extend([
        "Ablations apply only in the measured window; the full prefix and warmup are retained",
        "Disabled work leaves no-op timer dispatch and sensor subscription/feed overhead",
        "Planner-only freezes plan inputs; it is not trajectory or closed-loop equivalence",
        "An estimator callback begun before the boundary may finish inside the window"])
    if args.ros_plan_fixture:
        report["limitations"].append(
            "Fixed fixture in warmup and measurement; fresh seeded planner wrapper per solve outside timing; no warm starts")
    if args.ros_evolving_snapshots or args.planner_process:
        if args.process_role == 'planner' or args.planner_process:
            report['plan_coverage'] = plan_coverage(plans, args.start_offset_s, args.duration_s,
                float(node.get_parameter('plan_rate_hz').value),
                allow_path_transitions=args.path_reference_commit)
        positions = [list(message.data)[:3] for receipt, topic, message in events
                     if start <= receipt <= end and topic == '/device/state'
                     and message.predicate == ord('P')]
        report['recorded_position_window'] = {
            'samples': len(positions), 'units': ['mm', 'deg', 'mm'],
            'minimum': np.min(positions, axis=0).tolist() if positions else None,
            'maximum': np.max(positions, axis=0).tolist() if positions else None}
        report['handoff_timing_ms'] = {
            key: percentile_metrics([row[key] for row in plans if row[key] is not None])
            for key in ('snapshot_receive_ms', 'snapshot_transfer_ms',
                        'snapshot_publication_age_ms', 'snapshot_source_age_ms',
                        'snapshot_header_age_ms', 'snapshot_arrival_age_at_plan_end_ms')
            if any(row[key] is not None for row in plans)}
        report['limitations'].extend([
            ('Private socket worker uses production scheduling; executable deadline includes IPC'
             if args.planner_process else
             'Atomic /tmp file serialization is an experimental IPC transport, not production scheduling'),
            'Complete input bundles use recorded target/gain evidence, not live control belief',
            ('Production worker numerical conformance is tested separately, not by this timing replay'
             if args.planner_process else
             'GPU transfer synchronization is included; first/latest state and FK checks occur after measurement')])
    report['plan_validity'] = plan_outcomes(plans, allow_path_transitions=args.path_reference_commit)
    valid_elapsed = [row['elapsed_ms'] for row in plans if row['valid']]
    report['valid_plan_elapsed_ms'] = percentile_metrics(valid_elapsed) if valid_elapsed else {}
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    if args.path_reference_commit:
        print(json.dumps({"report": str(args.output/"report.json"),
                          "path_commit_summary": report["path_commit_summary"],
                          "bounded_path_commit_summary": report["bounded_path_commit_summary"],
                          "tangent_relaxed_path_commit_summary": report["tangent_relaxed_path_commit_summary"],
                          "plan_validity": report["plan_validity"]}), flush=True)
    if args.write_plan_fixture:
        if failures or not fixture_written:
            raise RuntimeError("planner fixture capture failed")
        return
    require_measurement_work(args.ros_ablation, report["stages"], plans, failures,
                             allow_path_transitions=args.path_reference_commit)
    if args.path_reference_commit and not report["plan_coverage"]["passed"]:
        raise RuntimeError(f"path replay cadence incomplete: {report['plan_coverage']}")


if __name__ == "__main__":
    main()
