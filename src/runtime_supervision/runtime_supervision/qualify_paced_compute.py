"""Bounded non-actuating O2 replay: evolving CPU64 snapshots and paced GPU32 MPPI.

Recorded receipt-time pacing is not a ROS executor or closed-loop plant replay.
Recorded gain posteriors are workload inputs, not re-estimated online beliefs.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, fields, replace
import json
from pathlib import Path
import threading
import time

import numpy as np
import torch

from catheter_control.orchestration.runtime import load_runtime_pair
from catheter_control.planning.mppi import CatheterMppi, MppiConfig
from catheter_control.safety.hardware_contract import load_hardware_contract
from catheter_control.safety.validation import percentile_metrics
from catheter_control.transmission.backlash import BacklashSnapshot
from catheter_control.transmission.engaged_gain import EngagedGainConfig, EngagedGainEstimator
from .benchmark_compute_isolation import differences, state_record
from .compute_profile import Measurements, read_events, replay, require_controller_idle


def recorded_config(parameters):
    """Read planner fields from recorded ROS parameters, retaining dataclass defaults."""
    aliases = {"step_s": "rollout_step_s"}
    values = {}
    for field in fields(MppiConfig):
        key = aliases.get(field.name, field.name if field.name in (
            "samples", "horizon_steps", "planning_deadline_s") else "mppi_"+field.name)
        if key in parameters:
            values[field.name] = parameters[key]
    # The production node explicitly disables these two policies.
    values.update(transmission_aware_rollout=False, rotation_direction_latch=False)
    return MppiConfig(**values)


def recorded_belief(values):
    """Owned planner-facing gain posterior from recorded diagnostics, no invented updates."""
    gain = EngagedGainEstimator(EngagedGainConfig(enabled=True)).snapshot()
    required = ("engaged_gain_mean", "engaged_gain_lower", "engaged_gain_upper",
                "engaged_gain_status", "backlash_engaged_direction")
    if any(key not in values for key in required):
        raise ValueError("recorded status lacks required gain/direction fields")
    arrays = {name: np.asarray(json.loads(values["engaged_gain_"+name]), dtype=float)
              for name in ("mean", "lower", "upper")}
    if any(array.shape != (3, 2) or not np.isfinite(array).all() or np.any(array <= 0)
           for array in arrays.values()):
        raise ValueError("invalid recorded gain posterior")
    direction = np.asarray(json.loads(values["backlash_engaged_direction"]), dtype=np.int8)
    if direction.shape != (3,) or np.any(np.abs(direction) > 1):
        raise ValueError("invalid recorded engagement direction")
    gain = replace(gain, **arrays,
                   status=tuple(tuple(row) for row in json.loads(values["engaged_gain_status"])),
                   active_direction=direction.copy())
    # All-engaged workload evaluates the full planner batch; it is not a
    # reconstruction of take-up arbitration or permission to command motion.
    return BacklashSnapshot(
        width_rad=np.zeros(3), width_positive_rad=np.zeros(3), width_negative_rad=np.zeros(3),
        remaining_rad=np.zeros(3), motion_direction=direction.copy(), engaged_direction=direction,
        confidence=np.ones(3), confirmation_count=np.ones(3, dtype=np.int32),
        phase=("ENGAGED", "UNKNOWN", "ENGAGED"), engaged_gain=gain)


def run_case(bundle, planner_bundle, contract, config, events, targets, start, end, rate, concurrent):
    runtime = bundle.runtime
    lock = threading.Lock()
    stop = threading.Event()
    latest = None
    position = target = belief = observed_tip = None
    target_index = 0
    epoch = None
    rows, errors, event_lateness, publication_ms = [], [], [], []
    skipped_slots = 0
    planner = CatheterMppi(planner_bundle.runtime, contract, config)

    def solve(snapshot):
        root, receipt, pos, goal, transmission, tip = snapshot
        started = time.perf_counter()
        root = root.clone_to(planner_bundle.runtime.device, dtype=planner_bundle.runtime.dtype)
        transfer_ms = 1000*(time.perf_counter()-started)
        plan = planner.plan(root, pos, goal, transmission_state=transmission,
                            observed_tip_base_m=tip, deadline_started_s=started)
        ended = time.perf_counter()
        return {"source_ns": root.timestamp_ns, "snapshot_receipt_ns": receipt,
                "snapshot_age_ms": 1000*(started-epoch)-(root.timestamp_ns-start)*1e-6,
                "elapsed_ms": 1000*(ended-started), "transfer_ms": transfer_ms,
                "reason": plan.reason, "valid": plan.valid,
                "gain_scenarios": plan.engaged_gain_scenario_count}

    def worker():
        nonlocal skipped_slots
        slot = 0
        slots = int(np.ceil((end-start)*1e-9*rate))
        try:
            while slot < slots and not stop.is_set():
                due = epoch+slot/rate
                if stop.wait(max(0., due-time.perf_counter())):
                    break
                with lock:
                    snapshot = latest
                if snapshot is not None:
                    row = solve(snapshot)
                    row["timer_lateness_ms"] = max(0., 1000*(time.perf_counter()-due)-row["elapsed_ms"])
                    rows.append(row)
                slot += 1
                # Skip missed timer slots rather than accumulate stale solves.
                while slot < slots and epoch+slot/rate < time.perf_counter():
                    slot += 1
                    skipped_slots += 1
        except BaseException as error:
            errors.append(repr(error))
            stop.set()

    thread = threading.Thread(target=worker) if concurrent else None

    def begin():
        nonlocal epoch
        if concurrent:
            if latest is None:
                raise ValueError("prefix did not establish snapshot, target and gain belief")
            # Warm all three-scenario allocations before the paced window.
            epoch = time.perf_counter()
            solve(latest)
            planner.reset()
        epoch = time.perf_counter()
        if thread:
            thread.start()

    def before(event):
        nonlocal position, target, target_index, belief, observed_tip
        if event.receipt_ns >= start:
            due = epoch+(event.receipt_ns-start)*1e-9
            time.sleep(max(0., due-time.perf_counter()))
            event_lateness.append(max(0., 1000*(time.perf_counter()-due)))
        if errors:
            raise RuntimeError(errors[0])
        while target_index < len(targets) and targets[target_index][0] <= event.receipt_ns:
            target = targets[target_index][1]
            target_index += 1
        if event.kind == "position":
            position = event.values.copy()
        elif event.kind == "status" and "engaged_gain_mean" in event.values:
            belief = recorded_belief(event.values)

    def publish(stage, event, result):
        nonlocal latest, observed_tip
        if stage == "marker" and result.accepted:
            observed_tip = np.asarray(event.values[-1]).copy()
        if stage == "before_marker" or any(value is None for value in (position, target, belief, observed_tip)):
            return
        started = time.perf_counter()
        snapshot = (runtime.clone_state(), event.receipt_ns, position.copy(), target.copy(), belief, observed_tip.copy())
        with lock:
            latest = snapshot
        if event.receipt_ns >= start:
            publication_ms.append(1000*(time.perf_counter()-started))

    meter = Measurements()
    try:
        _, corrections = replay(runtime, None, None, events, targets, start_ns=start, end_ns=end,
                                measured=meter, before_event=before, on_window_start=begin,
                                on_estimator_event=publish)
        if epoch is not None:
            stop.wait(max(0., epoch+(end-start)*1e-9-time.perf_counter()))
    finally:
        stop.set()
        if thread and thread.ident is not None:
            thread.join(10)
            if thread.is_alive():
                raise RuntimeError("planner failed to stop")
    if errors:
        raise RuntimeError(errors[0])
    return {"stages": meter.summaries(), "plans": rows, "marker_updates": corrections,
            "skipped_planner_slots": skipped_slots,
            "owned_snapshot_publication_ms": percentile_metrics(publication_ms) if publication_ms else {},
            "event_lateness_ms": percentile_metrics(event_lateness),
            "final_state": state_record(runtime.clone_state()),
            "deadline_misses": sum(row["elapsed_ms"] > 1000*config.planning_deadline_s for row in rows),
            "summaries": {key: percentile_metrics([row[key] for row in rows]) for key in
                          ("elapsed_ms", "transfer_ms", "snapshot_age_ms", "timer_lateness_ms")} if rows else {}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--model-manifest", required=True)
    parser.add_argument("--limits-file", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start-offset-s", type=float, default=40.)
    parser.add_argument("--duration-s", type=float, default=10.)
    args = parser.parse_args(argv)
    if not np.isfinite([args.start_offset_s, args.duration_s]).all() or args.start_offset_s < 0 or not 0 < args.duration_s <= 30:
        parser.error("finite nonnegative offset and duration in (0,30] required")
    require_controller_idle()
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA required; no fallback")
    args.output.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    parameters = json.loads((args.session/"controller_manifest.json").read_text())["controller_parameters"]
    config = recorded_config(parameters)
    if config.samples != 512 or not config.engaged_gain_scenarios:
        raise ValueError("qualification requires recorded 512 samples and gain scenarios")
    rate = float(parameters["plan_rate_hz"])
    if not np.isfinite(rate) or rate <= 0 or config.planning_deadline_s >= 1./rate:
        raise ValueError("recorded plan rate/deadline is invalid")
    events = [event for event in read_events(args.session, include_status=True) if event.kind != "plan"]
    targets = [(row["ros_time_ns"], np.asarray(row["target_m"])) for row in
               map(json.loads, (args.session/"trials.jsonl").read_text().splitlines()) if row["event"] == "trial_started"]
    start = events[0].receipt_ns+int(args.start_offset_s*1e9)
    end = start+int(args.duration_s*1e9)
    if end > events[-1].receipt_ns:
        raise ValueError("window outside recording")
    contract = load_hardware_contract(args.limits_file, "imricor_test")
    requested = np.asarray(parameters["controller_velocity_max"])
    upper = np.where(requested < 0, contract.velocity_max, requested)
    if np.any(upper > contract.velocity_max):
        raise ValueError("recorded velocity cap exceeds hardware limits")
    contract = replace(contract, velocity_max=upper, velocity_min=np.minimum(contract.velocity_min, upper))
    cases = {}
    for name, concurrent in (("cpu64_reference", False), ("cpu64_gpu32_paced", True)):
        estimator, planner = load_runtime_pair(args.model_manifest, estimator_device="cpu",
                                              planner_device="cuda:0", estimator_dtype="float64",
                                              options={"marker_estimator": "ukf", "adaptation_enabled": False})
        cases[name] = run_case(estimator, planner, contract, config, events, targets, start, end,
                              rate, concurrent)
        print("completed "+name, flush=True)
    mismatches = differences(cases["cpu64_reference"]["final_state"], cases["cpu64_gpu32_paced"]["final_state"])
    decisions = lambda case: [(row["source_ns"], row["accepted"], row["reason"], row["observable_rank"])
                             for row in case["marker_updates"]]
    paced = cases["cpu64_gpu32_paced"]
    decisions_equal = decisions(cases["cpu64_reference"]) == decisions(paced)
    report = {"scope": __doc__, "session": str(args.session),
              "window": {"start_ns": start, "end_ns": end},
              "manifest_sha256": estimator.identity.manifest_sha256,
              "config": asdict(config), "rate_hz": parameters["plan_rate_hz"], "cases": cases,
              "state_differences": mismatches,
              "marker_decisions_equal": decisions_equal,
              "offline_gate_passed": not mismatches and decisions_equal and bool(paced["plans"])
                  and paced["deadline_misses"] == 0 and paced["skipped_planner_slots"] == 0
                  and all(row["valid"] and row["gain_scenarios"] == 3 for row in paced["plans"]),
              "qualification": "OFFLINE_ONLY_NOT_ROS_OR_HARDWARE_QUALIFIED"}
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    print("report: "+str(args.output/"report.json"), flush=True)


if __name__ == "__main__":
    main()
