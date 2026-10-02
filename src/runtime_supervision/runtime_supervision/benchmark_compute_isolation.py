"""O2 recorded estimator replay with isolated/concurrent frozen GPU planning.

Non-actuating workload comparison, not a reproduction of ROS/GIL scheduling.
The concurrent planner consumes a fixed owned snapshot, not live estimator
outputs; its transfer is measured and included in solve wall time.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, fields, is_dataclass, replace
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
from .compute_profile import Measurements, read_events, replay, require_controller_idle


def state_record(value):
    """Serialize all snapshot fields, including optional memory and J/RLS."""
    if torch.is_tensor(value):
        return value.detach().cpu().tolist()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if is_dataclass(value):
        return {field.name: state_record(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, dict):
        return {key: state_record(child) for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return [state_record(child) for child in value]
    if hasattr(value, "state_dict"):
        return state_record(value.state_dict())
    return value


def differences(a, b, path=""):
    """Declared numerical tolerances; discrete state/reasons remain exact."""
    if isinstance(a, dict):
        if a.keys() != b.keys():
            return [{"field": path, "reason": "keys"}]
        return [row for key in a for row in differences(a[key], b[key], f"{path}.{key}")]
    if isinstance(a, list):
        try:
            x, y = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        except (ValueError, TypeError):
            return [] if a == b else [{"field": path, "reason": "discrete"}]
        if x.shape == y.shape and np.allclose(x, y, rtol=3e-5, atol=2e-6, equal_nan=True):
            return []
        return [{"field": path, "reason": "numeric", "max_abs": float(np.nanmax(abs(x-y))) if x.shape == y.shape else None}]
    if isinstance(a, float):
        return [] if np.isclose(a, b, rtol=3e-5, atol=2e-6, equal_nan=True) else [{"field": path, "reason": "numeric", "expected": a, "actual": b}]
    return [] if a == b else [{"field": path, "reason": "discrete", "expected": a, "actual": b}]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--model-manifest", type=Path, required=True)
    parser.add_argument("--limits-file", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start-offset-s", type=float, default=39)
    parser.add_argument("--duration-s", type=float, default=2)
    parser.add_argument("--samples", type=int, default=512)
    parser.add_argument("--estimator-dtype", choices=("float32", "float64"), default="float32",
                        help="CPU estimator precision; CUDA reference/planner remain float32")
    args = parser.parse_args(argv)
    if (not np.isfinite([args.start_offset_s, args.duration_s]).all()
            or args.start_offset_s < 0 or not 0 < args.duration_s <= 10 or args.samples < 8):
        parser.error("require nonnegative finite offset, duration in (0,10], samples >=8")
    if not torch.cuda.is_available():
        raise RuntimeError("O2 comparison requires CUDA; no CPU fallback")
    require_controller_idle()
    args.output.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    events = read_events(args.session)
    start = events[0].receipt_ns+int(args.start_offset_s*1e9)
    end = start+int(args.duration_s*1e9)
    if start > events[-1].receipt_ns:
        raise ValueError("window outside recording")
    contract = load_hardware_contract(args.limits_file, "imricor_test")
    upper = contract.velocity_max.copy()
    upper[1] = 0
    contract = replace(contract, velocity_max=upper, velocity_min=np.minimum(contract.velocity_min, upper))
    config = MppiConfig(samples=args.samples, horizon_steps=4,
                        point_rollout_step_s=.2, point_rollout_coarse_steps=True,
                        seed=17, planning_deadline_s=.06)
    cases = {}
    fixture = None
    for device, concurrent in (("cpu", False), ("cuda:0", False),
                               ("cuda:0", True), ("cpu", True)):
        estimator, planner_bundle = load_runtime_pair(
            str(args.model_manifest), estimator_device=device, planner_device="cuda:0",
            estimator_dtype=args.estimator_dtype if device == "cpu" else "float32",
            options={"marker_estimator": "ukf", "adaptation_enabled": False})
        runtime = estimator.runtime
        # Reuse O0's prefix/causal deferral/thinning, without artificial plans.
        owner_events = [event for event in events if event.kind != "plan"]
        if fixture is None:
            preparation = load_runtime_pair(
                str(args.model_manifest), estimator_device="cpu", planner_device="cpu",
                options={"marker_estimator": "ukf", "adaptation_enabled": False})[0].runtime
            replay(preparation, None, None, owner_events, [], start_ns=start,
                   end_ns=start-1, measured=Measurements())
            position = next(event.values for event in reversed(events)
                            if event.kind == "position" and event.receipt_ns < start)
            target = preparation.current_markers()[-1].detach().cpu().numpy()+np.array([0., 0., .005])
            fixture = preparation.clone_state(), position, target
        canonical, position, target = fixture
        if device == "cuda:0":
            canonical = canonical.clone_to("cuda:0")
        elif args.estimator_dtype == "float64":
            canonical = canonical.clone_to("cpu", dtype=torch.float64)
        stop, ready = threading.Event(), threading.Event()
        plan_rows, errors = [], []
        def plan_loop():
            try:
                while not stop.is_set():
                    planner = CatheterMppi(planner_bundle.runtime, contract, config)
                    started = time.perf_counter()
                    transfer_started = time.perf_counter()
                    root = canonical if device == "cuda:0" else canonical.clone_to("cuda:0", dtype=torch.float32)
                    transfer_ms = 1000*(time.perf_counter()-transfer_started)
                    plan = planner.plan(root, position, target, deadline_started_s=started)
                    elapsed = 1000*(time.perf_counter()-started)
                    if ready.is_set():
                        plan_rows.append({"elapsed_ms": elapsed, "transfer_ms": transfer_ms,
                                          "reason": plan.reason, "valid": plan.valid})
                    ready.set()
            except BaseException as error:
                errors.append(repr(error))
                ready.set()
        thread = threading.Thread(target=plan_loop) if concurrent else None
        def begin_window():
            if thread is None:
                return
            thread.start()
            if not ready.wait(10) or errors:
                stop.set()
                thread.join(10)
                raise RuntimeError(f"planner warmup failed: {errors}")
        meter = Measurements()
        try:
            _, corrections = replay(runtime, None, None, owner_events, [],
                start_ns=start, end_ns=end, measured=meter, on_window_start=begin_window)
        finally:
            stop.set()
            if thread and thread.ident is not None:
                thread.join(10)
                if thread.is_alive() or errors:
                    raise RuntimeError(f"planner worker failed to stop cleanly: {errors}")
        key = f"{device}_{'concurrent' if concurrent else 'isolated'}"
        cases[key] = {"stages": meter.summaries(), "marker_updates": corrections,
                      "estimator_dtype": str(runtime.dtype),
                      "planner_dtype": str(planner_bundle.runtime.dtype),
                      "final_state": state_record(runtime.clone_state()),
                      "final_markers_m": state_record(runtime.current_markers()),
                      "plan_count": len(plan_rows), "plans": plan_rows,
                      "plan_wall_ms": percentile_metrics([r["elapsed_ms"] for r in plan_rows]) if plan_rows else {},
                      "transfer_ms": percentile_metrics([r["transfer_ms"] for r in plan_rows]) if plan_rows else {},
                      "deadline_misses": sum(r["elapsed_ms"] > 1000*config.planning_deadline_s for r in plan_rows)}
        print(f"completed {key}: {len(corrections)} marker updates, {len(plan_rows)} plans", flush=True)
    baseline = cases["cuda:0_isolated"]
    checks = {}
    for name, case in cases.items():
        checks[name] = {"state_differences": differences(baseline["final_state"], case["final_state"]),
                       "same_device_concurrent_differences": differences(cases[name.split('_')[0]+"_isolated"]["final_state"], case["final_state"]),
                       "maximum_final_marker_difference_mm": float(1000*np.linalg.norm(np.asarray(baseline["final_markers_m"])-np.asarray(case["final_markers_m"]), axis=-1).max()),
                       "marker_decisions_equal": [(r["source_ns"], r["accepted"], r["reason"]) for r in baseline["marker_updates"]] == [(r["source_ns"], r["accepted"], r["reason"]) for r in case["marker_updates"]]}
    report = {"scope": __doc__, "session": str(args.session), "samples": args.samples,
              "config": asdict(config), "manifest_sha256": estimator.identity.manifest_sha256,
              "adaptation_enabled": False, "state_tolerance": {"rtol": 3e-5, "atol": 2e-6},
              "transmission_fixture": None,
              "torch": torch.__version__, "gpu": torch.cuda.get_device_name(),
              "window": {"start_ns": start, "end_ns": end}, "cases": cases, "conformance": checks,
              "qualification": "OFFLINE_ONLY_NOT_ROS_OR_HARDWARE_QUALIFIED"}
    (args.output/"comparison.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({"output": str(args.output), "conformance": checks}, indent=2))


if __name__ == "__main__":
    main()
