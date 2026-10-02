"""Non-actuating CPU/CUDA estimator conformance replay, not a timing benchmark."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

from cr_meta_lnn.deployment import load_runtime_bundle
from cr_meta_lnn.deployment.experimental.estimator_numerics import (
    compare_states, probe_common_prior)
from .benchmark_compute_isolation import differences, state_record
from .compute_profile import Measurements, read_events, replay, require_controller_idle


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--model-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=45.)
    args = parser.parse_args(argv)
    if not np.isfinite(args.duration_s) or not 0 < args.duration_s <= 120:
        parser.error("duration must be finite and in (0,120] seconds")
    require_controller_idle()
    if not torch.cuda.is_available():
        raise RuntimeError("numerical qualification requires CUDA; no fallback")
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    args.output.mkdir(parents=True, exist_ok=False)
    events = [event for event in read_events(args.session) if event.kind != "plan"]
    start = events[0].receipt_ns
    end = start+int(args.duration_s*1e9)
    summaries = {}
    selected_adaptation = json.loads(args.model_manifest.read_text())["features"]["model_adaptation"]["selected"]
    for dtype in ("float32", "float64"):
        # Respect the verified manifest; never bypass its feature interlock.
        for adaptation in (selected_adaptation,):
            key = f"{dtype}_rls_{'on' if adaptation else 'off'}"
            options = {"marker_estimator": "ukf", "adaptation_enabled": adaptation}
            cpu_bundle = load_runtime_bundle(args.model_manifest, device="cpu", dtype=dtype, options=options)
            gpu_bundle = load_runtime_bundle(args.model_manifest, device="cuda:0", dtype=dtype, options=options)
            cpu, gpu = cpu_bundle.runtime, gpu_bundle.runtime
            reference, probes = [], []
            def record_cpu(stage, event, result):
                if stage == "before_marker":
                    if (cpu.state.initialization_complete and len(probes) < 12
                            and (not probes or event.source_ns-probes[-1][1].source_ns > 2_000_000_000)):
                        root = cpu.clone_state_at_or_before(event.source_ns)
                        if root is not None:
                            probes.append((root, event))
                    return
                reference.append((stage, event.source_ns, cpu.clone_state(),
                                  cpu.current_markers().clone(),
                                  None if result is None else (result.accepted, result.reason)))
            replay(cpu, None, None, events, [], start_ns=start, end_ns=end,
                   measured=Measurements(), on_estimator_event=record_cpu)
            rows, first_failure = [], None
            cursor = 0
            def compare_gpu(stage, event, result):
                nonlocal cursor, first_failure
                if stage == "before_marker":
                    return
                a_stage, stamp, left, markers, decision = reference[cursor]
                assert (a_stage, stamp) == (stage, event.source_ns), "replay event alignment changed"
                cursor += 1
                right = gpu.clone_state().clone_to("cpu")
                mismatches = differences(state_record(left), state_record(right))
                row = {"stage": stage, "source_ns": stamp,
                       "receipt_offset_s": (event.receipt_ns-start)*1e-9,
                       "state_differences": mismatches,
                       "cpu_decision": decision,
                       "cuda_decision": None if result is None else (result.accepted, result.reason),
                       "cpu_observable_rank": int(round(float(torch.trace(left.estimator_observable_projection)))),
                       "cuda_observable_rank": int(round(float(torch.trace(right.estimator_observable_projection)))),
                       "decision_equal": decision == (None if result is None else (result.accepted, result.reason)),
                       **compare_states(left, right, markers, gpu.current_markers())}
                if mismatches and first_failure is None:
                    first_failure = row
                rows.append(row)
            replay(gpu, None, None, events, [], start_ns=start, end_ns=end,
                   measured=Measurements(), on_estimator_event=compare_gpu)
            assert cursor == len(reference), "replay callback counts changed"
            with (args.output/f"{key}.jsonl").open("w") as output:
                for row in rows:
                    output.write(json.dumps(row)+"\n")
            common = [{"source_ns": event.source_ns,
                       **probe_common_prior(cpu, gpu, root, event.values,
                                            event.quality, event.source_ns)}
                      for root, event in probes]
            numeric_keys = [name for name, value in rows[-1].items()
                            if isinstance(value, float) and name != "receipt_offset_s"]
            summaries[key] = {
                "calls": len(rows), "marker_calls": sum(row["stage"] == "marker" for row in rows),
                "first_strict_state_failure": first_failure,
                "maximum_discrepancies": {name: max(row[name] for row in rows) for name in numeric_keys},
                "decision_mismatches": sum(not row["decision_equal"] for row in rows),
                "acceptance_mismatches": sum(row["cpu_decision"][0] != row["cuda_decision"][0]
                                              for row in rows if row["stage"] == "marker"),
                "rank_mismatches": sum(row["cpu_observable_rank"] != row["cuda_observable_rank"] for row in rows),
                "discrete_mismatches": {name: sum(not row[name] for row in rows)
                                        for name in rows[-1] if name.endswith("_equal")},
                "rls_updates_cpu": sum(state.last_rls_weight > 0 for stage, _, state, _, _ in reference if stage == "marker"),
                "common_prior_probes": common,
            }
            print(json.dumps({"case": key, "calls": len(rows),
                              "decision_mismatches": summaries[key]["decision_mismatches"],
                              "maxima": summaries[key]["maximum_discrepancies"]}), flush=True)
    report = {"session": str(args.session), "manifest_sha256": cpu_bundle.identity.manifest_sha256,
              "duration_s": args.duration_s, "torch": torch.__version__,
              "gpu": torch.cuda.get_device_name(), "state_tolerance": {"rtol": 3e-5, "atol": 2e-6},
              "adaptation_selected_by_manifest": selected_adaptation,
              "scope": __doc__, "cases": summaries,
              "qualification": "DIAGNOSTIC_ONLY_NO_TIMING_OR_HARDWARE_QUALIFICATION"}
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")


if __name__ == "__main__":
    main()
