"""Numerical old/new qualification on trusted captured replay snapshots.

This is a subordinate worker of compare_compute, not a deployment runtime.
Rollout probes are identical radians/s arrays, not commands or planner outputs.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from .benchmark_compute_isolation import state_record


def cartesian_difference(old, new):
    """Require identical timestamp grids; never interpolate mismatched replay."""
    key = lambda row: (row["stage"], row["receipt_ns"], row["source_ns"], row["state_ns"])
    if [key(r) for r in old] != [key(r) for r in new] or not old:
        raise ValueError("Cartesian replay grids differ or are empty")
    errors = {"tip": [], "markers": [], "centerline": []}
    maximum = {name: {"error_mm": -1.} for name in errors}
    for a, b in zip(old, new):
        for name, field in (("tip", "markers_m"), ("markers", "markers_m"), ("centerline", "centerline_m")):
            x, y = np.asarray(a[field]), np.asarray(b[field])
            if x.shape != y.shape or not np.isfinite(x).all() or not np.isfinite(y).all():
                raise ValueError("invalid Cartesian comparison arrays")
            difference = 1000*np.linalg.norm(x-y, axis=-1)
            value = float(difference[-1] if name == "tip" else difference.max())
            errors[name].append(value)
            if value > maximum[name]["error_mm"]:
                maximum[name] = {"error_mm": value, **{k: a[k] for k in ("stage", "receipt_ns", "source_ns", "state_ns")}}
    return {name: {"count": len(values), "p50_mm": float(np.median(values)),
                   "p95_mm": float(np.percentile(values, 95)), "maximum": maximum[name]}
            for name, values in errors.items()}


def evaluate(fixture_report, model_manifest, device, output):
    """Load only the parent-created fixture after checking its recorded digest."""
    from cr_meta_lnn.deployment import load_runtime_bundle
    report_path = Path(fixture_report)
    record = json.loads(report_path.read_text())
    fixture = report_path.parent/"captured_states.pt"
    if hashlib.sha256(fixture.read_bytes()).hexdigest() != record["captured_states_sha256"]:
        raise ValueError("captured snapshot hash mismatch")
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    bundle = load_runtime_bundle(model_manifest, device=device,
                                 options={"marker_estimator": "ukf", "adaptation_enabled": False})
    runtime = bundle.runtime
    # Explicit opt-out of weights_only: these are our own hashed local
    # dataclasses, not an arbitrary downloaded model/checkpoint.
    states = torch.load(fixture, map_location=runtime.device, weights_only=False)
    if not states:
        raise ValueError("empty snapshot fixture")
    indices = np.unique(np.linspace(0, len(states)-1, min(8, len(states)), dtype=int))
    rates = np.zeros((7, 4, 3), dtype=np.float32)
    rates[1, :, 0], rates[2, :, 0] = .15, -.15
    rates[3, :, 2], rates[4, :, 2] = .15, -.15
    rates[5, :, [0, 2]], rates[6, :, [0, 2]] = .15, -.15
    dt = np.full(4, .2, dtype=np.float32)
    rows = []
    for index in indices:
        root = states[index]
        inputs = json.dumps(state_record(root), sort_keys=True, allow_nan=True).encode()
        prediction = runtime.predict_control_sequence_coarse(root, rates, dt)
        rows.append({"index": int(index), "source_ns": root.timestamp_ns,
                     "input_state_sha256": hashlib.sha256(inputs).hexdigest(),
                     "tips_m": prediction.tip_base_m.detach().cpu().tolist(),
                     "lambda": prediction.distal_lambda.detach().cpu().tolist(),
                     "final_markers_m": runtime.markers_for_state(prediction.final_state).detach().cpu().tolist()})
        print(f"frozen snapshot {len(rows)}/{len(indices)} source_ns={root.timestamp_ns}", flush=True)
    result = {"fixture_sha256": record["captured_states_sha256"], "rates_rad_s": rates.tolist(),
              "dt_s": dt.tolist(), "rows": rows,
              "scope": "identical complete roots and rate probes; no estimator updates or ROS/actuator interfaces"}
    Path(output).write_text(json.dumps(result, indent=2)+"\n")


def frozen_difference(old, new):
    if old["fixture_sha256"] != new["fixture_sha256"] or old["rates_rad_s"] != new["rates_rad_s"] or old["dt_s"] != new["dt_s"]:
        raise ValueError("frozen rollout inputs differ")
    if len(old["rows"]) != len(new["rows"]) or not old["rows"]:
        raise ValueError("invalid frozen rows")
    rows = []
    for a, b in zip(old["rows"], new["rows"]):
        if any(a[key] != b[key] for key in ("index", "source_ns", "input_state_sha256")):
            raise ValueError("frozen root state mismatch")
        rows.append({"index": a["index"], "source_ns": a["source_ns"],
                     "maximum_tip_difference_mm": float(1000*np.linalg.norm(np.asarray(a["tips_m"])-b["tips_m"], axis=-1).max()),
                     "maximum_final_marker_difference_mm": float(1000*np.linalg.norm(np.asarray(a["final_markers_m"])-b["final_markers_m"], axis=-1).max()),
                     "maximum_lambda_difference": float(np.abs(np.asarray(a["lambda"])-b["lambda"]).max())})
    return {"rows": rows, "maximum_tip_difference_mm": max(r["maximum_tip_difference_mm"] for r in rows),
            "maximum_final_marker_difference_mm": max(r["maximum_final_marker_difference_mm"] for r in rows),
            "maximum_lambda_difference": max(r["maximum_lambda_difference"] for r in rows)}
