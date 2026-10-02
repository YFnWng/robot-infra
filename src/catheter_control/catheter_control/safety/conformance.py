"""Deterministic, non-actuating Phase-5 cross-repository conformance replay."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np
import torch

from ..planning.mppi import CatheterMppi, MppiConfig
from ..transmission.backlash import BacklashSnapshot, rollout_backlash_state
from .hardware_contract import load_hardware_contract


FIXTURE = Path(__file__).parent/"fixtures"/"phase5_conformance_v1.json"
DEFAULT_MANIFEST = (
    Path(sys.prefix)/"lib/python3.10/site-packages/cr_meta_lnn"
    / "artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json")


def _default_limits() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return (Path(get_package_share_directory("control_interface"))
                / "config" / "catheter_limits.yaml")
    except (ImportError, LookupError):
        return (Path("/home/chen-lab/Yifan/robot-infra/src/control_interface")
                / "config" / "catheter_limits.yaml")

INITIAL_COUNTS = np.asarray([4883.0, 0.0, 0.0, 0.0, 0.0, 0.0])
JOINT_POSITION = np.asarray([20.0, 0.0, 7.5, 0.0, 0.0, 0.0])
ENCODER_TRACE = (
    (1_040_000_000, np.asarray([4895.0, 0.0, 3.0, 0.0, 0.0, 0.0])),
    (1_090_000_000, np.asarray([4915.0, 0.0, 9.0, 0.0, 0.0, 0.0])),
    (1_140_000_000, np.asarray([4908.0, 0.0, 5.0, 0.0, 0.0, 0.0])),
)
QUALITY = {
    "confidence": [0.9]*4,
    "reprojection_error_px": [0.5]*4,
    "source_rig_count": [2.0]*4,
}


def _array(value) -> list:
    if torch.is_tensor(value):
        value = value.detach().cpu().numpy()
    return np.asarray(value).tolist()


def _load_runtime(manifest: Path):
    from cr_meta_lnn.deployment import load_runtime_bundle
    return load_runtime_bundle(
        manifest, device="cpu", options={"adaptation_enabled": False})


def _fresh_runtime(manifest: Path):
    bundle = _load_runtime(manifest)
    runtime = bundle.runtime
    runtime.initialize(1_000_000_000, INITIAL_COUNTS)
    return bundle, runtime


def _runtime_trace(manifest: Path) -> dict[str, Any]:
    bundle, runtime = _fresh_runtime(manifest)
    initial_markers = runtime.current_markers()
    tips = [_array(initial_markers[-1])]
    for timestamp_ns, counts in ENCODER_TRACE:
        runtime.advance_encoder(timestamp_ns, counts)
        tips.append(_array(runtime.current_markers()[-1]))

    predicted = runtime.current_markers()
    attempts = (runtime.estimator_initialization_observations
                + runtime.estimator_initialization_consecutive_inliers+2)
    results = [runtime.observe_markers(
        ENCODER_TRACE[-1][0], predicted, QUALITY) for _ in range(attempts)]
    outlier = predicted.clone()
    outlier[-1] += outlier.new_tensor([0.05, 0.05, 0.05])
    rejected = runtime.observe_markers(
        ENCODER_TRACE[-1][0], outlier, QUALITY)

    state = runtime.clone_state()
    return {
        "identity": {
            "deployment_api_version": bundle.identity.deployment_api_version,
            "runtime_family": bundle.identity.runtime_family,
            "bundle_name": bundle.identity.bundle_name,
            "manifest_sha256": bundle.identity.manifest_sha256,
            "artifact_sha256": {
                item.artifact_id: item.sha256
                for item in bundle.identity.artifacts},
        },
        "initial_markers_m": _array(initial_markers),
        "tip_trace_m": tips,
        "final_state": {
            "timestamp_ns": state.timestamp_ns,
            "motor_angle_rad": _array(state.motor_angle_rad),
            "raw_motor_angle_rad": _array(state.raw_motor_angle_rad),
            "downstream": _array(state.downstream),
            "interface_pose": _array(state.interface_pose),
            "strain": _array(state.strain),
            "accepted_observations": state.accepted_observations,
            "consecutive_rejections": state.consecutive_rejections,
            "health": state.health,
        },
        "marker_updates": {
            "accepted": sum(item.accepted for item in results),
            "reasons": [item.reason for item in results],
            "health": [item.health for item in results],
            "rejected": asdict(rejected),
        },
    }


def _delayed_observation_trace(manifest: Path) -> dict[str, Any]:
    _, runtime = _fresh_runtime(manifest)
    historical_markers = None
    for timestamp_ns, counts in ENCODER_TRACE:
        runtime.advance_encoder(timestamp_ns, counts)
        if timestamp_ns == ENCODER_TRACE[0][0]:
            historical = runtime.clone_state_at_or_before(timestamp_ns)
            historical_markers = runtime.markers_for_state(historical)
    assert historical_markers is not None
    result = runtime.observe_markers(
        ENCODER_TRACE[0][0],
        historical_markers+historical_markers.new_tensor(
            [0.0001, -0.00005, 0.00003]),
        QUALITY)
    state = runtime.clone_state()
    return {
        "result": asdict(result),
        "present_timestamp_ns": state.timestamp_ns,
        "present_tip_m": _array(runtime.current_markers()[-1]),
        "estimator_status": runtime.estimator_status(state.timestamp_ns),
    }


def _transaction_preview(enabled: bool) -> dict[str, Any]:
    desired = np.zeros((1, 4, 3), dtype=np.float64)
    desired[0, :, 0] = -0.30
    desired[0, :, 2] = -0.20
    if not enabled:
        return {
            "policy": "direct",
            "command_motor_rad_s": desired[0].tolist(),
            "transmitted_motor_rad_s": desired[0].tolist(),
        }
    snapshot = BacklashSnapshot(
        width_rad=np.asarray([0.18, 0.0, 0.12]),
        width_positive_rad=np.asarray([0.18, 0.0, 0.12]),
        width_negative_rad=np.asarray([0.18, 0.0, 0.12]),
        remaining_rad=np.zeros(3),
        motion_direction=np.asarray([1, 0, 1], dtype=np.int8),
        engaged_direction=np.asarray([1, 0, 1], dtype=np.int8),
        confidence=np.ones(3),
        confirmation_count=np.ones(3, dtype=np.int32),
        phase=("ENGAGED", "UNKNOWN", "ENGAGED"))
    preview = rollout_backlash_state(
        desired, snapshot, np.asarray([0.50, 0.50, 0.30]), 0.20)
    return {
        "policy": "response_terminated_takeup",
        "command_motor_rad_s": _array(
            preview.compensated_motor_radians_per_second[0]),
        "transmitted_motor_rad_s": _array(
            preview.transmitted_motor_radians_per_second[0]),
        "remaining_rad": _array(preview.remaining_rad[0]),
        "takeup_delay_s": float(preview.takeup_delay_s[0]),
    }


def _controller_trace(manifest: Path, limits: Path) -> dict[str, Any]:
    _, runtime = _fresh_runtime(manifest)
    contract = load_hardware_contract(limits, "imricor_test")
    state = runtime.clone_state()
    tip = runtime.current_markers()[-1].detach().cpu().numpy()
    target = tip+np.asarray([0.004, 0.0, 0.004])
    modes = {
        "grouped": {
            "grouped_mode_sampling": True,
            "engaged_gain_scenarios": False,
            "best_candidate_guard": True,
            "takeup_risk_weight": 4.0,
            "takeup": True,
        },
        "plain_with_takeup": {
            "grouped_mode_sampling": False,
            "engaged_gain_scenarios": False,
            "best_candidate_guard": False,
            "takeup_risk_weight": 0.0,
            "takeup": True,
        },
        "plain": {
            "grouped_mode_sampling": False,
            "engaged_gain_scenarios": False,
            "best_candidate_guard": False,
            "takeup_risk_weight": 0.0,
            "takeup": False,
        },
    }
    result = {}
    for name, settings in modes.items():
        mode = dict(settings)
        takeup = mode.pop("takeup")
        planner = CatheterMppi(runtime, contract, MppiConfig(
            samples=32, horizon_steps=4, step_s=0.20,
            point_rollout_step_s=0.20, point_rollout_coarse_steps=True,
            planning_deadline_s=100.0, seed=17, **mode))
        plan = planner.plan(state, JOINT_POSITION, target)
        result[name] = {
            "valid": plan.valid,
            "reason": plan.reason,
            "command_logical_velocity": _array(
                plan.command_logical_velocity),
            "logical_velocity_sequence": _array(
                plan.logical_velocity_sequence),
            "best_tip_sequence_m": _array(plan.best_tip_sequence_m),
            "best_cost": plan.best_cost,
            "effective_samples": plan.effective_samples,
            "best_candidate_selected": plan.best_candidate_selected,
            "transaction_preview": _transaction_preview(takeup),
        }
    return {"target_tip_m": target.tolist(), "modes": result}


def capture(manifest: Path, limits: Path) -> dict[str, Any]:
    """Capture the deterministic behavior surface without ROS or commands."""
    return {
        "schema_version": 1,
        "scope": "OFFLINE_NON_ACTUATING_CONFORMANCE",
        "encoder_zero_policy": "READ_ONLY_NEVER_SET_ZERO",
        "runtime": _runtime_trace(manifest),
        "delayed_observation": _delayed_observation_trace(manifest),
        "controller": _controller_trace(manifest, limits),
    }


def _tolerance(path: str) -> float:
    if "best_cost" in path or "effective_samples" in path:
        return 1e-4
    if "velocity" in path or "rad_s" in path or "remaining_rad" in path:
        return 1e-6
    if any(token in path for token in (
            "markers_m", "tip_m", "tip_trace_m", "interface_pose")):
        return 3e-6
    if any(token in path for token in (
            "motor_angle_rad", "downstream", "strain")):
        return 2e-6
    return 1e-9


def compare(expected: Any, actual: Any, path: str = "") -> list[dict]:
    """Return field-specific conformance failures; discrete values are exact."""
    failures = []
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or set(expected) != set(actual):
            return [{"path": path, "expected_keys": sorted(expected),
                     "actual_keys": sorted(actual) if isinstance(actual, dict)
                     else None}]
        for key in expected:
            failures.extend(compare(
                expected[key], actual[key], f"{path}.{key}".strip(".")))
        return failures
    if isinstance(expected, list):
        if not isinstance(actual, list):
            return [{"path": path, "expected": expected, "actual": actual}]
        try:
            left = np.asarray(expected, dtype=np.float64)
            right = np.asarray(actual, dtype=np.float64)
        except (TypeError, ValueError):
            if expected != actual:
                failures.append(
                    {"path": path, "expected": expected, "actual": actual})
            return failures
        tolerance = _tolerance(path)
        if (left.shape != right.shape
                or not np.allclose(left, right, rtol=0.0, atol=tolerance,
                                   equal_nan=True)):
            failures.append({
                "path": path, "tolerance": tolerance,
                "maximum_absolute_difference": (
                    None if left.shape != right.shape else
                    float(np.nanmax(np.abs(left-right)))),
                "expected_shape": list(left.shape),
                "actual_shape": list(right.shape),
            })
        return failures
    if isinstance(expected, (float, int)) and not isinstance(expected, bool):
        tolerance = _tolerance(path)
        if not np.isclose(expected, actual, rtol=0.0, atol=tolerance,
                          equal_nan=True):
            failures.append({"path": path, "expected": expected,
                             "actual": actual, "tolerance": tolerance})
    elif expected != actual:
        failures.append({"path": path, "expected": expected, "actual": actual})
    return failures


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Run offline Phase-5 cross-repository conformance")
    parser.add_argument("--model-manifest", type=Path,
                        default=DEFAULT_MANIFEST)
    parser.add_argument("--limits-file", type=Path, default=_default_limits())
    parser.add_argument("--baseline", type=Path, default=FIXTURE)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--write-baseline", action="store_true")
    args = parser.parse_args(argv)
    actual = capture(args.model_manifest.resolve(), args.limits_file.resolve())
    if args.write_baseline:
        args.baseline.parent.mkdir(parents=True, exist_ok=True)
        args.baseline.write_text(
            json.dumps(actual, indent=2, sort_keys=True)+"\n",
            encoding="utf-8")
        failures = []
    else:
        expected = json.loads(args.baseline.read_text(encoding="utf-8"))
        failures = compare(expected, actual)
    report = {
        "baseline": str(args.baseline.resolve()),
        "manifest": str(args.model_manifest.resolve()),
        "passed": not failures,
        "hardware_qualified": False,
        "command_output": "ABSENT",
        "failures": failures,
        "actual": actual,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True)+"\n",
            encoding="utf-8")
    print(json.dumps({
        "passed": report["passed"], "failures": len(failures),
        "baseline": report["baseline"]}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
