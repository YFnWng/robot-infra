#!/usr/bin/env python3
"""Evaluate fixed-J0 model response against a recorded real MPPI bag."""
from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import torch


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPOSITORY_ROOT.parent
DEFAULT_META = WORKSPACE_ROOT / "cr_meta_lnn"
DEFAULT_COMMON = WORKSPACE_ROOT / "cr-common"
_TORCH_INTEROP_CONFIGURED = False


@dataclass
class ObservationState:
    timestamp_ns: int
    points: np.ndarray
    state: object


def _stamp_ns(message):
    return (int(message.header.stamp.sec)*1_000_000_000
            + int(message.header.stamp.nanosec))


def _read_events(bag: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = {
        item.name: get_message(item.type)
        for item in reader.get_all_topics_and_types()
        if item.name in ("/device/state", "/shape_tracking/markers")}
    required = {"/device/state", "/shape_tracking/markers"}
    if not required.issubset(topics):
        raise RuntimeError(f"bag lacks required topics: {required-topics.keys()}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(required)))
    events = []
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        message = deserialize_message(payload, topics[topic])
        timestamp_ns = _stamp_ns(message)
        if timestamp_ns <= 0:
            continue
        if topic == "/device/state":
            # DeviceStream.ENC is predicate 69 in the firmware contract.
            if int(message.predicate) == 69 and len(message.data) >= 3:
                values = np.asarray(message.data[:3], dtype=np.float64)
                if np.isfinite(values).all():
                    events.append((timestamp_ns, 0, "encoder", values))
        elif len(message.points) == 4:
            points = np.asarray(
                [[p.x, p.y, p.z] for p in message.points],
                dtype=np.float64)
            if np.isfinite(points).all():
                quality = {
                    channel.name: list(channel.values)
                    for channel in message.channels}
                events.append(
                    (timestamp_ns, 1, "markers", (points, quality)))
    events.sort(key=lambda item: (item[0], item[1]))
    return events


def _percentiles(values, scale=1.0):
    values = scale*np.asarray(values, dtype=np.float64)
    if not values.size:
        return {"count": 0}
    return {
        "count": int(values.size),
        "mean": float(values.mean()),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "maximum": float(values.max()),
    }


def _vector_response(actual, predicted, unit_scale=1.0,
                     minimum_actual_norm=0.0):
    actual = np.asarray(actual, dtype=np.float64)
    predicted = np.asarray(predicted, dtype=np.float64)
    if not len(actual):
        return {"count": 0}
    error = actual-predicted
    norm_a = np.linalg.norm(actual, axis=1)
    norm_p = np.linalg.norm(predicted, axis=1)
    valid = ((norm_a >= maximum_actual_threshold(minimum_actual_norm))
             & (norm_p > 1e-12))
    cosine = np.sum(actual[valid]*predicted[valid], axis=1)/(
        norm_a[valid]*norm_p[valid])
    denominator = np.sum(predicted*predicted, axis=1)
    gain_valid = valid & (denominator > 1e-18)
    gain = np.sum(actual[gain_valid]*predicted[gain_valid], axis=1)/(
        denominator[gain_valid])
    return {
        "count": int(len(actual)),
        "direction_sample_count": int(np.sum(valid)),
        "minimum_actual_norm_for_direction": (
            float(minimum_actual_norm*unit_scale)),
        "actual_norm": _percentiles(norm_a, unit_scale),
        "predicted_norm": _percentiles(norm_p, unit_scale),
        "error_norm": _percentiles(
            np.linalg.norm(error, axis=1), unit_scale),
        "direction_cosine": _percentiles(cosine),
        "signed_gain_actual_over_predicted": _percentiles(gain),
    }


def maximum_actual_threshold(value):
    return max(float(value), 1e-12)


def _axis_responses(action, actual, predicted):
    names = ("insertion", "rotation", "bending")
    magnitude = np.abs(action)
    purity = np.max(magnitude, axis=1)/np.maximum(
        np.sum(magnitude, axis=1), 1e-12)
    dominant = np.argmax(magnitude, axis=1)
    result = {}
    for index, name in enumerate(names):
        selected = (dominant == index) & (purity >= 0.70)
        result[name] = _vector_response(
            actual[selected], predicted[selected], 1e3, 0.00025)
        result[name]["minimum_action_purity"] = 0.70
    return result


def _column_comparison(fitted, initial):
    result = []
    for index, name in enumerate(("insertion", "rotation", "bending")):
        row = {"axis": name}
        for label, selection in (
                ("angular", slice(0, 3)), ("linear", slice(3, 6))):
            fit = fitted[selection, index]
            base = initial[selection, index]
            denominator = float(np.dot(base, base))
            norm_product = float(np.linalg.norm(fit)*np.linalg.norm(base))
            row[label] = {
                "direction_cosine": (
                    float(np.dot(fit, base)/norm_product)
                    if norm_product > 1e-18 else None),
                "gain_fitted_over_J0": (
                    float(np.dot(fit, base)/denominator)
                    if denominator > 1e-18 else None),
                "J0": base.tolist(),
                "fitted": fit.tolist(),
            }
        result.append(row)
    return result


def evaluate(arguments):
    global _TORCH_INTEROP_CONFIGURED
    meta = arguments.cr_meta_lnn_root.resolve()
    common = arguments.cr_common_root.resolve()
    for path in (meta.parent, common):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    from cr_meta_lnn.deployment import V171StreamingCatheterRuntime
    from cr_meta_lnn.networks.hybrid.kinematics import (
        inverse_transform, se3_log_torch)

    torch.set_num_threads(arguments.torch_threads)
    if not _TORCH_INTEROP_CONFIGURED:
        torch.set_num_interop_threads(1)
        _TORCH_INTEROP_CONFIGURED = True
    adaptation_enabled = bool(getattr(arguments, "adaptation_enabled", False))
    runtime = V171StreamingCatheterRuntime(
        arguments.distal_checkpoint,
        arguments.jacobian_initialization_json,
        device="cpu", marker_estimator=arguments.marker_estimator,
        adaptation_enabled=adaptation_enabled,
        estimator_filter_initial_covariance=(
            arguments.estimator_filter_initial_covariance),
        estimator_filter_process_std_sqrt_s=(
            arguments.estimator_filter_process_std_sqrt_s),
        adaptation_minimum_observations=(
            arguments.adaptation_minimum_observations),
        adaptation_minimum_normalized_action=(
            arguments.adaptation_minimum_normalized_action),
        adaptation_minimum_rotation_deg=(
            arguments.adaptation_minimum_rotation_deg),
        adaptation_minimum_translation_mm=(
            arguments.adaptation_minimum_translation_mm),
        adaptation_minimum_response_snr=(
            arguments.adaptation_minimum_response_snr),
        adaptation_maximum_window_s=arguments.adaptation_maximum_window_s,
        adaptation_directional_purity=arguments.adaptation_directional_purity,
        adaptation_reversal_holdoff_normalized_action_by_axis=(
            arguments.adaptation_reversal_holdoff_by_axis),
        adaptation_confirmation_windows=(
            arguments.adaptation_confirmation_windows),
        adaptation_consistency_cosine=(
            arguments.adaptation_consistency_cosine),
        adaptation_minimum_column_gain=(
            arguments.adaptation_minimum_column_gain),
        adaptation_maximum_column_gain=(
            arguments.adaptation_maximum_column_gain),
        adaptation_maximum_direction_deviation_deg=(
            arguments.adaptation_maximum_direction_deviation_deg))
    events = _read_events(arguments.bag)
    observations = []
    last_marker_ns = 0
    encoder_events = marker_events = accepted = rejected = 0
    adaptation_reasons = Counter()
    adaptation_reasons_by_axis = Counter()
    for timestamp_ns, _, kind, value in events:
        if kind == "encoder":
            encoder_events += 1
            if runtime.state is None:
                runtime.initialize(timestamp_ns, value)
            elif timestamp_ns > runtime.state.timestamp_ns:
                runtime.advance_encoder(timestamp_ns, value)
            continue
        marker_events += 1
        if runtime.state is None:
            continue
        if timestamp_ns-last_marker_ns < int(1e9/arguments.marker_rate_hz):
            continue
        points, quality = value
        result = runtime.observe_markers(timestamp_ns, points, quality)
        last_marker_ns = timestamp_ns
        if result.accepted and result.health == "TRACKING":
            accepted += 1
            adaptation_reasons[runtime.state.last_rls_reason] += 1
            adaptation_reasons_by_axis[
                f"{runtime.state.last_rls_axis}:"
                f"{runtime.state.last_rls_reason}"] += 1
            observations.append(ObservationState(
                timestamp_ns, points.copy(), runtime.clone_state()))
        else:
            rejected += 1
    if len(observations) < 3:
        raise RuntimeError("too few accepted observations for evaluation")

    times = [item.timestamp_ns for item in observations]
    next_start_ns = times[0]
    pairs = []
    tolerance_ns = int(arguments.window_tolerance_s*1e9)
    window_ns = int(arguments.window_s*1e9)
    stride_ns = int(arguments.stride_s*1e9)
    for start_index, start in enumerate(observations):
        if start.timestamp_ns < next_start_ns:
            continue
        target = start.timestamp_ns+window_ns
        end_index = bisect_left(times, target, lo=start_index+1)
        candidates = [i for i in (end_index-1, end_index)
                      if start_index < i < len(observations)]
        if not candidates:
            continue
        end_index = min(candidates, key=lambda i: abs(times[i]-target))
        if abs(times[end_index]-target) > tolerance_ns:
            continue
        pairs.append((start_index, end_index))
        next_start_ns = start.timestamp_ns+stride_ns

    j0 = np.asarray(runtime.initial_jacobian.jacobian, dtype=np.float64)
    action = []
    actual_twist = []
    predicted_twist = []
    actual_tip = []
    predicted_tip = []
    stationary_tip = []
    for start_index, end_index in pairs:
        start, end = observations[start_index], observations[end_index]
        dt = (end.timestamp_ns-start.timestamp_ns)*1e-9
        motor0 = np.asarray(
            start.state.motor_angle_rad.detach().cpu(), dtype=np.float64)
        motor1 = np.asarray(
            end.state.motor_angle_rad.detach().cpu(), dtype=np.float64)
        delta_motor = motor1-motor0
        observed_tip_delta = end.points[-1]-start.points[-1]
        if np.linalg.norm(delta_motor) < arguments.minimum_motor_delta_rad:
            stationary_tip.append(observed_tip_delta)
            continue
        relative = inverse_transform(start.state.interface_pose) @ (
            end.state.interface_pose)
        twist = np.asarray(
            se3_log_torch(relative).detach().cpu(), dtype=np.float64)
        velocity = torch.as_tensor(
            (delta_motor/dt)[None, :], dtype=torch.float32)
        prediction = runtime.predict_sequence(
            start.state, velocity, torch.tensor([dt], dtype=torch.float32))
        predicted_endpoint = np.asarray(
            prediction.tip_base_m[0].detach().cpu(), dtype=np.float64)
        model_start = np.asarray(runtime._marker_points(
            start.state.interface_pose, start.state.strain
        )[-1].detach().cpu(), dtype=np.float64)
        action.append(delta_motor)
        actual_twist.append(twist)
        local_jacobian = np.asarray(
            start.state.adaptive_jacobian.jacobian, dtype=np.float64)
        predicted_twist.append(local_jacobian@delta_motor)
        actual_tip.append(observed_tip_delta)
        predicted_tip.append(predicted_endpoint-model_start)

    action = np.asarray(action, dtype=np.float64)
    actual_twist = np.asarray(actual_twist, dtype=np.float64)
    predicted_twist = np.asarray(predicted_twist, dtype=np.float64)
    actual_tip = np.asarray(actual_tip, dtype=np.float64)
    predicted_tip = np.asarray(predicted_tip, dtype=np.float64)
    if len(action) < 3:
        raise RuntimeError("too few excited windows for evaluation")
    singular = np.linalg.svd(action, compute_uv=False)
    rank = int(np.linalg.matrix_rank(action))
    fitted = np.linalg.lstsq(action, actual_twist, rcond=None)[0].T
    report = {
        "schema": "catheter-real-model-response-v1",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "bag": str(arguments.bag.resolve()),
        "configuration": {
            "window_s": arguments.window_s,
            "stride_s": arguments.stride_s,
            "window_tolerance_s": arguments.window_tolerance_s,
            "minimum_motor_delta_rad": arguments.minimum_motor_delta_rad,
            "marker_replay_rate_hz": arguments.marker_rate_hz,
            "marker_estimator": arguments.marker_estimator,
            "adaptation_enabled": adaptation_enabled,
            "adaptation_mode": (
                "offline_recorded_replay_commits" if adaptation_enabled
                else "shadow_gate_evaluation_no_commits"),
            "distal_checkpoint": str(arguments.distal_checkpoint.resolve()),
            "jacobian_initialization_json": str(
                arguments.jacobian_initialization_json.resolve()),
            "adaptation_reversal_holdoff_normalized_action_by_axis": (
                list(arguments.adaptation_reversal_holdoff_by_axis)),
        },
        "replay": {
            "encoder_events": encoder_events,
            "marker_events": marker_events,
            "accepted_tracking_observations": accepted,
            "rejected_or_initializing_observations": rejected,
            "candidate_equal_duration_windows": len(pairs),
            "excited_windows": int(len(action)),
            "stationary_windows": int(len(stationary_tip)),
        },
        "excitation": {
            "rank": rank,
            "singular_values_rad": singular.tolist(),
            "condition_number": (
                float(singular[0]/singular[-1])
                if singular[-1] > 0 else math.inf),
            "motor_increment_norm_rad": _percentiles(
                np.linalg.norm(action, axis=1)),
        },
        "shadow_adaptation": {
            "committed_updates": int(adaptation_reasons.get(
                "rls_updated", 0)),
            "reason_counts": dict(sorted(adaptation_reasons.items())),
            "reason_counts_by_axis": dict(sorted(
                adaptation_reasons_by_axis.items())),
            "candidate_ready_count": int(adaptation_reasons.get(
                "rls_shadow_ready", 0)),
        },
        "full_tip_response_mm": _vector_response(
            actual_tip, predicted_tip, 1e3, 0.00025),
        "full_tip_response_by_dominant_motor_axis_mm": _axis_responses(
            action, actual_tip, predicted_tip),
        "interface_angular_response_rad": _vector_response(
            actual_twist[:, :3], predicted_twist[:, :3],
            minimum_actual_norm=math.radians(0.1)),
        "interface_linear_response_mm": _vector_response(
            actual_twist[:, 3:], predicted_twist[:, 3:], 1e3, 0.0001),
        "stationary_tip_delta_mm": _percentiles(
            np.linalg.norm(stationary_tip, axis=1), 1e3),
        "fitted_interface_jacobian": fitted.tolist(),
        "initial_interface_jacobian": j0.tolist(),
        "final_interface_jacobian": np.asarray(
            runtime.state.adaptive_jacobian.jacobian,
            dtype=np.float64).tolist(),
        "final_jacobian_change_frobenius": float(np.linalg.norm(
            runtime.state.adaptive_jacobian.jacobian-j0)),
        "jacobian_column_comparison": _column_comparison(fitted, j0),
        "interpretation_limits": [
            "The fitted pose increments come from UKF-corrected model state, "
            "so this is diagnostic rather than independent ground truth.",
            "Full-tip error contains proximal, distal, actuator, registration, "
            "and marker errors; it does not uniquely identify J0.",
            "A trustworthy three-column fit requires excitation rank 3 and a "
            "reasonable action-matrix condition number.",
        ],
    }
    return report


def _arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--cr-meta-lnn-root", type=Path, default=DEFAULT_META)
    parser.add_argument("--cr-common-root", type=Path, default=DEFAULT_COMMON)
    parser.add_argument("--distal-checkpoint", type=Path, default=(
        DEFAULT_META/"checkpoints"
        / "real_distal_first_order_v171_multistep_map_em.pt"))
    parser.add_argument("--jacobian-initialization-json", type=Path, default=(
        DEFAULT_META/"evaluation"/"real_joint_local_distal_v174.json"))
    parser.add_argument("--marker-estimator", default="ukf")
    parser.add_argument("--window-s", type=float, default=0.25)
    parser.add_argument("--stride-s", type=float, default=0.25)
    parser.add_argument("--window-tolerance-s", type=float, default=0.04)
    parser.add_argument("--minimum-motor-delta-rad", type=float, default=0.004)
    parser.add_argument("--marker-rate-hz", type=float, default=10.0)
    parser.add_argument("--torch-threads", type=int, default=2)
    parser.add_argument(
        "--estimator-filter-initial-covariance", type=float, default=.25)
    parser.add_argument(
        "--estimator-filter-process-std-sqrt-s", type=float, default=1.)
    parser.add_argument(
        "--adaptation-minimum-observations", type=int, default=8)
    parser.add_argument(
        "--adaptation-minimum-normalized-action", type=float, default=.10)
    parser.add_argument(
        "--adaptation-minimum-rotation-deg", type=float, default=.30)
    parser.add_argument(
        "--adaptation-minimum-translation-mm", type=float, default=.30)
    parser.add_argument(
        "--adaptation-minimum-response-snr", type=float, default=3.)
    parser.add_argument(
        "--adaptation-maximum-window-s", type=float, default=1.)
    parser.add_argument(
        "--adaptation-directional-purity", type=float, default=.90)
    parser.add_argument(
        "--adaptation-reversal-holdoff-by-axis", type=float, nargs=3,
        default=(4.5, 6.5, 4.0), metavar=("SHAFT0", "SHAFT1", "SHAFT2"))
    parser.add_argument(
        "--adaptation-confirmation-windows", type=int, default=2)
    parser.add_argument(
        "--adaptation-consistency-cosine", type=float, default=.80)
    parser.add_argument(
        "--adaptation-minimum-column-gain", type=float, default=.50)
    parser.add_argument(
        "--adaptation-maximum-column-gain", type=float, default=1.50)
    parser.add_argument(
        "--adaptation-maximum-direction-deviation-deg", type=float,
        default=30.)
    return parser.parse_args()


def main():
    arguments = _arguments()
    arguments.bag = arguments.bag.resolve()
    if not arguments.bag.is_dir():
        raise FileNotFoundError(arguments.bag)
    output = arguments.output or (
        arguments.bag/"real_model_response_evaluation.json")
    report = evaluate(arguments)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({
        "output": str(output),
        "excitation": report["excitation"],
        "full_tip_response_mm": report["full_tip_response_mm"],
        "jacobian_column_comparison": report["jacobian_column_comparison"],
    }, indent=2))


if __name__ == "__main__":
    main()
