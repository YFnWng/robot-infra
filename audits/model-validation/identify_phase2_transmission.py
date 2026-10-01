#!/usr/bin/env python3
"""Identify the separated Phase-2 proximal transmission coordinates.

This tool is deliberately offline and read-only.  It estimates stateful play
operators and local interface columns from isolated shaft-0/shaft-2 episodes,
then evaluates their causal superposition on compensated-bending episodes.
It also compares a no-clamp and explicit-clamp play model against rigid-motion
invariant marker geometry and the UKF distal-strain trace.

The result is an identification diagnostic, not a deployable checkpoint.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPOSITORY_ROOT.parent

from evaluate_causal_proximal_experiment import (
    _estimator_rows,
    _labels_for_estimators,
    _marker_rows,
    _pose_increment,
    _read,
    _episode_rows,
)


ENCODER_RADIANS_PER_COUNT = math.pi / 4000.0
NOMINAL_CHASSIS_METRES_PER_RAD = 5.357142731655222e-4
NOMINAL_KNOB_METRES_PER_RAD = -1.894938541187879e-4


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def asymmetric_play(signal, positive_width, negative_width):
    """Apply causal asymmetric play to one scalar input trajectory.

    ``positive_width`` and ``negative_width`` are input/output offsets on the
    corresponding loading branches.  A full negative-to-positive reversal
    traverses their sum.
    """
    value = np.asarray(signal, dtype=float)
    if value.ndim != 1 or not np.isfinite(value).all():
        raise ValueError("play input must be one finite trajectory")
    positive_width = float(positive_width)
    negative_width = float(negative_width)
    if positive_width < 0.0 or negative_width < 0.0:
        raise ValueError("play widths must be nonnegative")
    result = np.empty_like(value)
    if not value.size:
        return result
    result[0] = value[0]
    for index in range(1, len(value)):
        result[index] = max(
            value[index] - positive_width,
            min(result[index - 1], value[index] + negative_width),
        )
    return result


def marker_pairwise_distances(points):
    """Six rigid-motion-invariant distances for four ordered markers."""
    value = np.asarray(points, dtype=float)
    if value.shape != (4, 3) or not np.isfinite(value).all():
        raise ValueError("marker points must have shape (4,3) and be finite")
    return np.asarray([
        np.linalg.norm(value[left] - value[right])
        for left in range(4) for right in range(left + 1, 4)
    ])


@dataclass(frozen=True)
class Window:
    start: int
    end: int
    basis: str
    tier: str
    repetition: int
    episode: str


def build_windows(times_ns, labels, basis, horizon_s=0.25):
    """Build backward-looking windows that never cross episode boundaries."""
    times = np.asarray(times_ns, dtype=np.int64)
    windows = []
    for end, end_label in enumerate(labels):
        if end_label is None or end_label["basis"] != basis:
            continue
        desired = times[end] - int(float(horizon_s) * 1e9)
        start = int(np.searchsorted(times, desired, side="right") - 1)
        if start < 0 or start >= end:
            continue
        start_label = labels[start]
        if (start_label is None
                or start_label["index"] != end_label["index"]):
            continue
        elapsed = (times[end] - times[start]) * 1e-9
        if not 0.8 * horizon_s <= elapsed <= 1.2 * horizon_s:
            continue
        gaps = np.diff(times[start:end + 1]) * 1e-9
        if gaps.size and float(np.max(gaps)) > 0.15:
            continue
        windows.append(Window(
            start, end, end_label["basis"], end_label["tier"],
            int(end_label["repetition"]), end_label["name"]))
    return windows


def _responses(values, windows, *, pose=False):
    result = []
    valid_windows = []
    for window in windows:
        left, right = values[window.start], values[window.end]
        if not np.isfinite(left).all() or not np.isfinite(right).all():
            continue
        result.append(_pose_increment(left, right) if pose else right - left)
        valid_windows.append(window)
    if not result:
        width = 6 if pose else int(values.shape[-1])
        return np.empty((0, width)), []
    return np.asarray(result), valid_windows


def _response_scale(response, train_mask, fixed=None):
    if fixed is not None:
        scale = np.asarray(fixed, dtype=float)
    else:
        selected = np.asarray(response[train_mask], dtype=float)
        scale = np.percentile(np.abs(selected), 75, axis=0)
        overall = float(np.percentile(np.abs(selected), 50))
        scale = np.maximum(scale, max(overall * 0.1, 1e-9))
    if scale.shape != (response.shape[1],) or np.any(scale <= 0.0):
        raise ValueError("response scale must match response dimension")
    return scale


def _fit_column(input_value, response, windows, train_mask, scale,
                ridge=1e-12):
    action = np.asarray([
        input_value[window.end] - input_value[window.start]
        for window in windows
    ], dtype=float)
    response = np.asarray(response, dtype=float)
    normalized = response / scale
    selected_action = action[train_mask]
    selected_response = normalized[train_mask]
    denominator = float(selected_action @ selected_action + ridge)
    column_normalized = (
        selected_action @ selected_response / denominator)
    prediction_normalized = action[:, None] * column_normalized
    residual_normalized = normalized - prediction_normalized
    column_physical = column_normalized * scale
    return {
        "action": action,
        "column": column_physical,
        "prediction": prediction_normalized * scale,
        "normalized_squared_error": np.sum(
            residual_normalized ** 2, axis=1),
    }


def _mask(windows, holdout_repetition, *, holdout=False):
    repetitions = np.asarray([window.repetition for window in windows])
    return ((repetitions == holdout_repetition) if holdout else
            (repetitions != holdout_repetition))


def _group_scores(errors, windows, mask):
    groups = {}
    for error, window, selected in zip(errors, windows, mask):
        if not selected:
            continue
        key = f"{window.tier}:rep{window.repetition}"
        groups.setdefault(key, []).append(float(error))
    return {key: {
        "windows": len(values),
        "mean_normalized_squared_error": float(np.mean(values)),
        "median_normalized_squared_error": float(np.median(values)),
    } for key, values in sorted(groups.items())}


def search_play(signal, response, windows, scale, holdout_repetition,
                grid_size=21, maximum_side_width=None):
    """Grid-search asymmetric play and fit one response column per candidate."""
    signal = np.asarray(signal, dtype=float)
    response = np.asarray(response, dtype=float)
    train = _mask(windows, holdout_repetition)
    holdout = _mask(windows, holdout_repetition, holdout=True)
    if not train.any() or not holdout.any():
        raise ValueError("both training and held-out repetitions are required")
    span = float(np.percentile(signal, 99) - np.percentile(signal, 1))
    maximum = (0.35 * span if maximum_side_width is None
               else float(maximum_side_width))
    if maximum <= 0.0:
        raise ValueError("play search requires a nonzero input span")

    def evaluate(positive, negative):
        transmitted = asymmetric_play(signal, positive, negative)
        fit = _fit_column(
            transmitted, response, windows, train, scale)
        score = float(np.mean(
            fit["normalized_squared_error"][train]))
        return score, transmitted, fit

    widths = np.linspace(0.0, maximum, int(grid_size))
    candidates = []
    for positive in widths:
        for negative in widths:
            score, _, _ = evaluate(positive, negative)
            candidates.append((score, positive, negative))
    coarse = min(candidates)
    spacing = maximum / max(int(grid_size) - 1, 1)
    positive_grid = np.linspace(
        max(0.0, coarse[1] - spacing),
        min(maximum, coarse[1] + spacing), 11)
    negative_grid = np.linspace(
        max(0.0, coarse[2] - spacing),
        min(maximum, coarse[2] + spacing), 11)
    refined = []
    for positive in positive_grid:
        for negative in negative_grid:
            score, _, _ = evaluate(positive, negative)
            refined.append((score, positive, negative))
    best = min(refined)
    best_score, transmitted, fit = evaluate(best[1], best[2])
    baseline_score, _, baseline = evaluate(0.0, 0.0)
    errors = fit["normalized_squared_error"]
    baseline_errors = baseline["normalized_squared_error"]
    resolution = spacing / 5.0
    positive_at_upper = bool(best[1] >= maximum - 1.5 * resolution)
    negative_at_upper = bool(best[2] >= maximum - 1.5 * resolution)
    return {
        "positive_width_rad": float(best[1]),
        "negative_width_rad": float(best[2]),
        "reversal_width_rad": float(best[1] + best[2]),
        "maximum_side_width_rad": maximum,
        "grid_size": int(grid_size),
        "refined_resolution_rad": resolution,
        "positive_width_at_upper_search_bound": positive_at_upper,
        "negative_width_at_upper_search_bound": negative_at_upper,
        "width_fit_identifiable_inside_search": not (
            positive_at_upper or negative_at_upper),
        "train_score": best_score,
        "train_no_play_score": baseline_score,
        "holdout_score": float(np.mean(errors[holdout])),
        "holdout_no_play_score": float(np.mean(
            baseline_errors[holdout])),
        "holdout_fractional_improvement": float(
            1.0 - np.mean(errors[holdout])
            / max(np.mean(baseline_errors[holdout]), 1e-15)),
        "column_per_input_rad": fit["column"].tolist(),
        "transmitted": transmitted,
        "fit": fit,
        "train_groups": _group_scores(errors, windows, train),
        "holdout_groups": _group_scores(errors, windows, holdout),
    }


def _strip_arrays(value):
    if isinstance(value, dict):
        return {key: _strip_arrays(item) for key, item in value.items()
                if key not in {"transmitted", "fit"}}
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def _matched_markers(estimators, markers, tolerance_ns=2_000_000):
    times = np.asarray([row["timestamp_ns"] for row in markers], dtype=np.int64)
    features = np.full((len(estimators), 6), np.nan)
    points = np.full((len(estimators), 4, 3), np.nan)
    matched = 0
    for index, estimator in enumerate(estimators):
        target = estimator["timestamp_ns"]
        right = int(np.searchsorted(times, target))
        choices = [candidate for candidate in (right - 1, right)
                   if 0 <= candidate < len(times)]
        if not choices:
            continue
        nearest = min(choices, key=lambda candidate: abs(times[candidate]-target))
        row = markers[nearest]
        if abs(row["timestamp_ns"] - target) > tolerance_ns:
            continue
        if row["points"].shape != (4, 3):
            continue
        marker_ids = row["marker_id"]
        if marker_ids is not None and not np.array_equal(
                marker_ids.astype(int), np.arange(4)):
            continue
        points[index] = row["points"]
        features[index] = marker_pairwise_distances(row["points"])
        matched += 1
    return points, features, matched


def export_replay(session: Path, output: Path):
    """Export a ROS-independent Phase-2 replay table for model evaluation."""
    _, messages = _read(session)
    traces = _episode_rows(messages)
    estimators = _estimator_rows(messages)
    markers = _marker_rows(messages)
    if not estimators:
        raise ValueError("replay export requires accepted estimator traces")
    labels = _labels_for_estimators(traces, estimators)
    marker_points, _, marker_matches = _matched_markers(estimators, markers)

    def label(name, default=""):
        return np.asarray([
            default if item is None else item[name] for item in labels])

    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        schema_version=np.asarray(1, dtype=np.int64),
        source_session=np.asarray(str(session)),
        timestamps_ns=np.asarray(
            [row["timestamp_ns"] for row in estimators], dtype=np.int64),
        state_timestamps_ns=np.asarray(
            [row["state_timestamp_ns"] for row in estimators],
            dtype=np.int64),
        raw_encoder_counts=np.asarray([
            row["raw_encoder_counts"][:3] for row in estimators],
            dtype=np.float64),
        raw_motor_angle_rad=np.asarray([
            row["raw_encoder_counts"][:3] * ENCODER_RADIANS_PER_COUNT
            for row in estimators], dtype=np.float64),
        interface_motor_angle_rad=np.asarray(
            [row["motor"][:3] for row in estimators], dtype=np.float64),
        interface_pose=np.asarray(
            [row["pose"] for row in estimators], dtype=np.float64),
        distal_strain=np.asarray(
            [row["distal_strain"] for row in estimators], dtype=np.float64),
        marker_points_m=marker_points.astype(np.float64),
        marker_matched=np.isfinite(marker_points).all(axis=(1, 2)),
        episode_index=label("index", -1).astype(np.int64),
        episode_name=label("name").astype(str),
        excitation_basis=label("basis").astype(str),
        speed_tier=label("tier").astype(str),
        repetition=label("repetition", -1).astype(np.int64),
    )
    return {
        "path": str(output),
        "sha256": _sha256(output),
        "rows": len(estimators),
        "matched_marker_rows": marker_matches,
    }


def _combined_validation(interface_fits, pose, labels, times, state_scale,
                         horizon_s):
    windows = build_windows(
        times, labels, "shaft_0_plus_2_validation", horizon_s)
    response, windows = _responses(pose, windows, pose=True)
    if not windows:
        return {"windows": 0}
    transmitted0 = interface_fits["shaft_0"]["transmitted"]
    transmitted2 = interface_fits["shaft_2"]["transmitted"]
    column0 = interface_fits["shaft_0"]["fit"]["column"]
    column2 = interface_fits["shaft_2"]["fit"]["column"]
    prediction = np.asarray([
        ((transmitted0[window.end] - transmitted0[window.start]) * column0
         + (transmitted2[window.end] - transmitted2[window.start]) * column2)
        for window in windows
    ])
    errors = np.sum(((response - prediction) / state_scale) ** 2, axis=1)
    response_norm = np.linalg.norm(response / state_scale, axis=1)
    prediction_norm = np.linalg.norm(prediction / state_scale, axis=1)
    cosine = np.divide(
        np.sum(response * prediction / state_scale ** 2, axis=1),
        np.maximum(response_norm * prediction_norm, 1e-15))
    directional = (response_norm > 1e-8) & (prediction_norm > 1e-8)
    return {
        "windows": len(windows),
        "nonzero_prediction_windows": int(np.count_nonzero(directional)),
        "zero_prediction_fraction": float(
            1.0 - np.count_nonzero(directional) / len(windows)),
        "mean_normalized_squared_error": float(np.mean(errors)),
        "median_normalized_squared_error": float(np.median(errors)),
        "direction_cosine_median": (
            None if not np.any(directional) else
            float(np.median(cosine[directional]))),
        "direction_cosine_p05": (
            None if not np.any(directional) else
            float(np.percentile(cosine[directional], 5))),
        "groups": _group_scores(
            errors, windows, np.ones(len(windows), dtype=bool)),
    }


def identify(session: Path, jacobian_json: Path, *, horizon_s=0.25,
             grid_size=21, holdout_repetition=3):
    bag, messages = _read(session)
    traces = _episode_rows(messages)
    estimators = _estimator_rows(messages)
    markers = _marker_rows(messages)
    if not traces or not estimators or not markers:
        raise ValueError("Phase 2 identification requires traces, estimator, and markers")
    labels = _labels_for_estimators(traces, estimators)
    times = np.asarray([row["timestamp_ns"] for row in estimators], dtype=np.int64)
    pose = np.asarray([row["pose"] for row in estimators])
    counts = np.asarray([row["raw_encoder_counts"] for row in estimators])
    raw_motor = counts * ENCODER_RADIANS_PER_COUNT
    strain = np.asarray([row["distal_strain"] for row in estimators])
    marker_points, marker_shape, marker_matches = _matched_markers(
        estimators, markers)

    with jacobian_json.open(encoding="utf-8") as stream:
        jacobian = json.load(stream)["jacobian_fits"]["shaft"]
    state_scale = np.asarray(jacobian["state_scale"], dtype=float)

    interface_fits = {}
    for basis, axis, gain in (
            ("shaft_0", 0, NOMINAL_CHASSIS_METRES_PER_RAD),
            ("shaft_2", 2, NOMINAL_KNOB_METRES_PER_RAD)):
        windows = build_windows(times, labels, basis, horizon_s)
        marker0_response, windows = _responses(
            marker_points[:, 0, :], windows, pose=False)
        train = _mask(windows, holdout_repetition)
        marker0_scale = _response_scale(marker0_response, train)
        fit = search_play(
            raw_motor[:, axis], marker0_response, windows, marker0_scale,
            holdout_repetition, grid_size=grid_size)
        # Select transmission play from a directly observed proximal marker,
        # then fit the full UKF interface-pose column without allowing its
        # current model assumptions to choose the play width.
        pose_response, pose_windows = _responses(pose, windows, pose=True)
        if pose_windows != windows:
            raise RuntimeError("finite marker and pose windows must align")
        pose_fit = _fit_column(
            fit["transmitted"], pose_response, windows, train, state_scale)
        holdout = _mask(windows, holdout_repetition, holdout=True)
        fit["width_objective"] = "measured_marker_0_translation"
        fit["marker_0_translation_column_per_input_rad"] = (
            fit["column_per_input_rad"])
        fit["marker_0_translation_scale_m"] = marker0_scale.tolist()
        fit["column_per_input_rad"] = pose_fit["column"].tolist()
        fit["ukf_pose_train_score_at_selected_width"] = float(np.mean(
            pose_fit["normalized_squared_error"][train]))
        fit["ukf_pose_holdout_score_at_selected_width"] = float(np.mean(
            pose_fit["normalized_squared_error"][holdout]))
        fit["fit"] = pose_fit
        fit["basis"] = basis
        fit["raw_axis"] = axis
        fit["nominal_metres_per_motor_rad"] = gain
        fit["positive_width_equivalent_mm"] = (
            abs(gain) * fit["positive_width_rad"] * 1e3)
        fit["negative_width_equivalent_mm"] = (
            abs(gain) * fit["negative_width_rad"] * 1e3)
        fit["reversal_width_equivalent_mm"] = (
            abs(gain) * fit["reversal_width_rad"] * 1e3)
        interface_fits[basis] = fit

    knob_coordinate = interface_fits["shaft_2"]["transmitted"]
    shape_windows = build_windows(times, labels, "shaft_2", horizon_s)
    shape_response, shape_windows = _responses(
        marker_shape, shape_windows, pose=False)
    shape_train = _mask(shape_windows, holdout_repetition)
    shape_scale = _response_scale(shape_response, shape_train)
    clamp_marker = search_play(
        knob_coordinate, shape_response, shape_windows, shape_scale,
        holdout_repetition, grid_size=grid_size)

    strain_windows = build_windows(times, labels, "shaft_2", horizon_s)
    strain_response, strain_windows = _responses(
        strain, strain_windows, pose=False)
    strain_train = _mask(strain_windows, holdout_repetition)
    strain_scale = _response_scale(strain_response, strain_train)
    clamp_strain = search_play(
        knob_coordinate, strain_response, strain_windows, strain_scale,
        holdout_repetition, grid_size=grid_size)

    for fit in (clamp_marker, clamp_strain):
        fit["positive_width_equivalent_mm"] = (
            abs(NOMINAL_KNOB_METRES_PER_RAD)
            * fit["positive_width_rad"] * 1e3)
        fit["negative_width_equivalent_mm"] = (
            abs(NOMINAL_KNOB_METRES_PER_RAD)
            * fit["negative_width_rad"] * 1e3)
        fit["reversal_width_equivalent_mm"] = (
            abs(NOMINAL_KNOB_METRES_PER_RAD)
            * fit["reversal_width_rad"] * 1e3)

    # The knob-drive and handle-clamp play elements are cascaded and cannot be
    # uniquely allocated from distal observations alone.  Sweep small,
    # mechanically plausible fixed knob-drive reversal gaps to expose how the
    # inferred clamp gap changes instead of presenting one arbitrary split as
    # a physical measurement.
    knob_prior_sensitivity = []
    for drive_reversal_mm in (0.0, 0.25, 0.5, 1.0):
        side_rad = (
            0.5 * drive_reversal_mm * 1e-3
            / abs(NOMINAL_KNOB_METRES_PER_RAD))
        assumed_knob = asymmetric_play(raw_motor[:, 2], side_rad, side_rad)
        marker_fit = search_play(
            assumed_knob, shape_response, shape_windows, shape_scale,
            holdout_repetition, grid_size=grid_size)
        strain_fit = search_play(
            assumed_knob, strain_response, strain_windows, strain_scale,
            holdout_repetition, grid_size=grid_size)
        for fit in (marker_fit, strain_fit):
            fit["reversal_width_equivalent_mm"] = (
                abs(NOMINAL_KNOB_METRES_PER_RAD)
                * fit["reversal_width_rad"] * 1e3)
        knob_prior_sensitivity.append({
            "assumed_knob_drive_reversal_width_mm": drive_reversal_mm,
            "marker_clamp_reversal_width_mm": marker_fit[
                "reversal_width_equivalent_mm"],
            "marker_holdout_fractional_improvement": marker_fit[
                "holdout_fractional_improvement"],
            "marker_width_fit_identifiable_inside_search": marker_fit[
                "width_fit_identifiable_inside_search"],
            "ukf_strain_clamp_reversal_width_mm": strain_fit[
                "reversal_width_equivalent_mm"],
            "ukf_strain_holdout_fractional_improvement": strain_fit[
                "holdout_fractional_improvement"],
            "ukf_strain_width_fit_identifiable_inside_search": strain_fit[
                "width_fit_identifiable_inside_search"],
        })

    chassis_column = interface_fits["shaft_0"]["fit"]["column"]
    knob_column = interface_fits["shaft_2"]["fit"]["column"]
    chassis_per_metre = chassis_column / NOMINAL_CHASSIS_METRES_PER_RAD
    knob_per_metre = knob_column / NOMINAL_KNOB_METRES_PER_RAD
    translation_cosine = float(
        chassis_per_metre[3:] @ knob_per_metre[3:]
        / max(np.linalg.norm(chassis_per_metre[3:])
              * np.linalg.norm(knob_per_metre[3:]), 1e-15))

    result = {
        "schema_version": 1,
        "analysis": "phase2_separated_proximal_transmission",
        "session": str(session),
        "bag": str(bag),
        "jacobian_json": str(jacobian_json),
        "jacobian_sha256": _sha256(jacobian_json),
        "configuration": {
            "horizon_s": float(horizon_s),
            "grid_size": int(grid_size),
            "holdout_repetition": int(holdout_repetition),
            "encoder_radians_per_count": ENCODER_RADIANS_PER_COUNT,
            "nominal_chassis_metres_per_rad": (
                NOMINAL_CHASSIS_METRES_PER_RAD),
            "nominal_knob_metres_per_rad": NOMINAL_KNOB_METRES_PER_RAD,
        },
        "counts": {
            "causal_traces": len(traces),
            "accepted_estimator_traces": len(estimators),
            "marker_observations": len(markers),
            "matched_marker_observations": marker_matches,
        },
        "interface_transmission": {
            key: _strip_arrays(value)
            for key, value in interface_fits.items()
        },
        "interface_column_comparison": {
            "chassis_column_per_transmitted_metre": (
                chassis_per_metre.tolist()),
            "knob_column_per_transmitted_metre": knob_per_metre.tolist(),
            "translation_direction_cosine": translation_cosine,
        },
        "compensated_bend_superposition_validation": _combined_validation(
            interface_fits, pose, labels, times, state_scale, horizon_s),
        "handle_clamp_diagnostics": {
            "marker_pairwise_geometry": _strip_arrays(clamp_marker),
            "ukf_distal_strain": _strip_arrays(clamp_strain),
            "small_knob_drive_prior_sensitivity": knob_prior_sensitivity,
        },
        "interpretation_limits": [
            "Interface fits use visually corrected UKF pose as an initialization diagnostic.",
            "Marker pairwise distances are independent of rigid interface pose but are not a complete distal-shape coordinate.",
            "Cascaded handle-clamp and downstream v171 play are not uniquely identifiable from Phase 2 alone.",
            "This report does not create or authorize a deployable model artifact.",
        ],
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    parser.add_argument(
        "--jacobian-json", type=Path,
        default=(WORKSPACE_ROOT / "cr_meta_lnn" / "evaluation"
                 / "real_joint_local_distal_v174.json"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--horizon-s", type=float, default=0.25)
    parser.add_argument("--grid-size", type=int, default=21)
    parser.add_argument("--holdout-repetition", type=int, default=3)
    parser.add_argument(
        "--replay-npz", type=Path,
        help="ROS-independent replay output (default: SESSION/phase2_transmission_replay.npz)")
    parser.add_argument("--no-replay", action="store_true")
    arguments = parser.parse_args()
    if arguments.grid_size < 3:
        parser.error("--grid-size must be at least 3")
    session = arguments.session.expanduser().resolve()
    result = identify(
        session, arguments.jacobian_json.expanduser().resolve(),
        horizon_s=arguments.horizon_s,
        grid_size=arguments.grid_size,
        holdout_repetition=arguments.holdout_repetition)
    if not arguments.no_replay:
        replay = (arguments.replay_npz or
                  session / "phase2_transmission_replay.npz")
        result["replay_export"] = export_replay(
            session, replay.expanduser().resolve())
    output = (arguments.output or
              session / "phase2_transmission_identification.json")
    output = output.expanduser().resolve()
    with output.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    marker = result["handle_clamp_diagnostics"]["marker_pairwise_geometry"]
    print(json.dumps({
        "result": "COMPLETE",
        "output": str(output),
        "marker_matches": result["counts"]["matched_marker_observations"],
        "marker_clamp_reversal_width_mm": marker[
            "reversal_width_equivalent_mm"],
        "marker_holdout_fractional_improvement": marker[
            "holdout_fractional_improvement"],
        "compensated_superposition": result[
            "compensated_bend_superposition_validation"],
    }, indent=2))


if __name__ == "__main__":
    main()
