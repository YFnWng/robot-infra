#!/usr/bin/env python3
"""Summarize one recorded causal proximal-identification experiment.

All response windows are backward-looking and remain inside one labelled
episode.  The tool never mutates the bag or model artifacts.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

from control_interface.msg import ManagerEvent
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import numpy as np
import yaml


TOPICS = {
    "/collection/causal_trace",
    "/catheter_mppi/estimator_trace",
    "/collection/events",
    "/manager/safety_status",
    "/manager/control",
    "/device/command_tx",
    "/device/state",
    "/device/event",
    "/shape_tracking/markers",
}
TOPICS |= {"/sim" + topic for topic in tuple(TOPICS)}


def _stamp_ns(message):
    return (int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec))


def _bag_path(path: Path):
    if (path / "metadata.yaml").is_file():
        return path
    if (path / "robot_bag" / "metadata.yaml").is_file():
        return path / "robot_bag"
    matches = list(path.glob("**/metadata.yaml"))
    if len(matches) != 1:
        raise ValueError(
            f"expected one rosbag below {path}, found {len(matches)}")
    return matches[0].parent


def _storage_id(bag: Path):
    with (bag / "metadata.yaml").open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    info = metadata.get("rosbag2_bagfile_information", metadata)
    return str(info.get("storage_identifier", "mcap"))


def _read(path: Path):
    bag = _bag_path(path)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=_storage_id(bag)),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    topic_types = {
        topic.name: get_message(topic.type)
        for topic in reader.get_all_topics_and_types()
    }
    present = TOPICS.intersection(topic_types)
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(present)))
    messages = defaultdict(list)
    while reader.has_next():
        topic, serialized, bag_timestamp = reader.read_next()
        message = deserialize_message(serialized, topic_types[topic])
        messages[topic].append((int(bag_timestamp), message))
    return bag, messages


def _messages_for(messages, topic):
    return messages.get(topic, messages.get("/sim" + topic, []))


def _rotation_vector(rotation):
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1e-8:
        return 0.5 * np.array([
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1]])
    return angle / (2.0 * math.sin(angle)) * np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1]])


def _pose_increment(left, right):
    relative = np.linalg.inv(left) @ right
    return np.concatenate((_rotation_vector(relative[:3, :3]),
                           relative[:3, 3]))


def _percentiles(values):
    values = np.asarray(values, dtype=float)
    if not values.size:
        return None
    return {
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "maximum": float(np.max(values)),
    }


def _device_event_classification(text):
    """Classify motion-watchdog events without invalidating good telemetry.

    SUSPECTED, RETRYING, and RECOVERED are firmware monitor state transitions,
    not confirmed encoder-integrity failures.  They remain visible in the
    report but only a confirmed/latched event fails the safety gate.
    """
    value = str(text).strip().upper()
    if value.startswith(("MOTION_SUSPECTED:", "MOTION_RETRYING:",
                         "MOTION_RECOVERED:")):
        return "advisory"
    if value.startswith(("MOTION_CONFIRMED:", "MOTION_LATCHED:")):
        return "fault"
    return "other"


def _episode_rows(messages):
    rows = []
    for _, message in _messages_for(messages, "/collection/causal_trace"):
        rows.append({
            "timestamp_ns": _stamp_ns(message),
            "index": int(message.episode_index),
            "name": message.episode_name,
            "basis": message.excitation_basis,
            "tier": message.speed_tier,
            "repetition": int(message.repetition),
            "intended_timestamp_ns": int(
                message.intended_command_timestamp_ns),
            "episode_start_timestamp_ns": int(
                message.episode_start_timestamp_ns),
            "planned_motor_onset_delay_s": np.asarray(getattr(
                message, "planned_motor_axis_onset_delay_s",
                [-1.0] * 6), dtype=float),
            "requested": np.asarray(
                message.requested_logical_velocity, dtype=float),
            "commanded": np.asarray(
                message.commanded_logical_velocity, dtype=float),
            "motor_request": np.asarray(
                message.requested_motor_axis_velocity, dtype=float),
            "motor_rpm": np.asarray(message.predicted_motor_rpm, dtype=int),
            "predicted_motor_rate": np.asarray(
                message.predicted_motor_radians_per_second, dtype=float),
            "reference": np.asarray(
                message.reference_logical_position, dtype=float),
            "position": np.asarray(
                message.measured_logical_position, dtype=float),
            "encoder": np.asarray(message.raw_encoder_counts, dtype=float),
        })
    rows.sort(key=lambda row: row["timestamp_ns"])
    return rows


def _estimator_rows(messages):
    rows = []
    for _, message in _messages_for(
            messages, "/catheter_mppi/estimator_trace"):
        rows.append({
            "timestamp_ns": int(message.observation_timestamp_ns),
            "state_timestamp_ns": int(message.state_timestamp_ns),
            "pose": np.asarray(message.interface_pose, dtype=float).reshape(4, 4),
            "raw_encoder_counts": np.asarray(
                message.raw_encoder_counts, dtype=float),
            "motor": np.asarray(message.motor_angle_rad, dtype=float),
            "covariance_trace": float(message.estimator_covariance_trace),
            "nis": float(message.innovation_nis),
            "reason": message.marker_update_reason,
            "accepted_observations": int(message.accepted_observations),
            "adaptation_weight": float(message.adaptation_weight),
            "adaptation_update_norm": float(message.adaptation_update_norm),
            "distal_strain": np.asarray(message.distal_strain, dtype=float),
        })
    rows.sort(key=lambda row: row["timestamp_ns"])
    return rows


def _marker_rows(messages):
    """Return registered marker observations keyed by acquisition stamp."""
    rows = []
    for _, message in _messages_for(messages, "/shape_tracking/markers"):
        channels = {channel.name: np.asarray(channel.values, dtype=float)
                    for channel in message.channels}
        rows.append({
            "timestamp_ns": _stamp_ns(message),
            "frame_id": message.header.frame_id,
            "points": np.asarray(
                [[point.x, point.y, point.z] for point in message.points],
                dtype=float),
            "marker_id": channels.get("marker_id"),
            "confidence": channels.get("confidence"),
            "reprojection_error_px": channels.get(
                "reprojection_error_px"),
        })
    rows.sort(key=lambda row: row["timestamp_ns"])
    return rows


def _labels_for_estimators(traces, estimators):
    trace_times = np.asarray([row["timestamp_ns"] for row in traces], dtype=np.int64)
    labels = []
    for row in estimators:
        index = int(np.searchsorted(
            trace_times, row["timestamp_ns"], side="right") - 1)
        labels.append(None if index < 0 else traces[index])
    return labels


def _response_windows(estimators, labels, horizons):
    timestamps = np.asarray(
        [row["timestamp_ns"] for row in estimators], dtype=np.int64)
    windows = []
    for end, (end_row, end_label) in enumerate(zip(estimators, labels)):
        if end_label is None or end_label["basis"] == "static":
            continue
        for horizon in horizons:
            desired = end_row["timestamp_ns"] - int(horizon * 1e9)
            start = int(np.searchsorted(timestamps, desired, side="right") - 1)
            if start < 0 or start >= end:
                continue
            start_row, start_label = estimators[start], labels[start]
            if (start_label is None
                    or start_label["index"] != end_label["index"]):
                continue
            elapsed = (end_row["timestamp_ns"]
                       - start_row["timestamp_ns"]) * 1e-9
            if not 0.8 * horizon <= elapsed <= 1.2 * horizon:
                continue
            gaps = np.diff(timestamps[start:end + 1]) * 1e-9
            if gaps.size and float(np.max(gaps)) > 0.15:
                # Do not silently bridge a dropout/reinitialization interval.
                continue
            action = end_row["motor"] - start_row["motor"]
            response = _pose_increment(start_row["pose"], end_row["pose"])
            axis = int(np.argmax(np.abs(action)))
            windows.append({
                "episode": end_label["name"],
                "basis": end_label["basis"],
                "tier": end_label["tier"],
                "repetition": end_label["repetition"],
                "horizon_s": horizon,
                "elapsed_s": elapsed,
                "action": action,
                "response": response,
                "dominant_axis": axis,
                "sign": int(np.sign(action[axis])),
            })
    return windows


def _summarize_windows(windows):
    grouped = defaultdict(list)
    for window in windows:
        key = (window["basis"], window["tier"], window["repetition"],
               window["sign"], window["horizon_s"])
        grouped[key].append(window)
    result = []
    for key, group in sorted(grouped.items(), key=lambda item: str(item[0])):
        actions = np.asarray([row["action"] for row in group])
        responses = np.asarray([row["response"] for row in group])
        axis = int(np.argmax(np.sum(actions * actions, axis=0)))
        scalar = actions[:, axis]
        denominator = float(scalar @ scalar)
        column = (np.zeros(6) if denominator <= 1e-16
                  else scalar @ responses / denominator)
        predictions = scalar[:, None] * column
        residual = responses - predictions
        response_norm = np.linalg.norm(responses, axis=1)
        prediction_norm = np.linalg.norm(predictions, axis=1)
        cosines = np.divide(
            np.sum(responses * predictions, axis=1),
            np.maximum(response_norm * prediction_norm, 1e-15))
        result.append({
            "basis": key[0], "speed_tier": key[1],
            "repetition": key[2], "sign": key[3], "horizon_s": key[4],
            "samples": len(group), "dominant_raw_axis": axis,
            "fitted_twist_per_motor_rad": column.tolist(),
            "response_norm": _percentiles(response_norm),
            "residual_norm": _percentiles(np.linalg.norm(residual, axis=1)),
            "direction_cosine": _percentiles(cosines),
        })
    return result


def _device_rows(messages, topic, predicate=None):
    rows = []
    for bag_timestamp, message in _messages_for(messages, topic):
        if predicate is not None and int(message.predicate) != predicate:
            continue
        rows.append({
            "timestamp_ns": int(bag_timestamp),
            "source_timestamp_ns": _stamp_ns(message),
            "predicate": int(message.predicate),
            "data": np.asarray(message.data, dtype=float),
        })
    rows.sort(key=lambda row: row["timestamp_ns"])
    return rows


def _raw_command(logical):
    logical = np.asarray(logical, dtype=float)
    if logical.size < 3:
        return np.zeros(2)
    return np.array([logical[0] - logical[2], logical[2]])


def _planned_phase3_delays_ms(basis):
    if basis == "timing_insertion_only":
        return [0.0, -1.0]
    if basis == "timing_tendon_only":
        return [-1.0, 0.0]
    if basis == "timing_simultaneous":
        return [0.0, 0.0]
    for prefix, delayed_axis in (("timing_insertion_lead_", 1),
                                 ("timing_tendon_lead_", 0)):
        if basis.startswith(prefix) and basis.endswith("ms"):
            delay = float(basis[len(prefix):-2])
            result = [0.0, 0.0]
            result[delayed_axis] = delay
            return result
    return [-1.0, -1.0]


def _first_command_onsets(rows, start_ns, end_ns, epsilon=1e-9,
                          value_key="data"):
    onset = [None, None]
    for row in rows:
        timestamp = row["timestamp_ns"]
        if timestamp < start_ns or timestamp > end_ns:
            continue
        raw = (_raw_command(row[value_key]) if value_key != "raw" else
               np.asarray(row["raw"], dtype=float))
        for axis in range(2):
            if onset[axis] is None and abs(raw[axis]) > epsilon:
                onset[axis] = timestamp
    return onset


def _credible_encoder_onsets(rows, baseline_ns, end_ns, directions,
                             thresholds):
    selected = [row for row in rows
                if baseline_ns - 250_000_000 <= row["timestamp_ns"] <= end_ns]
    before = [row for row in selected if row["timestamp_ns"] <= baseline_ns]
    if not before:
        return [None, None]
    baseline = before[-1]["data"]
    results = [None, None]
    # Positive catheter-tendon motor motion decreases raw encoder axis 2
    # because its physical transmission ratio is negative.  Work in motor
    # coordinates rather than assuming encoder-count sign equals command sign.
    physical_axes = ((0, 1.0), (2, -1.0))
    for local_axis, (physical_axis, encoder_polarity) in enumerate(
            physical_axes):
        direction = directions[local_axis] * encoder_polarity
        if direction == 0:
            continue
        qualified = []
        for row in selected:
            if row["timestamp_ns"] < baseline_ns:
                continue
            displacement = ((row["data"][physical_axis]
                             - baseline[physical_axis]) * direction)
            qualified.append((row["timestamp_ns"],
                              displacement > thresholds[local_axis]))
        for left, right in zip(qualified[:-1], qualified[1:]):
            if left[1] and right[1]:
                results[local_axis] = left[0]
                break
    return results


def _phase3_command_waveform(rows, start_ns, end_ns, expected_delays_ms,
                             expected_directions, expected_travel,
                             *, value_key="data", onset_tolerance_ms=20.0):
    """Validate one raw-shaft timing waveform without modifying samples."""
    selected = [row for row in rows
                if start_ns <= row["timestamp_ns"] <= end_ns]
    if not selected:
        return {
            "valid": False,
            "violations": ["no command samples"],
            "onset_ns": [None, None],
        }
    timestamps = np.asarray(
        [row["timestamp_ns"] for row in selected], dtype=np.int64)
    raw = np.asarray([
        (_raw_command(row[value_key]) if value_key != "raw" else
         np.asarray(row["raw"], dtype=float))
        for row in selected], dtype=float)
    active = np.abs(raw) > 1e-9
    violations = []
    onsets = [None, None]
    regions = []
    signed_travel = []
    for axis in range(2):
        flags = active[:, axis]
        starts = np.flatnonzero(flags & np.r_[True, ~flags[:-1]])
        regions.append(int(starts.size))
        if starts.size:
            onsets[axis] = int(timestamps[starts[0]])
        travel = float(np.trapezoid(
            raw[:, axis], timestamps.astype(float) * 1e-9))
        signed_travel.append(travel)
        expected_active = expected_delays_ms[axis] >= 0.0
        if not expected_active:
            if np.any(flags):
                violations.append(f"axis_{axis}_unexpected_motion")
            continue
        if not np.any(flags):
            violations.append(f"axis_{axis}_missing_motion")
            continue
        if starts.size != 1:
            violations.append(
                f"axis_{axis}_pulse_regions={int(starts.size)}")
        direction = int(expected_directions[axis])
        if direction == 0 or np.any(raw[flags, axis] * direction <= 0.0):
            violations.append(f"axis_{axis}_wrong_direction_or_reversal")
        onset_error_ms = (
            (onsets[axis] - start_ns) * 1e-6
            - expected_delays_ms[axis])
        if abs(onset_error_ms) > onset_tolerance_ms:
            violations.append(
                f"axis_{axis}_onset_error_ms={onset_error_ms:.3f}")
        target = float(expected_travel[axis]) * direction
        # Two command periods plus five percent of travel accommodates timer
        # phase at the first/last sample without accepting a changed pulse.
        dt = (float(np.median(np.diff(timestamps))) * 1e-9
              if timestamps.size > 1 else 0.01)
        peak = float(np.max(np.abs(raw[:, axis])))
        tolerance = max(0.05 * abs(target), 2.0 * dt * peak, 1e-3)
        if abs(travel - target) > tolerance:
            violations.append(
                f"axis_{axis}_travel={travel:.6g}_expected={target:.6g}")
    return {
        "valid": not violations,
        "violations": violations,
        "onset_ns": onsets,
        "pulse_regions": regions,
        "signed_travel": signed_travel,
    }


def _credible_estimator_onset(estimators, baseline_ns, end_ns,
                              translation_threshold_m,
                              rotation_threshold_rad,
                              distal_threshold):
    before = [row for row in estimators if row["timestamp_ns"] <= baseline_ns]
    if not before:
        return None, None
    baseline = before[-1]
    candidates = [row for row in estimators
                  if baseline_ns <= row["timestamp_ns"] <= end_ns]
    pose_flags = []
    distal_flags = []
    for row in candidates:
        increment = _pose_increment(baseline["pose"], row["pose"])
        pose_flags.append((
            row["timestamp_ns"],
            (np.linalg.norm(increment[:3]) > rotation_threshold_rad
             or np.linalg.norm(increment[3:]) > translation_threshold_m)))
        distal_delta = row["distal_strain"] - baseline["distal_strain"]
        distal_flags.append((
            row["timestamp_ns"],
            np.linalg.norm(distal_delta) > distal_threshold))

    def persistent_onset(flags):
        for left, right in zip(flags[:-1], flags[1:]):
            if left[1] and right[1]:
                return left[0]
        return None

    return persistent_onset(pose_flags), persistent_onset(distal_flags)


def _bootstrap_median_ci(values, seed=171, samples=2000):
    values = np.asarray([value for value in values if value is not None],
                        dtype=float)
    if not values.size:
        return None
    rng = np.random.default_rng(seed)
    medians = np.median(
        rng.choice(values, size=(samples, values.size), replace=True), axis=1)
    return {
        "count": int(values.size),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "bootstrap_median_ci95": [
            float(np.percentile(medians, 2.5)),
            float(np.percentile(medians, 97.5)),
        ],
    }


def _phase3_timing_summary(messages, traces, estimators, static_steps):
    timing_indices = sorted({row["index"] for row in traces
                             if row["basis"].startswith("timing_")
                             and row["basis"] not in {
                                 "timing_precondition", "timing_recovery"}})
    if not timing_indices:
        return None
    manager = _device_rows(messages, "/manager/control", predicate=86)
    transmitted = _device_rows(
        messages, "/device/command_tx", predicate=86)
    encoders = _device_rows(messages, "/device/state", predicate=69)

    static_translation = (1e-4 if not len(static_steps) else max(
        1e-6, float(np.percentile(
            np.linalg.norm(static_steps[:, 3:], axis=1), 99))))
    static_rotation = (2e-3 if not len(static_steps) else max(
        1e-6, float(np.percentile(
            np.linalg.norm(static_steps[:, :3], axis=1), 99))))
    static_distal_steps = []
    estimator_labels = _labels_for_estimators(traces, estimators)
    for left, right, left_label, right_label in zip(
            estimators[:-1], estimators[1:],
            estimator_labels[:-1], estimator_labels[1:]):
        if (left_label is not None and right_label is not None
                and left_label["index"] == right_label["index"]
                and left_label["basis"] == "static"):
            static_distal_steps.append(np.linalg.norm(
                right["distal_strain"] - left["distal_strain"]))
    distal_threshold = (1e-5 if not static_distal_steps else max(
        1e-9, float(np.percentile(static_distal_steps, 99))))

    encoder_step_thresholds = []
    static_intervals = []
    for index in sorted({row["index"] for row in traces
                         if row["basis"] == "static"}):
        group = [row for row in traces if row["index"] == index]
        static_intervals.append(
            (group[0]["timestamp_ns"], group[-1]["timestamp_ns"]))
    for physical_axis in (0, 2):
        static_encoder_steps = []
        for left, right in zip(encoders[:-1], encoders[1:]):
            if right["timestamp_ns"] - left["timestamp_ns"] > 100_000_000:
                continue
            if not any(
                    start <= left["timestamp_ns"] <= right["timestamp_ns"] <= end
                    for start, end in static_intervals):
                continue
            static_encoder_steps.append(abs(
                right["data"][physical_axis] - left["data"][physical_axis]))
        p99 = (0.0 if not static_encoder_steps else
               float(np.percentile(static_encoder_steps, 99)))
        encoder_step_thresholds.append(max(3.0, 2.0 * p99))

    trials = []
    for index in timing_indices:
        group = [row for row in traces if row["index"] == index]
        start_ns = group[0]["episode_start_timestamp_ns"]
        end_ns = group[-1]["timestamp_ns"]
        planned_delays = _planned_phase3_delays_ms(group[0]["basis"])
        reference_raw = np.asarray([
            _raw_command(row["reference"])[[0, 1]] for row in group])
        reference_delta = reference_raw[-1] - reference_raw[0]
        expected_directions = np.sign(reference_delta).astype(int)
        expected_travel = np.abs(reference_delta)
        for axis, delay in enumerate(planned_delays):
            if delay < 0.0:
                expected_directions[axis] = 0
                expected_travel[axis] = 0.0
        # The source publishes logical ``commanded`` velocity. Convert that
        # command exactly as firmware does; ``motor_request`` is a separate
        # learned-model projection and may contain an additional model-count
        # guard that is not part of the manager/device transport path.
        source_rows = [{"timestamp_ns": row["timestamp_ns"],
                        "raw": _raw_command(row["commanded"])}
                       for row in group]
        source_waveform = _phase3_command_waveform(
            source_rows, start_ns, end_ns, planned_delays,
            expected_directions, expected_travel, value_key="raw")
        manager_waveform = _phase3_command_waveform(
            manager, start_ns, end_ns, planned_delays,
            expected_directions, expected_travel)
        tx_waveform = _phase3_command_waveform(
            transmitted, start_ns, end_ns, planned_delays,
            expected_directions, expected_travel)
        source_onsets = source_waveform["onset_ns"]
        manager_onsets = manager_waveform["onset_ns"]
        tx_onsets = tx_waveform["onset_ns"]
        earliest_source = min(
            (value for axis, value in enumerate(source_onsets)
             if planned_delays[axis] >= 0.0 and value is not None),
            default=start_ns)
        encoder_onsets = _credible_encoder_onsets(
            encoders, earliest_source, end_ns, expected_directions,
            encoder_step_thresholds)
        pose_onset, distal_onset = _credible_estimator_onset(
            estimators, earliest_source, end_ns,
            static_translation, static_rotation, distal_threshold)

        def delay_ms(later, earlier):
            return None if later is None or earlier is None else (
                (later - earlier) * 1e-6)

        trial = {
            "episode_index": index,
            "episode": group[0]["name"],
            "condition": group[0]["basis"],
            "speed_tier": group[0]["tier"],
            "repetition": group[0]["repetition"],
            "planned_motor_onset_delay_ms": planned_delays,
            "expected_motor_axis_travel": expected_travel.tolist(),
            "source_waveform": source_waveform,
            "manager_waveform": manager_waveform,
            "device_tx_waveform": tx_waveform,
            "command_topology_valid": bool(
                source_waveform["valid"]
                and manager_waveform["valid"]
                and tx_waveform["valid"]),
            "source_onset_ns": source_onsets,
            "manager_onset_ns": manager_onsets,
            "device_tx_onset_ns": tx_onsets,
            "encoder_onset_ns": encoder_onsets,
            "interface_response_onset_ns": pose_onset,
            "distal_response_onset_ns": distal_onset,
            "source_to_manager_ms": [
                delay_ms(manager_onsets[axis], source_onsets[axis])
                for axis in range(2)],
            "source_to_device_tx_ms": [
                delay_ms(tx_onsets[axis], source_onsets[axis])
                for axis in range(2)],
            "command_to_encoder_ms": [
                delay_ms(encoder_onsets[axis], source_onsets[axis])
                for axis in range(2)],
            "observed_source_onset_skew_ms": delay_ms(
                source_onsets[1], source_onsets[0]),
            "encoder_onset_skew_ms": delay_ms(
                encoder_onsets[1], encoder_onsets[0]),
            "first_encoder_to_interface_ms": delay_ms(
                pose_onset,
                min((value for value in encoder_onsets if value is not None),
                    default=None)),
            "first_encoder_to_distal_ms": delay_ms(
                distal_onset,
                min((value for value in encoder_onsets if value is not None),
                    default=None)),
        }
        trials.append(trial)

    grouped = []
    keys = sorted({(trial["condition"], trial["speed_tier"])
                   for trial in trials})
    for condition, tier in keys:
        group = [trial for trial in trials
                 if trial["condition"] == condition
                 and trial["speed_tier"] == tier]
        grouped.append({
            "condition": condition,
            "speed_tier": tier,
            "trials": len(group),
            "valid_command_topology_trials": sum(
                trial["command_topology_valid"] for trial in group),
            "command_to_encoder_axis0_ms": _bootstrap_median_ci([
                trial["command_to_encoder_ms"][0] for trial in group]),
            "command_to_encoder_axis2_ms": _bootstrap_median_ci([
                trial["command_to_encoder_ms"][1] for trial in group]),
            "encoder_onset_skew_axis2_minus_axis0_ms": (
                _bootstrap_median_ci([
                    trial["encoder_onset_skew_ms"] for trial in group])),
            "first_encoder_to_interface_ms": _bootstrap_median_ci([
                trial["first_encoder_to_interface_ms"] for trial in group]),
            "first_encoder_to_distal_ms": _bootstrap_median_ci([
                trial["first_encoder_to_distal_ms"] for trial in group]),
        })
    invalid = [trial["episode"] for trial in trials
               if not trial["command_topology_valid"]]
    return {
        "command_topology_valid": not invalid,
        "valid_command_topology_trials": len(trials) - len(invalid),
        "invalid_command_topology_trials": invalid,
        "encoder_displacement_threshold_counts": encoder_step_thresholds,
        "interface_translation_threshold_mm": 1e3 * static_translation,
        "interface_rotation_threshold_rad": static_rotation,
        "distal_strain_threshold": distal_threshold,
        "trials": trials,
        "groups": grouped,
    }


def evaluate(path: Path):
    bag, messages = _read(path)
    traces = _episode_rows(messages)
    estimators = _estimator_rows(messages)
    if not traces:
        raise ValueError("bag has no /collection/causal_trace messages")
    if not estimators:
        raise ValueError("bag has no /catheter_mppi/estimator_trace messages")
    labels = _labels_for_estimators(traces, estimators)
    windows = _response_windows(estimators, labels, (0.25, 0.5, 1.0))

    episodes = []
    for index in sorted({row["index"] for row in traces}):
        group = [row for row in traces if row["index"] == index]
        encoder = np.asarray([row["encoder"] for row in group])
        commanded = np.asarray([row["commanded"] for row in group])
        motor_request = np.asarray([row["motor_request"] for row in group])
        motor_rpm = np.asarray([row["motor_rpm"] for row in group])
        entry = {
            "index": index, "name": group[0]["name"],
            "basis": group[0]["basis"], "speed_tier": group[0]["tier"],
            "repetition": group[0]["repetition"], "samples": len(group),
            "duration_s": (group[-1]["timestamp_ns"]
                           - group[0]["timestamp_ns"]) * 1e-9,
            "command_peak_abs": np.nanmax(np.abs(commanded), axis=0).tolist(),
            "motor_request_peak_abs": np.nanmax(
                np.abs(motor_request), axis=0).tolist(),
            "motor_rpm_peak_abs": np.max(np.abs(motor_rpm), axis=0).tolist(),
            "raw_encoder_span_counts": (
                np.nanmax(encoder, axis=0) - np.nanmin(encoder, axis=0)).tolist(),
        }
        if entry["basis"] == "shaft_2":
            span = np.asarray(entry["raw_encoder_span_counts"])
            entry["shaft_0_to_2_encoder_span_ratio"] = float(
                span[0] / max(span[2], 1e-12))
            entry["predicted_shaft_0_request_peak"] = float(
                np.max(np.abs(motor_request[:, 0])))
        episodes.append(entry)

    static_steps = []
    for left, right, left_label, right_label in zip(
            estimators[:-1], estimators[1:], labels[:-1], labels[1:]):
        if (left_label is None or right_label is None
                or left_label["index"] != right_label["index"]
                or left_label["basis"] != "static"):
            continue
        delta = _pose_increment(left["pose"], right["pose"])
        static_steps.append(delta)
    static_steps = np.asarray(static_steps)
    phase3_timing = _phase3_timing_summary(
        messages, traces, estimators, static_steps)

    safety = []
    motion_advisories = []
    other_device_events = []
    for _, message in _messages_for(messages, "/manager/safety_status"):
        if "INHIBITED" in message.text or "FAULT" in message.text:
            safety.append(message.text)
    for _, message in _messages_for(messages, "/device/event"):
        if message.predicate == ManagerEvent.STALL:
            classification = _device_event_classification(message.text)
            value = f"device:{message.text}"
            if classification == "fault":
                safety.append(value)
            elif classification == "advisory":
                motion_advisories.append(value)
            else:
                other_device_events.append(value)

    encoder = np.asarray([row["encoder"] for row in traces], dtype=float)
    encoder_steps = np.diff(encoder, axis=0)
    encoder_integrity = {
        "finite": bool(np.isfinite(encoder).all()),
        "maximum_abs_step_counts": (
            np.max(np.abs(encoder_steps), axis=0).tolist()
            if len(encoder_steps) else [0.0] * 6),
        "p99_abs_step_counts": (
            np.percentile(np.abs(encoder_steps), 99, axis=0).tolist()
            if len(encoder_steps) else [0.0] * 6),
    }
    adaptation_updates = sum(
        row["adaptation_weight"] > 0.0 or row["adaptation_update_norm"] > 0.0
        for row in estimators)
    fitting_bases_present = {episode["basis"] for episode in episodes} >= {
        "shaft_0", "shaft_1", "shaft_2"}
    phase3_topology_valid = (
        None if phase3_timing is None else
        bool(phase3_timing["command_topology_valid"]))
    output = {
        "schema_version": 1,
        "bag": str(bag),
        "counts": {
            "causal_command_traces": len(traces),
            "accepted_estimator_traces": len(estimators),
            "causal_response_windows": len(windows),
            "adaptation_updates": adaptation_updates,
            "safety_events": len(safety),
            "motion_watchdog_advisories": len(motion_advisories),
        },
        "time_range_ns": [traces[0]["timestamp_ns"], traces[-1]["timestamp_ns"]],
        "episodes": episodes,
        "static_noise": {
            "samples": len(static_steps),
            "rotation_increment_rad": (
                None if not len(static_steps) else
                _percentiles(np.linalg.norm(static_steps[:, :3], axis=1))),
            "translation_increment_mm": (
                None if not len(static_steps) else _percentiles(
                    1e3 * np.linalg.norm(static_steps[:, 3:], axis=1))),
        },
        "causal_window_groups": _summarize_windows(windows),
        "phase3_timing": phase3_timing,
        "safety_events": safety,
        "motion_watchdog_advisories": motion_advisories,
        "other_device_events": other_device_events,
        "encoder_integrity": encoder_integrity,
        "preliminary_gates": {
            "adaptation_disabled": adaptation_updates == 0,
            "no_manager_or_device_faults": not safety,
            "encoder_telemetry_finite": encoder_integrity["finite"],
            "has_both_static_blocks": {episode["name"] for episode in episodes}
            >= {"static_start", "static_end"},
            # A timing-only schedule is qualified by its raw command topology,
            # not by the fitting bases required by the full identification
            # schedule.
            "has_required_experiment_bases": (
                phase3_timing is not None or fitting_bases_present),
            **({"phase3_command_topology_valid": phase3_topology_valid}
               if phase3_timing is not None else {}),
        },
    }
    return bag, output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()
    bag, result = evaluate(arguments.session.expanduser().resolve())
    default_output = bag.parent / "causal_experiment_summary.json"
    output = (arguments.output or default_output).expanduser().resolve()
    with output.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({
        "result": ("PASS" if all(result["preliminary_gates"].values())
                   else "REVIEW"),
        "output": str(output),
        "counts": result["counts"],
        "preliminary_gates": result["preliminary_gates"],
    }, indent=2))


if __name__ == "__main__":
    main()
