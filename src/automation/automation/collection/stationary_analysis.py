"""Estimate Phase-1 stationary noise floors from a finalized causal bag."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def _statistics(values):
    array = np.asarray(values, dtype=np.float64)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return {"count": 0}
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "std": float(np.std(array)),
        "p50": float(np.quantile(array, 0.50)),
        "p95": float(np.quantile(array, 0.95)),
        "p99": float(np.quantile(array, 0.99)),
        "maximum": float(np.max(array)),
    }


def _increment_norm(samples, scale=1.0):
    if len(samples) < 2:
        return []
    values = np.asarray(samples, dtype=np.float64)
    return (np.linalg.norm(np.diff(values, axis=0), axis=1) * scale).tolist()


def _rotation_increment_deg(poses):
    result = []
    for left, right in zip(poses[:-1], poses[1:]):
        relative = np.asarray(left)[:3, :3].T @ np.asarray(right)[:3, :3]
        cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
        result.append(math.degrees(math.acos(float(cosine))))
    return result


def analyze(session_dir):
    session = Path(session_dir).expanduser().resolve()
    with (session / "manifest.json").open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    if manifest.get("schedule") not in ("stationary", "phase_1"):
        raise ValueError("stationary analysis requires schedule=stationary")
    prefix = "/sim" if manifest.get("use_sim") else ""
    topics = {
        prefix + "/shape_tracking/markers": "markers",
        prefix + "/catheter_mppi/estimator_trace": "estimator",
        prefix + "/device/state": "device",
    }
    bag = Path(manifest["bag_output"])
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = sorted(set(topics) - set(types))
    if missing:
        raise ValueError(f"stationary bag is missing topics: {missing}")
    classes = {topic: get_message(types[topic]) for topic in topics}

    marker_tip = []
    estimated_tip = []
    observed_tip = []
    interface_pose = []
    distal_strain = []
    correction_latency_ms = []
    observation_timestamps = []
    positions = []
    encoders = []
    while reader.has_next():
        topic, data, receipt_ns = reader.read_next()
        if topic not in topics:
            continue
        message = deserialize_message(data, classes[topic])
        kind = topics[topic]
        if kind == "markers" and message.points:
            point = message.points[-1]
            marker_tip.append([point.x, point.y, point.z])
        elif kind == "estimator":
            estimated_tip.append(list(message.estimated_tip_m))
            observed_tip.append(list(message.observed_tip_m))
            interface_pose.append(
                np.asarray(message.interface_pose).reshape(4, 4))
            distal_strain.append(list(message.distal_strain))
            observation_timestamps.append(int(message.observation_timestamp_ns))
            correction_latency_ms.append(
                (int(receipt_ns) - int(message.observation_timestamp_ns)) * 1e-6)
        elif kind == "device":
            # Firmware predicate values are stable ASCII codes: POS='P', ENC='E'.
            if int(message.predicate) == ord("P"):
                positions.append(list(message.data))
            elif int(message.predicate) == ord("E"):
                encoders.append(list(message.data))

    interface_translation = [pose[:3, 3] for pose in interface_pose]
    accepted_spacing_ms = (
        np.diff(np.asarray(observation_timestamps, dtype=np.int64)) * 1e-6
        if len(observation_timestamps) >= 2 else [])
    metrics = {
        "raw_marker_tip_increment_mm": _statistics(
            _increment_norm(marker_tip, 1000.0)),
        "ukf_estimated_tip_increment_mm": _statistics(
            _increment_norm(estimated_tip, 1000.0)),
        "accepted_observed_tip_increment_mm": _statistics(
            _increment_norm(observed_tip, 1000.0)),
        "ukf_interface_translation_increment_mm": _statistics(
            _increment_norm(interface_translation, 1000.0)),
        "ukf_interface_rotation_increment_deg": _statistics(
            _rotation_increment_deg(interface_pose)),
        "ukf_distal_strain_increment_norm": _statistics(
            _increment_norm(distal_strain)),
        "accepted_observation_spacing_ms": _statistics(
            accepted_spacing_ms),
        "camera_to_estimator_trace_latency_ms": _statistics(
            correction_latency_ms),
        "position_increment_norm": _statistics(_increment_norm(positions)),
        "encoder_increment_count_norm": _statistics(
            _increment_norm(encoders)),
    }
    tip_floors = [
        metrics[name].get("p99", math.nan)
        for name in ("raw_marker_tip_increment_mm",
                     "ukf_estimated_tip_increment_mm")]
    finite_tip_floors = [value for value in tip_floors if math.isfinite(value)]
    report = {
        "schema_version": 1,
        "result": "PASS" if finite_tip_floors else "FAIL",
        "session": str(session),
        "metrics": metrics,
        "credible_response_rule": {
            "tip_increment_floor_mm": (
                max(finite_tip_floors) if finite_tip_floors else None),
            "required_directionally_persistent_accepted_observations": 2,
            "also_require_covariance_normalized_evidence": True,
        },
    }
    return report


def _atomic_json(path, payload):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("session_dir")
    parsed = parser.parse_args(args)
    session = Path(parsed.session_dir).expanduser().resolve()
    report = analyze(session)
    output = session / "stationary_noise.json"
    _atomic_json(output, report)
    print(json.dumps({"result": report["result"], "output": str(output)}))
    if report["result"] != "PASS":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
