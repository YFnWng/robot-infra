#!/usr/bin/env python3
"""Build a causal full-stack timing report from one recorded control session."""
from __future__ import annotations

import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def _bag(path: Path) -> Path:
    if (path / "metadata.yaml").is_file():
        return path
    if (path / "robot_bag" / "metadata.yaml").is_file():
        return path / "robot_bag"
    found = list(path.glob("**/metadata.yaml"))
    if len(found) != 1:
        raise FileNotFoundError(f"expected one finalized rosbag below {path}")
    return found[0].parent


def _stamp_ns(message) -> int:
    header = getattr(message, "header", None)
    if header is None:
        return 0
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _summary(values):
    data = np.asarray([x for x in values if math.isfinite(x)], dtype=float)
    if not data.size:
        return {"count": 0}
    return {
        "count": int(data.size), "mean_ms": float(np.mean(data)),
        "p50_ms": float(np.percentile(data, 50)),
        "p95_ms": float(np.percentile(data, 95)),
        "p99_ms": float(np.percentile(data, 99)),
        "max_ms": float(np.max(data)),
    }


def _values(message):
    if hasattr(message, "joint_vel"):
        return np.asarray(message.joint_vel, dtype=float)
    return np.asarray(getattr(message, "data", []), dtype=float)


def _match_delays(source, destination, maximum_ms=250.0):
    """Match each destination to the newest equal-valued source command."""
    source_times = [row[0] for row in source]
    delays = []
    unmatched = 0
    for time_ns, message in destination:
        index = bisect_left(source_times, time_ns) - 1
        wanted = _values(message)
        matched = False
        while index >= 0 and (time_ns-source_times[index]) <= maximum_ms*1e6:
            candidate = _values(source[index][1])
            if candidate.shape == wanted.shape and np.allclose(
                    candidate, wanted, rtol=0.0, atol=1e-9):
                delays.append((time_ns-source_times[index])*1e-6)
                matched = True
                break
            index -= 1
        if not matched:
            unmatched += 1
    return _summary(delays), unmatched


def analyze(session: Path):
    bag = _bag(session)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {item.name: get_message(item.type)
             for item in reader.get_all_topics_and_types()}
    rows = {name: [] for name in types}
    header_age = {name: [] for name in types}
    while reader.has_next():
        topic, payload, recorded_ns = reader.read_next()
        message = deserialize_message(payload, types[topic])
        rows[topic].append((recorded_ns, message))
        stamp = _stamp_ns(message)
        if stamp:
            header_age[topic].append((recorded_ns-stamp)*1e-6)

    prefix = "/sim" if "/sim/catheter_mppi/control_cycle_timing" in rows else ""
    trace_topic = prefix + "/catheter_mppi/control_cycle_timing"
    traces = [message for _, message in rows.get(trace_topic, [])]
    trace_fields = (
        "marker_source_age_ms", "accepted_marker_commit_age_ms",
        "position_commit_age_ms", "encoder_commit_age_ms",
        "planner_snapshot_age_ms", "planner_callback_elapsed_ms",
        "planner_elapsed_ms", "sample_projection_ms", "rollout_ms",
        "cost_weighting_ms", "update_projection_ms",
        "estimator_callback_latest_ms", "estimator_timer_lateness_latest_ms",
        "marker_pending_age_latest_ms", "marker_owner_duration_latest_ms",
        "marker_rewind_ms", "marker_correction_ms", "marker_replay_ms",
        "marker_total_ms")
    internal = {name: _summary([float(getattr(row, name)) for row in traces])
                for name in trace_fields}

    teleop = rows.get(prefix + "/teleop/control", [])
    manager = [(t, m) for t, m in rows.get(prefix + "/manager/control", [])
               if getattr(m, "predicate", None) == 86]
    tx = [(t, m) for t, m in rows.get(prefix + "/device/command_tx", [])
          if getattr(m, "predicate", None) == 86]
    teleop_manager, unmatched_manager = _match_delays(teleop, manager)
    manager_tx, unmatched_tx = _match_delays(manager, tx)

    feedback = sorted(rows.get(prefix + "/device/state", []))
    feedback_times = [time_ns for time_ns, _ in feedback]
    tx_feedback = []
    for time_ns, _ in tx:
        index = bisect_left(feedback_times, time_ns)
        if index < len(feedback_times):
            tx_feedback.append((feedback_times[index]-time_ns)*1e-6)

    vision_metrics = {}
    vision_status_counts = {}
    marker_status_topic = prefix + "/shape_tracking/marker_status"
    for _, message in rows.get(marker_status_topic, []):
        for status in message.status:
            if status.name != "automation/four_ring_markers":
                continue
            vision_status_counts[status.message] = (
                vision_status_counts.get(status.message, 0) + 1)
            for item in status.values:
                if item.key in ("processing_latency_ms",
                                "rig_timestamp_skew_ms"):
                    try:
                        vision_metrics.setdefault(item.key, []).append(
                            float(item.value))
                    except ValueError:
                        pass

    timing_windows = {}
    status_topic = prefix + "/catheter_mppi/status"
    for _, message in rows.get(status_topic, []):
        for status in message.status:
            if status.name != "catheter_control/mppi":
                continue
            for item in status.values:
                if item.key.startswith("timing_"):
                    try:
                        timing_windows.setdefault(item.key, []).append(
                            float(item.value))
                    except ValueError:
                        pass

    report = {
        "schema_version": 1,
        "session": str(session.resolve()),
        "bag": str(bag),
        "trace_topic": trace_topic,
        "trace_rows": len(traces),
        "deadline_miss_rows": sum(not row.plan_valid and
                                  row.plan_reason == "deadline_missed"
                                  for row in traces),
        "vision_stage": {
            "metrics": {key: _summary(values)
                        for key, values in vision_metrics.items()},
            "status_counts": vision_status_counts,
        },
        "internal_control_cycle": internal,
        "topic_recording_header_age": {
            topic: _summary(values) for topic, values in header_age.items()
            if values and topic in {
                prefix + "/shape_tracking/markers", trace_topic,
                prefix + "/teleop/control", prefix + "/manager/control",
                prefix + "/device/command_tx", prefix + "/device/state"}},
        "distributed_command_path": {
            "teleop_to_manager_matching_command": teleop_manager,
            "teleop_to_manager_unmatched": unmatched_manager,
            "manager_to_serial_tx_matching_command": manager_tx,
            "manager_to_serial_tx_unmatched": unmatched_tx,
            "serial_tx_to_next_device_feedback": _summary(tx_feedback),
        },
        "diagnostic_window_series": {
            key: _summary(values) for key, values in timing_windows.items()},
        "notes": [
            "Cross-process delays use rosbag receive timestamps.",
            "Header-age distributions use publisher header stamps and therefore include transport plus recorder scheduling.",
            "Command stages are paired only when their six values match exactly; unmatched counts are reported explicitly.",
        ],
    }
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("session")
    parser.add_argument("--output")
    args = parser.parse_args()
    session = Path(args.session).expanduser().resolve()
    output = (Path(args.output).expanduser().resolve() if args.output else
              session / "full_stack_timing_qualification.json")
    report = analyze(session)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({"result": "COMPLETE", "output": str(output),
                      "trace_rows": report["trace_rows"],
                      "deadline_miss_rows": report["deadline_miss_rows"]},
                     sort_keys=True))


if __name__ == "__main__":
    main()
