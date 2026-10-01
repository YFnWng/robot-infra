"""Validate and seal a finalized causal-experiment SQLite rosbag."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

import rosbag2_py


CRITICAL_SUFFIXES = (
    "/teleop/control",
    "/manager/control",
    "/manager/safety_status",
    "/device/state",
    "/device/command_tx",
    "/shape_tracking/markers",
    "/shape_tracking/marker_status",
    "/collection/events",
    "/collection/causal_trace",
    "/catheter_mppi/estimator_trace",
    "/catheter_mppi/status",
)

ONLINE_TRACKING_DATA_SUFFIXES = (
    "/shape_tracking/markers",
    "/catheter_mppi/estimator_trace",
)


def _critical_suffixes(manifest):
    parameters = manifest.get("parameters", {})
    if parameters.get("causal_require_estimator_tracking", True):
        return CRITICAL_SUFFIXES
    return tuple(
        suffix for suffix in CRITICAL_SUFFIXES
        if suffix not in ONLINE_TRACKING_DATA_SUFFIXES)


def _atomic_json(path: Path, payload):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)


def inspect_session(session_dir):
    session = Path(session_dir).expanduser().resolve()
    manifest_path = session / "manifest.json"
    errors = []
    if not manifest_path.is_file():
        return {"result": "FAIL", "errors": ["missing manifest.json"]}, None
    with manifest_path.open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    identity_path = session / "runtime_identity.json"
    if not identity_path.is_file():
        errors.append("missing runtime_identity.json")
        identity = None
    else:
        with identity_path.open("r", encoding="utf-8") as stream:
            identity = json.load(stream)
        if identity.get("errors"):
            errors.append("runtime identity contains node query errors")

    bag = Path(manifest.get("bag_output", session / "robot_bag"))
    if not bag.is_absolute():
        bag = session / bag
    metadata = bag / "metadata.yaml"
    databases = sorted(bag.glob("*.db3"))
    if not metadata.is_file():
        errors.append("bag is not finalized: metadata.yaml missing")
    if not databases or any(path.stat().st_size == 0 for path in databases):
        errors.append("missing or empty SQLite3 bag database")

    counts = {}
    topic_types = {}
    if metadata.is_file() and databases:
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(
                    uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
            topic_types = {
                item.name: item.type for item in reader.get_all_topics_and_types()
            }
            counts = {name: 0 for name in topic_types}
            while reader.has_next():
                topic, _data, _timestamp = reader.read_next()
                counts[topic] = counts.get(topic, 0) + 1
        except Exception as exc:
            errors.append(f"failed to read finalized bag: {type(exc).__name__}: {exc}")

    configured = set(manifest.get("record_topics", []))
    prefix = "/sim" if manifest.get("use_sim") else ""
    critical = [prefix + suffix for suffix in _critical_suffixes(manifest)]
    for topic in critical:
        if topic not in configured:
            errors.append(f"critical topic absent from recording request: {topic}")
        elif counts.get(topic, 0) < 1:
            errors.append(f"critical topic has no recorded messages: {topic}")
    for topic in ("/parameter_events", "/rosout"):
        if topic not in configured:
            errors.append(f"runtime-audit topic absent from request: {topic}")
        elif topic not in topic_types:
            errors.append(f"runtime-audit topic absent from bag: {topic}")

    report = {
        "schema_version": 1,
        "result": "PASS" if not errors else "FAIL",
        "session": str(session),
        "bag": str(bag),
        "storage_id": "sqlite3",
        "database_files": [str(path) for path in databases],
        "topic_counts": counts,
        "topic_types": topic_types,
        "errors": errors,
    }
    manifest["effective_runtime_identity"] = identity
    manifest["recording_completeness"] = {
        "result": report["result"],
        "report": str(session / "completeness.json"),
    }
    return report, manifest


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("session_dir")
    parsed = parser.parse_args(args)
    session = Path(parsed.session_dir).expanduser().resolve()
    report, manifest = inspect_session(session)
    session.mkdir(parents=True, exist_ok=True)
    _atomic_json(session / "completeness.json", report)
    if manifest is not None:
        _atomic_json(session / "manifest.json", manifest)
    print(json.dumps({
        "result": report["result"],
        "report": str(session / "completeness.json"),
        "errors": report.get("errors", []),
    }, sort_keys=True))
    if report["result"] != "PASS":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
