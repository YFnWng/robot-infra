"""Non-actuating recording identity, storage, and process-lifecycle tests."""
from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
import sys
import time

import pytest
import yaml

from experiments.recording_session import (
    allocate_session, live_bag_ready, qualify_recording, session_token, write_manifest)
from experiments.session_recording import RecordingProcess, finalize_session


@pytest.mark.parametrize("value", ["", "../outside", "/tmp", "UpperCase", "a b", "a;cmd"])
def test_semantic_labels_reject_paths_and_shell_syntax(value):
    with pytest.raises(ValueError):
        session_token(value)


def test_identity_is_descriptive_utc_and_never_overwrites(tmp_path):
    now = datetime(2026, 10, 2, tzinfo=timezone.utc)
    session = allocate_session(tmp_path, "twoaxis_plain_n512_repeat01", now)
    assert session.name == "twoaxis_plain_n512_repeat01_20261002T000000000000Z"
    with pytest.raises(FileExistsError):
        allocate_session(tmp_path, "twoaxis_plain_n512_repeat01", now)
    with pytest.raises(ValueError):
        allocate_session(tmp_path, "valid", datetime(2026, 10, 2))


def finalized_bag(session):
    bag = session / "robot_bag"
    bag.mkdir()
    with sqlite3.connect(bag / "data.db3") as conn:
        conn.executescript(
            "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT);"
            "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER);"
            "INSERT INTO topics VALUES(1, '/device/state');"
            "INSERT INTO messages VALUES(1, 1);")
    (bag / "metadata.yaml").write_text(yaml.safe_dump({
        "rosbag2_bagfile_information": {
            "storage_identifier": "sqlite3", "relative_file_paths": ["data.db3"]}}))
    return {"video_enabled": False, "required_topics": ["/device/state"]}


def test_bag_qualification_and_missing_topics(tmp_path):
    manifest = finalized_bag(tmp_path)
    assert qualify_recording(tmp_path, manifest)["passed"]
    manifest["required_topics"].append("/shape_tracking/markers")
    result = qualify_recording(tmp_path, manifest)
    assert not result["passed"]
    assert "missing recorded messages" in result["errors"][0]


def test_live_readiness_requires_initialized_storage_and_required_topics(tmp_path):
    bag = tmp_path / "robot_bag"
    assert not live_bag_ready(bag, ["/device/state"])
    finalized_bag(tmp_path)
    assert live_bag_ready(bag, ["/device/state"])
    assert not live_bag_ready(bag, ["/missing"])
    assert not live_bag_ready(bag, [])


def test_missing_finalized_metadata_is_not_success(tmp_path):
    result = qualify_recording(tmp_path, {"video_enabled": False, "required_topics": []})
    assert not result["passed"]


def test_storage_path_cannot_escape_session(tmp_path):
    manifest = finalized_bag(tmp_path)
    metadata = tmp_path / "robot_bag" / "metadata.yaml"
    metadata.write_text(yaml.safe_dump({"rosbag2_bagfile_information": {
        "storage_identifier": "sqlite3", "relative_file_paths": ["../../outside.db3"]}}))
    assert "escapes" in qualify_recording(tmp_path, manifest)["errors"][0]


def test_video_requires_both_finalized_reports_and_frames(tmp_path):
    manifest = finalized_bag(tmp_path)
    video = tmp_path / "semantic_video"
    video.mkdir()
    manifest.update(video_enabled=True, video_directory=str(video))
    assert not qualify_recording(tmp_path, manifest)["passed"]
    document = {"recording_reports": {
        rig: {"indexed_frames": 1, "playable_frames": 1}
        for rig in ("primary", "oblique")}, "camera_sync": {"paired_frames": 1}}
    for rig in ("primary", "oblique"):
        (video / f"{rig}_{video.name}.svo2").write_bytes(b"test fixture")
        (video / f"{rig}_frame_index.csv").write_text("svo_frame,timestamp_ns\n0,1\n")
    (video / "camera_frame_pairs.csv").write_text("fixture\n")
    (video / "session_metadata.json").write_text(json.dumps(document))
    assert qualify_recording(tmp_path, manifest)["passed"]
    document["recording_reports"]["oblique"]["indexed_frames"] = 0
    (video / "session_metadata.json").write_text(json.dumps(document))
    assert not qualify_recording(tmp_path, manifest)["passed"]


@pytest.mark.parametrize("reason,ready,expected", [
    ("operator_stop", True, "complete"),
    ("operator_stop", False, "partial"),
    ("recorder_failed", True, "failed")])
def test_shutdown_order_and_recording_not_task_success(tmp_path, reason, ready, expected):
    manifest = finalized_bag(tmp_path)
    order = []

    class Owned:
        def __init__(self, role):
            self.role = role

        def stop(self):
            order.append(self.role)
            return {"returncode": 0, "escalated": False}

    processes = {role: Owned(role) for role in ("camera", "bag", "controller")}
    state = finalize_session(tmp_path, manifest, processes, reason, ready)
    assert order == ["controller", "bag", "camera"]
    assert state == expected
    assert json.loads((tmp_path / "session_manifest.json").read_text())["state"] == expected


def test_real_process_is_gracefully_finalized_without_ros_or_hardware(tmp_path):
    script = (
        "import signal,time,sys; "
        "signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); "
        "print('ready', flush=True); time.sleep(30)")
    log = tmp_path / "owned.log"
    owned = RecordingProcess([sys.executable, "-u", "-c", script], log)
    try:
        deadline = time.monotonic() + 3
        while log.stat().st_size == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert "ready" in log.read_text()
    finally:
        result = owned.stop(grace_s=1)
    assert result == {"returncode": 0, "escalated": False}


def test_manifest_replacement_is_complete_json(tmp_path):
    path = tmp_path / "session_manifest.json"
    write_manifest(path, {"state": "starting"})
    write_manifest(path, {"state": "complete"})
    assert json.loads(path.read_text()) == {"state": "complete"}
    assert not path.with_suffix(".json.tmp").exists()
