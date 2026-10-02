"""Opt-in isolated DDS smoke test, never opening cameras or robot devices."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from experiments.recording_session import write_manifest


@pytest.mark.skipif(os.environ.get("RUN_RECORDING_ROS_SMOKE") != "1",
                    reason="opt-in synthetic ROS test in an isolated domain")
def test_real_bag_supervision_finalizes_on_interrupt(tmp_path):
    fixture = Path(__file__).with_name("recording_ros_fixture.py")
    environment = dict(os.environ, ROS_DOMAIN_ID="117", ROS_LOCALHOST_ONLY="1",
                       ROS_LOG_DIR=str(tmp_path / "ros_logs"))
    manifest_path = tmp_path / "session_manifest.json"
    topics = ["/device/state", "/shape_tracking/markers", "/catheter_mppi/status"]
    write_manifest(manifest_path, {
        "state": "allocated", "video_enabled": False,
        "command_output_enabled": False, "startup_timeout_s": 20.0,
        "required_topics": topics,
        "commands": {
            "bag": ["ros2", "bag", "record", "-s", "sqlite3",
                    "-o", str(tmp_path / "robot_bag"), *topics],
            "camera": [sys.executable, str(fixture)],
            "controller": [sys.executable, "-c", (
                "import signal,sys,time; "
                "signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); time.sleep(60)")],
        }})
    with (tmp_path / "supervisor.log").open("w") as log:
        process = subprocess.Popen([
            sys.executable, "-m", "experiments.session_recording",
            "--manifest", str(manifest_path)], env=environment,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 25
            document = {}
            while time.monotonic() < deadline and process.poll() is None:
                document = json.loads(manifest_path.read_text())
                if document["state"] == "recording_ready":
                    break
                time.sleep(0.1)
            assert document.get("state") == "recording_ready", (
                tmp_path / "supervisor.log").read_text()
            # Exercise repeated graph-only probes while the real recorder writes.
            for _ in range(60):
                time.sleep(0.1)
                assert process.poll() is None
                document = json.loads(manifest_path.read_text())
                assert document["state"] == "recording_ready"
                assert document["readiness"]["recorder_evidence"]["probe"] == (
                    "ros_graph_and_file_existence")
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=110)
    document = json.loads(manifest_path.read_text())
    assert process.returncode == 0, (tmp_path / "supervisor.log").read_text()
    assert document["state"] == "complete"
    assert document["qualification"]["passed"]
    assert all(value["returncode"] == 0 for value in document["processes"].values())
