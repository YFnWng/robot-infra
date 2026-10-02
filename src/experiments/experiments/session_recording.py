"""Supervise recording processes without owning any robot command interface."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from .recording_session import (
    recorder_readiness, qualify_recording, utc_now, write_manifest)


def recorder_subscriptions(node, required_topics):
    """Observe only the owned default recorder, not arbitrary subscribers."""
    return [topic for topic in required_topics if any(
        endpoint.node_name == "rosbag2_recorder" and endpoint.node_namespace == "/"
        for endpoint in node.get_subscriptions_info_by_topic(topic))]


class RecordingProcess:
    """One owned process group, with a bounded graceful shutdown."""

    def __init__(self, command: list[str], log_path: Path):
        self.log = log_path.open("xb")
        try:
            self.process = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=self.log,
                stderr=subprocess.STDOUT, start_new_session=True)
        except BaseException:
            self.log.close()
            raise

    def stop(self, grace_s: float = 30.0) -> dict:
        escalated = False
        # Signal surviving descendants even if the group leader has exited.
        for signum, timeout in ((signal.SIGINT, grace_s),
                                (signal.SIGTERM, 3.0),
                                (signal.SIGKILL, 3.0)):
            try:
                os.killpg(self.process.pid, signum)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                escalated = True
        self.log.close()
        return {"returncode": self.process.poll(), "escalated": escalated}


def finalize_session(session, manifest, processes, reason, ready_once):
    """Stop controller first; retain camera capture until the bag is finalized."""
    manifest.update(state="stopping", stop_reason=reason, stopped_at=utc_now())
    write_manifest(session / "session_manifest.json", manifest)
    outcomes = {}
    for role in ("controller", "bag", "camera"):
        if role in processes:
            outcomes[role] = processes[role].stop()
    evidence = qualify_recording(session, manifest)
    abnormal = any(result["escalated"] or result["returncode"] not in (0, -2)
                   for result in outcomes.values())
    state = "complete" if (reason == "operator_stop" and ready_once
                           and evidence["passed"] and not abnormal) else "partial"
    if reason != "operator_stop" or abnormal:
        state = "failed"
    manifest.update(state=state, finalized_at=utc_now(), processes=outcomes,
                    qualification=evidence, recording_ready_seen=ready_once)
    write_manifest(session / "session_manifest.json", manifest)
    return state


def main(argv=None):
    # ROS imports stay out of filesystem/process helpers and offline tests.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import remove_ros_args
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import PointCloud
    from control_interface.msg import DeviceStream
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args(remove_ros_args(args=argv)[1:])
    path = args.manifest.resolve()
    session = path.parent
    manifest = json.loads(path.read_text())
    if manifest["state"] != "allocated":
        raise ValueError("recording session cannot be resumed or overwritten")
    processes = {}
    stopped = False
    ready_once = False
    reason = "startup_failed"

    def stop_requested(_signum, _frame):
        nonlocal stopped
        stopped = True

    old_handlers = {sig: signal.signal(sig, stop_requested)
                    for sig in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = Node("research_session_recording")
    received = {}
    camera_active = not manifest["video_enabled"]
    recorder_active = False
    recorder_evidence = {}
    next_query = 0.0
    next_manifest = 0.0
    status = node.create_publisher(String, "/experiments/session_status", 10)

    def marker_status(message):
        nonlocal camera_active
        if not manifest["video_enabled"]:
            return
        for diagnostic in message.status:
            values = {item.key: item.value for item in diagnostic.values}
            camera_active = (
                values.get("recording_active") == "true"
                and values.get("recording_session_dir") == manifest["video_directory"])
            received["camera"] = time.monotonic()

    def device(message):
        if message.predicate in (DeviceStream.POS, DeviceStream.ENC):
            received[str(message.predicate)] = time.monotonic()

    node.create_subscription(DiagnosticArray, "/shape_tracking/marker_status",
                             marker_status, 10)
    node.create_subscription(PointCloud, "/shape_tracking/markers",
                             lambda _: received.update(markers=time.monotonic()),
                             qos_profile_sensor_data)
    node.create_subscription(DeviceStream, "/device/state", device,
                             qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, "/catheter_mppi/status",
                             lambda _: received.update(controller=time.monotonic()), 10)
    started = time.monotonic()
    try:
        # Never compete with an already-running camera/controller/recorder.
        discovery_end = started + 1.0
        while time.monotonic() < discovery_end and not stopped:
            rclpy.spin_once(node, timeout_sec=0.1)
        conflicts = {name for name, _ in node.get_node_names_and_namespaces()} & {
            "marker_tracking", "catheter_mppi", "rosbag2_recorder"}
        if conflicts:
            raise RuntimeError(f"stop existing session processes first: {sorted(conflicts)}")
        if not stopped:
            for role in ("bag", "camera", "controller"):
                processes[role] = RecordingProcess(
                    manifest["commands"][role], session / f"{role}.log")
        manifest["state"] = "starting"
        write_manifest(path, manifest)
        print(f"[research session] {session}", flush=True)
        while not stopped:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            for role, owned in processes.items():
                if owned.process.poll() is not None:
                    raise RuntimeError(f"{role} exited unexpectedly; see {role}.log")
            if now >= next_query:
                recorder_evidence = recorder_readiness(
                    session / "robot_bag", manifest["required_topics"],
                    recorder_subscriptions(node, manifest["required_topics"]))
                recorder_active = recorder_evidence["ready"]
                next_query = now + 1.0
            camera_fresh = not manifest["video_enabled"] or (
                now - received.get("camera", -float("inf")) < 2.0)
            streams = ("markers", "controller", str(DeviceStream.POS),
                       str(DeviceStream.ENC))
            fresh = all(now - received.get(key, -float("inf")) < 2.0
                        for key in streams)
            ready = camera_active and camera_fresh and recorder_active and fresh
            state = "recording_ready" if ready else "recording_not_ready"
            manifest["readiness"] = {
                "camera_active": camera_active and camera_fresh,
                "recorder_active": recorder_active,
                "telemetry_fresh": fresh, "observed_streams": sorted(received),
                "recorder_evidence": recorder_evidence,
            }
            if ready_once and not (camera_active and camera_fresh
                                   and recorder_active):
                raise RuntimeError(
                    f"required recording readiness lost: {manifest['readiness']}")
            if manifest["state"] != state:
                manifest["state"] = state
                if ready:
                    ready_once = True
                    manifest.setdefault("ready_at", utc_now())
                    if manifest["video_enabled"]:
                        link = Path(manifest["video_directory"]) / "robot_bag"
                        if not link.is_symlink():
                            link.symlink_to(session / "robot_bag", target_is_directory=True)
                write_manifest(path, manifest)
                print(f"[research session] {state} (recording only; no task started)",
                      flush=True)
            if now >= next_manifest:
                write_manifest(path, manifest)
                next_manifest = now + 1.0
            status.publish(String(data=json.dumps({
                "session_id": session.name, "state": state,
                "command_output_enabled": manifest["command_output_enabled"]})))
            if not ready_once and now - started > manifest["startup_timeout_s"]:
                raise RuntimeError(f"recording readiness timed out: {manifest['readiness']}")
        reason = "operator_stop"
    except Exception as exc:
        reason = f"{type(exc).__name__}: {exc}"
        print(f"[research session] {reason}", flush=True)
    finally:
        try:
            state = finalize_session(session, manifest, processes, reason, ready_once)
            print(f"[research session] {state}; see session_manifest.json", flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            for sig, previous in old_handlers.items():
                signal.signal(sig, previous)
    return 0 if state == "complete" else 2


if __name__ == "__main__":
    raise SystemExit(main())
