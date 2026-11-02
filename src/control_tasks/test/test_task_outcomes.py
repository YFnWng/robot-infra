"""Offline failure evidence regressions; no ROS initialization or commands."""
import json
import threading
from types import SimpleNamespace

import pytest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from control_tasks.task_outcomes import TaskFailure, controller_failure
from control_tasks.trajectory_action import TipTrajectoryActionServer
from control_tasks.path_action import TipPathActionServer


def status(state, reason, snapshot="none", armed=False):
    return DiagnosticArray(status=[DiagnosticStatus(
        name="catheter_control/mppi", message=state, values=[
            KeyValue(key=k, value=v) for k, v in {
                "armed": str(armed), "reason": reason,
                "fault_snapshot": snapshot}.items()])])


@pytest.mark.parametrize("server", [TipTrajectoryActionServer, TipPathActionServer])
def test_fault_survives_cleanup_status_and_is_copied_to_result(server):
    reason = "takeup_confirmation_timeout"
    snapshot = json.dumps(dict(reason=reason, encoder_age_ms=12.))
    node = SimpleNamespace(_lock=threading.Lock(), controller_failure=("", ""))
    server._status_cb(node, status("FAULTED", reason, snapshot))
    server._status_cb(node, status("DISARMED", "disarmed_by_user"))
    if server is TipTrajectoryActionServer:
        sequence = SimpleNamespace(complete=False, reached_waypoints=0, timed_out_waypoints=0)
        result = server._result(node, sequence, "controller faulted", controller_fault=True)
    else:
        result = server._result(node, False, "controller FAULTED", [], controller_fault=True)
    assert not result.success
    assert result.fault_reason == reason
    assert json.loads(result.fault_snapshot_json)["encoder_age_ms"] == 12.
    assert reason in result.message
    server._status_cb(node, status("READY", "ready", armed=True))
    assert node.controller_failure == ("", "")
    # The already-returned result is independent of later lifecycle cleanup.
    assert result.fault_reason == reason


@pytest.mark.parametrize("raw", ["none", "bad JSON", "[]", '{"reason":"old_fault"}',
                                 '{"reason":"current","age":NaN}'])
def test_missing_or_mismatched_evidence_does_not_break_cleanup(raw):
    assert controller_failure(dict(reason="current", fault_snapshot=raw)) == ("current", "")


def test_noncontroller_failure_does_not_inherit_old_fault():
    node = SimpleNamespace(_lock=threading.Lock(), controller_failure=("old_fault", "{}"))
    result = TipPathActionServer._result(node, False, "timeout: endpoint not settled", [])
    assert result.fault_reason == result.fault_snapshot_json == ""
    assert not result.success


def test_expected_failure_preserves_serializable_evidence():
    failure = TaskFailure("target failed", fault_reason="marker_stale", fault_snapshot_json="{}")
    assert json.loads(json.dumps(failure.fields())) == {
        "fault_reason": "marker_stale", "fault_snapshot_json": "{}"}


def test_sparse_client_records_failure_and_returns_nonzero_after_cleanup(tmp_path, monkeypatch):
    from control_tasks import sparse_point_experiment as client
    import numpy as np
    events = []
    monkeypatch.setattr(client.rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(client.rclpy, "ok", lambda: True)
    monkeypatch.setattr(client.rclpy, "shutdown", lambda: events.append("shutdown"))
    spec = SimpleNamespace(generator={}, home_position=np.zeros(6))
    monkeypatch.setattr(client, "load_sparse_point_experiment", lambda _: spec)
    def fail(*args):
        raise TaskFailure("controller faulted: takeup_confirmation_timeout",
                          fault_reason="takeup_confirmation_timeout",
                          fault_snapshot_json='{"reason":"takeup_confirmation_timeout"}')
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)),
        get_logger=lambda: SimpleNamespace(error=lambda _: events.append("logged")),
        wait_for_inputs=lambda _: None, home=lambda _: None, tip=np.zeros(3),
        generate_targets=lambda _: [np.zeros(3)], run_target=fail,
        cancel_active=lambda: events.append("cancel"),
        _publish_mode=lambda _: events.append("release"),
        destroy_node=lambda: events.append("destroy"))
    node.get_logger = lambda: SimpleNamespace(info=lambda _: None,
                                             error=lambda _: events.append("logged"))
    monkeypatch.setattr(client, "SparsePointExperiment", lambda _: node)
    journal = tmp_path / "trials.jsonl"
    assert client.main(["task", "protocol.yaml", "--trial-records", str(journal)]) == 2
    rows = [json.loads(line) for line in journal.read_text().splitlines()]
    assert [row["event"] for row in rows][-2:] == ["trial_failed", "task_failed"]
    assert rows[-1]["fault_reason"] == "takeup_confirmation_timeout"
    assert events == ["cancel", "release", "logged", "destroy", "shutdown"]


def test_path_client_records_expected_failure_and_cleans_up(tmp_path, monkeypatch):
    from control_tasks import path_file as client
    import numpy as np
    events = []
    source = tmp_path / "path.yaml"
    source.write_text("path_knots_m: [[0, 0, 0], [0, 0, 0.01]]\n")
    output = tmp_path / "result.json"
    result = TipPathActionServer._result(
        SimpleNamespace(_lock=threading.Lock(), controller_failure=("marker_stale", "{}")),
        False, "controller FAULTED", [], controller_fault=True)
    class Future:
        def __init__(self, value):
            self.value = value
        def result(self):
            return self.value
    handle = SimpleNamespace(accepted=True,
        get_result_async=lambda: Future(SimpleNamespace(result=result)),
        cancel_goal_async=lambda: (events.append("cancel") or Future(None)))
    node = SimpleNamespace(tip=np.zeros(3), tip_received_at=client.time.monotonic(),
        client=SimpleNamespace(wait_for_server=lambda **_: True,
                               send_goal_async=lambda *_, **kwargs: Future(handle)),
        feedback_cb=lambda _: None, feedback_records=[],
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)),
        get_logger=lambda: SimpleNamespace(info=lambda _: None,
                                           error=lambda _: events.append("logged")),
        destroy_node=lambda: events.append("destroy"))
    monkeypatch.setattr(client, "PathFileClient", lambda *_: node)
    monkeypatch.setattr(client, "_goal", lambda *_: object())
    monkeypatch.setattr(client.rclpy, "init", lambda **_: None)
    monkeypatch.setattr(client.rclpy, "ok", lambda: True)
    monkeypatch.setattr(client.rclpy, "shutdown", lambda: events.append("shutdown"))
    monkeypatch.setattr(client.rclpy, "spin_until_future_complete", lambda *_, **kwargs: None)
    assert client.main(["path", str(source), "--result-file", str(output)]) == 2
    saved = json.loads(output.read_text())
    assert saved["fault_reason"] == "marker_stale"
    assert not saved["success"]
    assert saved["final_error_mm"] is None
    assert events == ["cancel", "logged", "destroy", "shutdown"]
