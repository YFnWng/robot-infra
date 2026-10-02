"""Research task binding and censored outcome tests; no ROS or devices."""
import json

import pytest

from experiments.reaching_analysis import trial_metrics
from experiments.reaching_session import prepare_task
from experiments import reaching_analysis


def test_preparation_preserves_frozen_targets_and_refuses_reuse(tmp_path):
    (tmp_path / "session_manifest.json").write_text(json.dumps({
        "state": "recording_ready", "command_output_enabled": True}))
    source = tmp_path / "source.yaml"
    source.write_text("generator:\n  type: absolute_targets_m\n  targets_m: [[0, 0, 0.1]]\n")
    command = prepare_task(tmp_path, source)
    assert command[:4] == ["ros2", "run", "control_tasks", "catheter_sparse_point_experiment"]
    assert (tmp_path / "task_targets.yaml").read_bytes() == source.read_bytes()
    assert json.loads((tmp_path / "task_manifest.json").read_text())["targets_m"] == [[0, 0, 0.1]]
    with pytest.raises(FileExistsError):
        prepare_task(tmp_path, source)


@pytest.mark.parametrize("state,enabled", [("starting", True), ("recording_ready", False)])
def test_recording_gate_fails_closed(tmp_path, state, enabled):
    (tmp_path / "session_manifest.json").write_text(json.dumps({
        "state": state, "command_output_enabled": enabled}))
    with pytest.raises(ValueError):
        prepare_task(tmp_path, tmp_path / "missing.yaml")


def test_metrics_keep_failed_and_unattempted_trials():
    records = [
        {"trial": 1, "event": "trial_started", "monotonic_ns": 0},
        {"trial": 1, "event": "trial_completed", "monotonic_ns": 2_000_000_000,
         "reached": 1, "timed_out": 0, "measured_tip_m": [0, 0, .101]},
        {"trial": 2, "event": "trial_started", "monotonic_ns": 3_000_000_000},
    ]
    rows = trial_metrics(records, [[0, 0, .1]] * 3)
    assert rows[0]["measured_final_error_mm"] == pytest.approx(1)
    assert rows[0]["time_to_result_s"] == 2
    assert rows[1]["outcome"] == "interrupted_or_faulted"
    assert rows[2]["outcome"] == "not_attempted"
    assert rows[0]["command_reversals"] is None


def test_duplicate_boundary_is_not_silently_averaged():
    with pytest.raises(ValueError):
        trial_metrics([{"event": "trial_started", "trial": 1}] * 2, [[0, 0, 0]])


def test_offline_report_retains_home_failure(tmp_path, monkeypatch):
    (tmp_path / "session_manifest.json").write_text(json.dumps({
        "state": "recording_ready", "command_output_enabled": True}))
    source = tmp_path / "source.yaml"
    source.write_text("generator:\n  type: absolute_targets_m\n  targets_m: [[0, 0, 0.1]]\n")
    prepare_task(tmp_path, source)
    (tmp_path / "session_manifest.json").write_text(json.dumps({"state": "partial"}))
    (tmp_path / "trials.jsonl").write_text(json.dumps({
        "event": "task_failed", "reason": "home timeout"}) + "\n")
    monkeypatch.setattr(reaching_analysis, "qualify_recording",
                        lambda *_: {"passed": False, "errors": ["missing bag"]})
    output = reaching_analysis.analyze(tmp_path)
    assert "not_attempted" in (output / "report.md").read_text()
    integrity = json.loads((output / "integrity.json").read_text())
    assert integrity["task_failures"][0]["reason"] == "home timeout"
    assert not integrity["recording"]["passed"]
