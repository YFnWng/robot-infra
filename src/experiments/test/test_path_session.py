import json
from pathlib import Path
import subprocess

import pytest
import yaml

from experiments.path_analysis import analyze, path_metrics
from experiments.path_session import execute_task, prepare_task, validate_acquisition


@pytest.fixture(autouse=True)
def home_template(monkeypatch):
    from experiments import path_session
    source = (Path(__file__).resolve().parents[2] / "catheter_control" / "config"
              / "experiments" / "sparse_points" / "farther_tendon_12_plain.yaml")
    monkeypatch.setattr(path_session, "_home_template", lambda: source)


def fixture(tmp_path, rotation=0):
    (tmp_path / "session_manifest.json").write_text(json.dumps({
        "state": "recording_ready", "command_output_enabled": True}))
    (tmp_path / "controller_manifest.json").write_text(json.dumps({
        "controller_parameters": {"controller_velocity_max": [10, rotation, 4.5, 4, 25, 25]}}))
    source = tmp_path / "frozen.yaml"
    source.write_text(yaml.safe_dump({
        "path_knots_m": [[0, 0, .06], [.004, .004, .065], [0, 0, .06]],
        "auto_arm": True}))
    return source


def test_preparation_preserves_path_and_refuses_reuse(tmp_path):
    source = fixture(tmp_path)
    command = prepare_task(tmp_path, source)
    assert command[:4] == ["ros2", "run", "control_tasks", "catheter_tip_path_file"]
    assert (tmp_path / "task_path.yaml").read_bytes() == source.read_bytes()
    from control_tasks.sparse_point_experiment import load_sparse_point_experiment
    home = load_sparse_point_experiment(tmp_path / "task_home.yaml")
    assert home.home_position.tolist() == [20, 0, 0, 0, 0, 0]
    assert home.decoupled_tendon_prehome
    assert home.rotation_guard_enabled
    assert home.generator == {"type": "absolute_targets_m", "targets_m": [[0, 0, .06]]}
    assert "--preserve-knots" in command
    assert set(home.required_controller_status) == {
        "command_output_enabled", "controller_velocity_max"}
    with pytest.raises(FileExistsError):
        prepare_task(tmp_path, source)


def test_execution_homes_before_submitting_path(tmp_path, monkeypatch):
    from experiments import path_session
    events = []
    command = prepare_task(tmp_path, fixture(tmp_path))
    def run(args, **kwargs):
        events.append((args, kwargs))
        write_acquisition(tmp_path / "acquisition_trials.jsonl")
    monkeypatch.setattr(path_session.subprocess, "run", run)
    monkeypatch.setattr(path_session.os, "execvp",
                        lambda *args: events.append(args))
    execute_task(tmp_path, command)
    assert events[0][0][3] == "catheter_sparse_point_experiment"
    assert "--home-only" not in events[0][0]
    assert events[0][1] == {"check": True}
    assert events[1] == ("ros2", command)


def test_synchronized_path_reuses_acquisition_with_explicit_status_gate(tmp_path):
    source = fixture(tmp_path, rotation=25)
    identity = tmp_path / "controller_manifest.json"
    data = json.loads(identity.read_text())
    data["controller_parameters"]["synchronized_sheath_rotation"] = True
    identity.write_text(json.dumps(data))
    prepare_task(tmp_path, source)
    home = yaml.safe_load((tmp_path / "task_home.yaml").read_text())
    assert home["required_controller_status"]["synchronized_sheath_rotation"] is True
    assert home["rotation_guard"]["enabled"] is False
    assert json.loads((tmp_path / "task_manifest.json").read_text())["task"] == "synchronized_rotation_continuous_path"


def write_acquisition(journal, *, reached=1, final_event="task_completed", target=None):
    rows = [{"event": "trial_started", "target_m": target or [0, 0, .06]},
            {"event": "trial_completed", "reached": reached, "timed_out": 1-reached},
            {"event": final_event, "reached": reached, "timed_out": 1-reached}]
    journal.write_text("\n".join(json.dumps(row) for row in rows))


@pytest.mark.parametrize("changes", [
    {"reached": 0}, {"final_event": "task_interrupted"},
    {"target": [0, 0, .07]},
])
def test_unsuccessful_acquisition_never_starts_tracking(tmp_path, monkeypatch, changes):
    from experiments import path_session
    command = prepare_task(tmp_path, fixture(tmp_path))
    monkeypatch.setattr(path_session.subprocess, "run", lambda *_args, **_kwargs:
                        write_acquisition(tmp_path / "acquisition_trials.jsonl", **changes))
    monkeypatch.setattr(path_session.os, "execvp",
                        lambda *_: pytest.fail("tracking started without acquisition"))
    with pytest.raises(RuntimeError, match="acquisition incomplete"):
        execute_task(tmp_path, command)


def test_missing_and_incomplete_acquisition_evidence_fails(tmp_path):
    journal = tmp_path / "acquisition.jsonl"
    with pytest.raises(FileNotFoundError):
        validate_acquisition(journal, [0, 0, .06])
    journal.write_text("")
    with pytest.raises(RuntimeError):
        validate_acquisition(journal, [0, 0, .06])


@pytest.mark.parametrize("failure", [subprocess.CalledProcessError(2, "home"),
                                     KeyboardInterrupt()])
def test_failed_or_interrupted_home_never_submits_path(tmp_path, monkeypatch, failure):
    from experiments import path_session
    command = prepare_task(tmp_path, fixture(tmp_path))
    def fail(*args, **kwargs):
        raise failure
    monkeypatch.setattr(path_session.subprocess, "run", fail)
    monkeypatch.setattr(path_session.os, "execvp",
                        lambda *_: pytest.fail("path submitted after failed home"))
    with pytest.raises(type(failure)):
        execute_task(tmp_path, command)


@pytest.mark.parametrize("failure", [subprocess.CalledProcessError(2, "home"),
                                     RuntimeError("unexpected bug")])
def test_cli_reports_child_failure_but_does_not_hide_unexpected_errors(tmp_path, monkeypatch, capsys, failure):
    import sys
    from experiments import path_session
    monkeypatch.setattr(sys, "argv", ["path_session", "--session", str(tmp_path),
                                     "--path", "path.yaml", "--execute"])
    monkeypatch.setattr(path_session, "prepare_task", lambda *_: ["not-executed"])
    def fail(*_):
        raise failure
    monkeypatch.setattr(path_session, "execute_task", fail)
    if isinstance(failure, subprocess.CalledProcessError):
        with pytest.raises(SystemExit) as error:
            path_session.main()
        assert error.value.code == 2
        assert "tracking not started" in capsys.readouterr().out
    else:
        with pytest.raises(RuntimeError, match="unexpected bug"):
            path_session.main()


def test_twoaxis_preparation_rejects_enabled_rotation(tmp_path):
    with pytest.raises(ValueError, match="rotation disabled"):
        prepare_task(tmp_path, fixture(tmp_path, rotation=25))
    assert not (tmp_path / "task_path.yaml").exists()


def test_preparation_rejects_unfrozen_generator(tmp_path):
    source = fixture(tmp_path)
    source.write_text("generator: {type: circle_plane_from_current_tip}")
    with pytest.raises(ValueError, match="freeze"):
        prepare_task(tmp_path, source)


def test_path_metrics_preserves_missing_results():
    metrics = path_metrics({"success": False, "message": "faulted"})
    assert metrics["final_error_mm"] is None
    assert metrics["last_progress"] is None
    assert not metrics["success"]


def test_legacy_reference_length_is_not_relabelled_as_actual_coverage():
    metrics = path_metrics({"completed_arc_length_m": .025})
    assert metrics["reference_arc_length_m"] == .025
    assert metrics["completed_arc_length_m"] is None
    assert metrics["path_coverage"] is None


def test_new_metrics_separate_reference_coverage_and_tracking_error():
    metrics = path_metrics({"completed_arc_length_m": .004,
                            "reference_arc_length_m": .025,
                            "path_coverage": .16, "traversal_complete": False,
                            "rms_closest_path_error_mm": .2})
    assert metrics["completed_arc_length_m"] == .004
    assert metrics["reference_arc_length_m"] == .025
    assert metrics["path_coverage"] == .16
    assert not metrics["traversal_complete"]
    assert metrics["rms_closest_path_error_mm"] == .2


def test_state_time_excludes_missing_feedback_spans():
    result = {"feedback": [
        {"elapsed_s": 0, "progress": 0, "governor_state": "TRANSMISSION_HOLD"},
        {"elapsed_s": .1, "progress": 0, "governor_state": "ADVANCE"},
        {"elapsed_s": 1, "progress": .2, "governor_state": "ADVANCE"}]}
    metrics = path_metrics(result)
    assert metrics["governor_state_seconds"] == {"TRANSMISSION_HOLD": .1}


def test_report_retains_incomplete_result(tmp_path, monkeypatch):
    from experiments import path_analysis
    prepare_task(tmp_path, fixture(tmp_path))
    (tmp_path / "session_manifest.json").write_text('{"state": "partial"}')
    (tmp_path / "path_result.json").write_text("{")
    monkeypatch.setattr(path_analysis, "qualify_recording",
                        lambda *_: {"passed": False, "errors": ["missing bag"]})
    output = analyze(tmp_path)
    assert "missing/incomplete task result" in (output / "report.md").read_text()
    metrics = json.loads((output / "path_metrics.json").read_text())
    assert metrics["rms_closest_path_error_mm"] is None
    assert not json.loads((output / "acquisition_result.json").read_text())["passed"]


def test_success_report_generates_feedback_and_plot(tmp_path, monkeypatch):
    from experiments import path_analysis
    prepare_task(tmp_path, fixture(tmp_path))
    write_acquisition(tmp_path / "acquisition_trials.jsonl")
    (tmp_path / "session_manifest.json").write_text('{"state": "complete"}')
    result = {"success": True, "message": "path complete",
              "started_monotonic_ns": 0, "ended_monotonic_ns": 100_000_000,
              "final_error_mm": .5, "rms_closest_path_error_mm": .4,
              "p95_closest_path_error_mm": .6, "completed_arc_length_m": .025,
              "feedback": [{"elapsed_s": 0, "progress": 0,
                            "reference_error_mm": .2, "closest_path_error_mm": .1,
                            "governor_state": "ADVANCE"},
                           {"elapsed_s": .1, "progress": 1,
                            "reference_error_mm": .5, "closest_path_error_mm": .4,
                            "governor_state": "FINAL_HOLD"}]}
    (tmp_path / "path_result.json").write_text(json.dumps(result))
    monkeypatch.setattr(path_analysis, "qualify_recording",
                        lambda *_: {"passed": True, "errors": []})
    output = analyze(tmp_path)
    assert (output / "path_tracking.png").stat().st_size > 0
    assert "reference_error_mm" in (output / "path_feedback.csv").read_text()
    assert json.loads((output / "acquisition_result.json").read_text())["passed"]
