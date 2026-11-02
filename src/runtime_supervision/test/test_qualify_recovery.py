"""Reporting harness tests: subprocesses are inert, never launch hardware."""
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from runtime_supervision import qualify_recovery as runner


@pytest.fixture
def harness(tmp_path, monkeypatch):
    repo = tmp_path / "repo"
    test = repo / "src" / "test_owner.py"
    test.parent.mkdir(parents=True)
    test.write_text("def test_owner(): pass\n")
    monkeypatch.setattr(runner, "STAGES", {"first": ("test_owner.py",), "second": ("test_owner.py",)})
    monkeypatch.setattr(runner, "installed_identity", lambda _: {"owner": "matched"})
    return repo, tmp_path / "out"


def fake_run(monkeypatch, xml, code=0):
    def run(command, **kwargs):
        path = Path(next(arg.split("=", 1)[1] for arg in command if arg.startswith("--junitxml=")))
        path.write_text(xml)
        assert kwargs["timeout"] == 120.
        assert kwargs["env"]["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] == "1"
        return subprocess.CompletedProcess(command, code)
    monkeypatch.setattr(runner.subprocess, "run", run)


def test_suite_collects_results_and_exact_test_identity(harness, monkeypatch):
    repo, output = harness
    fake_run(monkeypatch, '<testsuites><testsuite><testcase name="ok"/></testsuite></testsuites>')
    report = runner.qualify(repo, output)
    assert report["passed"] and report["executed_commands"] == 0
    assert not report["live_timing_qualified"]
    assert len(report["stages"]) == 2
    assert all(stage["counts"]["passed"] == 1 and stage["test_sha256"] for stage in report["stages"])
    assert json.loads((output / "suite.json").read_text()) == report
    with pytest.raises(FileExistsError):
        runner.qualify(repo, output)


@pytest.mark.parametrize("xml,code", [('<testsuites/>', 0),
    ('<testsuite><testcase><skipped/></testcase></testsuite>', 0),
    ('<testsuite><testcase><failure/></testcase></testsuite>', 0),
    ('<testsuite><testcase><error/></testcase></testsuite>', 1),
    ('<testsuite><testcase/></testsuite>', 1), ('broken xml', 0)])
def test_empty_skipped_invalid_and_failed_runs_cannot_pass(harness, monkeypatch, xml, code):
    repo, output = harness
    fake_run(monkeypatch, xml, code)
    report = runner.qualify(repo, output)
    assert not report["passed"] and report["failures"] == ["first", "second"]


def test_timeout_is_reported_and_later_stage_is_attempted(harness, monkeypatch):
    repo, output = harness
    def timeout(command, **kwargs):
        raise subprocess.TimeoutExpired(command, kwargs["timeout"])
    monkeypatch.setattr(runner.subprocess, "run", timeout)
    report = runner.qualify(repo, output)
    assert not report["passed"] and len(report["stages"]) == 2
    assert all("TimeoutExpired" in stage["error"] for stage in report["stages"])


def test_identity_error_produces_failed_report_without_running_tests(harness, monkeypatch):
    repo, output = harness
    def mismatch(_):
        raise RuntimeError("installed/source mismatch")
    monkeypatch.setattr(runner, "installed_identity", mismatch)
    report = runner.qualify(repo, output)
    assert not report["passed"] and not report["stages"]
    assert "mismatch" in report["failures"][0]


@pytest.mark.parametrize("timeout", [0., -1., float("nan"), float("inf"), 601.])
def test_invalid_budget_is_rejected_before_artifact_creation(harness, timeout):
    repo, output = harness
    with pytest.raises(ValueError):
        runner.qualify(repo, output, timeout)
    assert not output.exists()


def test_identity_rejects_source_injection(tmp_path, monkeypatch):
    monkeypatch.setattr(runner.importlib.util, "find_spec", lambda _: SimpleNamespace(origin=str(tmp_path / "source.py")))
    with pytest.raises(RuntimeError, match="outside rebuilt overlay"):
        runner.installed_identity(tmp_path)


def test_explicit_wheel_root_is_recorded_without_installing(harness, monkeypatch, tmp_path):
    repo, output = harness
    packages = tmp_path / "packages"
    model = packages / "cr_meta_lnn/deployment/experimental/joint_engagement_filter.py"
    model.parent.mkdir(parents=True)
    model.write_text("# isolated fixture\n")
    (packages / "cr_meta_lnn-1.0.0.dist-info").mkdir()
    fake_run(monkeypatch, '<testsuite><testcase/></testsuite>')
    report = runner.qualify(repo, output, model_package_root=packages)
    assert report["passed"] and report["model_package"]["root"] == str(packages)
    assert report["model_package"]["sha256"]


def test_source_tree_without_wheel_metadata_is_rejected(harness):
    repo, output = harness
    report = runner.qualify(repo, output, model_package_root=repo)
    assert not report["passed"] and not report["stages"]
