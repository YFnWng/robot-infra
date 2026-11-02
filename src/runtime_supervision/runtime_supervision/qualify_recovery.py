"""Bounded offline owner integration qualification; no ROS graph or devices.

Reuse maintained fault-injection tests rather than copying controller policy.
Loaded production owners must match the rebuilt overlay before tests run.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


OWNERS = {
    "catheter_control.node": "catheter_control/catheter_control/node.py",
    "catheter_control.safety.lifecycle": "catheter_control/catheter_control/safety/lifecycle.py",
    "catheter_control.planning.path_tracking": "catheter_control/catheter_control/planning/path_tracking.py",
    "catheter_control.planning.trajectory": "catheter_control/catheter_control/planning/trajectory.py",
    "catheter_control.transmission.backlash": "catheter_control/catheter_control/transmission/backlash.py",
    "catheter_control.orchestration.task_gc": "catheter_control/catheter_control/orchestration/task_gc.py",
    "control_tasks.path_action": "control_tasks/control_tasks/path_action.py",
    "control_tasks.trajectory_action": "control_tasks/control_tasks/trajectory_action.py",
    "runtime_supervision.qualify_recovery": "runtime_supervision/runtime_supervision/qualify_recovery.py",
}

STAGES = {
    "lifecycle": ("catheter_control/test/test_phase4_lifecycle.py",),
    "callback_recovery": ("catheter_control/test/test_path_plan_commit.py",),
    "task_completion": (
        "catheter_control/test/test_path_tracking.py",
        "catheter_control/test/test_trajectory.py",
        "control_tasks/test/test_path_completion.py",
        "control_tasks/test/test_task_outcomes.py"),
    "transmission": (
        "catheter_control/test/test_backlash.py",
        "catheter_control/test/test_tendon_confirmation_persistence.py"),
    "gc_and_configuration": (
        "catheter_control/test/test_task_gc.py",
        "catheter_control/test/test_configuration.py",
        "catheter_control/test/test_parameters.py"),
}


def installed_identity(repo: Path) -> dict:
    """Require overlay loading and exact source bytes, not a source injection."""
    records = {}
    for name, relative in OWNERS.items():
        spec = importlib.util.find_spec(name)
        if spec is None or spec.origin is None:
            raise RuntimeError(f"missing installed owner: {name}")
        loaded = Path(spec.origin).absolute()
        source = repo / "src" / relative
        if not any(loaded.is_relative_to(repo / folder) for folder in ("build", "install")):
            raise RuntimeError(f"owner outside rebuilt overlay: {name}: {loaded}")
        data = loaded.read_bytes()
        if data != source.read_bytes():
            raise RuntimeError(f"installed/source mismatch: {name}")
        records[name] = dict(loaded_path=str(loaded), source_path=str(source),
                             sha256=hashlib.sha256(data).hexdigest())
    return records


def junit_counts(path: Path) -> dict:
    cases = list(ET.parse(path).getroot().iter("testcase"))
    counts = dict(tests=len(cases), failures=0, errors=0, skipped=0, passed=0)
    for case in cases:
        outcome = next((name for name in ("error", "failure", "skipped")
                        if case.find(name) is not None), None)
        key = {"error": "errors", "failure": "failures", "skipped": "skipped"}.get(outcome, "passed")
        counts[key] += 1
    return counts


def qualify(repo: Path, output: Path, timeout_s: float = 120.,
            model_package_root: Path | None = None) -> dict:
    repo = repo.resolve()
    output = output.absolute()
    if not math.isfinite(timeout_s) or not 0 < timeout_s <= 600:
        raise ValueError("stage timeout must be finite, positive and at most 600 s")
    # Fresh artifacts only. Never overwrite another session or a prior verdict.
    output.mkdir(parents=True, exist_ok=False)
    report = dict(schema_version=1, passed=False, scope="offline inert-owner integration",
                  executed_commands=0, ros_nodes_constructed=False,
                  live_timing_qualified=False, model_accuracy_qualified=False,
                  interpreter=sys.executable, python_prefix=sys.prefix,
                  stage_timeout_s=timeout_s, stages=[], failures=[])
    started = time.monotonic()
    try:
        report["identity"] = installed_identity(repo)
        environment = os.environ.copy()
        environment["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
        if model_package_root is not None:
            packages = model_package_root.resolve()
            model = packages / "cr_meta_lnn" / "deployment" / "experimental" / "joint_engagement_filter.py"
            if not model.is_file() or not list(packages.glob("cr_meta_lnn-*.dist-info")):
                raise RuntimeError("model package root requires an isolated installed cr_meta_lnn wheel")
            report["model_package"] = dict(root=str(packages), confirmation_owner=str(model),
                sha256=hashlib.sha256(model.read_bytes()).hexdigest())
            environment["PYTHONPATH"] = str(packages) + os.pathsep + environment.get("PYTHONPATH", "")
        for name, files in STAGES.items():
            paths = [repo / "src" / path for path in files]
            if any(not path.is_file() for path in paths):
                raise RuntimeError(f"missing maintained test in stage {name}")
            test_hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                           for path in paths}
            xml = output / f"{name}.xml"
            command = [sys.executable, "-m", "pytest", "-q",
                       f"--rootdir={output}", f"--junitxml={xml}",
                       *map(str, paths)]
            stage = dict(name=name, passed=False, command=command, test_sha256=test_hashes)
            stage_started = time.monotonic()
            try:
                with (output / f"{name}.log").open("w") as log:
                    result = subprocess.run(command, cwd=output, env=environment,
                                            stdout=log, stderr=subprocess.STDOUT,
                                            timeout=timeout_s, check=False)
                stage["exit_code"] = result.returncode
                stage["counts"] = junit_counts(xml)
                counts = stage["counts"]
                stage["passed"] = bool(result.returncode == 0 and counts["passed"] > 0
                                       and not counts["errors"] and not counts["failures"]
                                       and not counts["skipped"])
                if any(hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest
                       for path, digest in test_hashes.items()):
                    stage["passed"] = False
                    stage["error"] = "test identity changed during stage"
            except (subprocess.TimeoutExpired, OSError, ET.ParseError) as error:
                stage["error"] = f"{type(error).__name__}: {error}"
            stage["elapsed_s"] = time.monotonic() - stage_started
            report["stages"].append(stage)
            if not stage["passed"]:
                report["failures"].append(name)
        # Detect source/overlay changes during the suite, not just at entry.
        if installed_identity(repo) != report["identity"]:
            raise RuntimeError("owner identity changed during qualification")
        if model_package_root is not None and hashlib.sha256(model.read_bytes()).hexdigest() != report["model_package"]["sha256"]:
            raise RuntimeError("model confirmation owner changed during qualification")
        report["passed"] = not report["failures"] and len(report["stages"]) == len(STAGES)
    except (RuntimeError, OSError, ImportError, ValueError) as error:
        report["failures"].append(f"{type(error).__name__}: {error}")
    finally:
        report["elapsed_s"] = time.monotonic() - started
        (output / "suite.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage-timeout-s", type=float, default=120.)
    parser.add_argument("--model-package-root", type=Path,
                        help="packages directory from a passing isolated-wheel suite; no installation")
    args = parser.parse_args(argv)
    report = qualify(args.repo, args.output, args.stage_timeout_s, args.model_package_root)
    print(json.dumps(dict(suite=str(args.output.absolute() / "suite.json"), passed=report["passed"])))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
