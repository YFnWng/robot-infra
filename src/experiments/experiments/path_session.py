"""Bind a frozen continuous path to the existing coordinated recording."""
import argparse
import hashlib
import json
import os
import subprocess
from pathlib import Path

import yaml
import numpy as np
from control_tasks.task_outcomes import TaskFailure


def _home_template():
    from ament_index_python.packages import get_package_share_directory
    return (Path(get_package_share_directory("catheter_control")) / "config"
            / "experiments" / "sparse_points" / "farther_tendon_12_plain.yaml")


def prepare_task(session, path_file):
    session = Path(session).resolve()
    manifest = json.loads((session / "session_manifest.json").read_text())
    if manifest.get("state") != "recording_ready":
        raise ValueError("coordinated recording must be recording_ready")
    if not manifest.get("command_output_enabled"):
        raise ValueError("recording controller output is disabled")
    content = Path(path_file).read_bytes()
    document = yaml.safe_load(content)
    if not isinstance(document, dict) or "generator" in document:
        raise ValueError("freeze one absolute path before controller comparisons")
    # Geometry validation remains with the canonical path client.
    from control_tasks.path_file import load_path_file
    spec = load_path_file(path_file, document["path_knots_m"][0])
    identity = json.loads((session / "controller_manifest.json").read_text())
    params = identity["controller_parameters"]
    synchronized = params.get("synchronized_sheath_rotation", False) is True
    if params["controller_velocity_max"][1] != 0 and not synchronized:
        raise ValueError("two-axis paths require rotation disabled")
    # One existing point task performs guarded home and then start acquisition.
    # No generated model targets or approach spline are involved.
    home_source = _home_template()
    home = yaml.safe_load(home_source.read_text())
    home["required_controller_status"] = {
        "command_output_enabled": True,
        "controller_velocity_max": params["controller_velocity_max"],
    }
    home["frame_id"] = spec.frame_id
    if synchronized:
        home["required_controller_status"]["synchronized_sheath_rotation"] = True
        home["rotation_guard"]["enabled"] = False
    home["marker_topic"] = spec.marker_topic
    home["generator"] = {"type": "absolute_targets_m",
                         "targets_m": [spec.knots_m[0].tolist()]}
    home["target_tolerance_mm"] = spec.final_tolerance_mm
    home["target_settle_time_s"] = spec.final_settle_time_s
    snapshot = session / "task_path.yaml"
    with snapshot.open("xb") as stream:
        stream.write(content)
    home_snapshot = session / "task_home.yaml"
    with home_snapshot.open("x") as stream:
        yaml.safe_dump(home, stream, sort_keys=False)
    with (session / "task_manifest.json").open("x") as stream:
        json.dump({"schema_version": 2, "task": (
                   "synchronized_rotation_continuous_path" if synchronized
                   else "two_axis_continuous_path"),
                   "path_source": str(Path(path_file).resolve()),
                   "path_sha256": hashlib.sha256(content).hexdigest(),
                   "path_knots_m": spec.knots_m.tolist(),
                   "initial_home": {"source": str(home_source),
                                    "joint_position": home["home"]["joint_position"],
                                    "protocol": "task_home.yaml",
                                    "journal": "acquisition_trials.jsonl"},
                   "initial_acquisition": {
                       "target_m": spec.knots_m[0].tolist(),
                       "tolerance_mm": spec.final_tolerance_mm,
                       "protocol": "task_home.yaml",
                       "journal": "acquisition_trials.jsonl",
                       "client": "control_tasks/catheter_sparse_point_experiment"},
                   "tracking_geometry": "frozen_knots_without_approach",
                   "client": "control_tasks/catheter_tip_path_file"}, stream,
                  indent=2, allow_nan=False)
    return ["ros2", "run", "control_tasks", "catheter_tip_path_file",
            str(snapshot), "--preserve-knots",
            "--result-file", str(session / "path_result.json")]


def validate_acquisition(journal, target):
    """A zero exit code can mean a point timeout; require its recorded success."""
    rows = [json.loads(line) for line in Path(journal).read_text().splitlines()]
    starts = [row for row in rows if row.get("event") == "trial_started"]
    completed = [row for row in rows if row.get("event") == "trial_completed"]
    if (not rows or rows[-1].get("event") != "task_completed"
            or rows[-1].get("reached") != 1 or rows[-1].get("timed_out") != 0
            or len(starts) != 1 or len(completed) != 1
            or completed[0].get("reached") != 1 or completed[0].get("timed_out") != 0
            or not np.array_equal(np.asarray(starts[0].get("target_m")), np.asarray(target))):
        raise TaskFailure("path start acquisition incomplete or unsuccessful; tracking not started")


def execute_task(session, command):
    session = Path(session).resolve()
    protocol = json.loads((session / "task_manifest.json").read_text())
    journal = session / protocol["initial_acquisition"]["journal"]
    subprocess.run([
        "ros2", "run", "control_tasks", "catheter_sparse_point_experiment",
        str(session / "task_home.yaml"),
        "--trial-records", str(journal),
    ], check=True)
    validate_acquisition(journal, protocol["initial_acquisition"]["target_m"])
    # The point action has finished before any continuous path goal is sent.
    os.execvp(command[0], command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--path", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required: this task can arm and actuate")
    command = prepare_task(args.session, args.path)
    try:
        execute_task(args.session, command)
    except subprocess.CalledProcessError as error:
        print(f"Path acquisition failed (exit {error.returncode}); tracking not started. "
              f"See {args.session / 'acquisition_trials.jsonl'}", flush=True)
        raise SystemExit(error.returncode if error.returncode > 0 else 2)
    except TaskFailure as error:
        print(str(error), flush=True)
        raise SystemExit(2)
