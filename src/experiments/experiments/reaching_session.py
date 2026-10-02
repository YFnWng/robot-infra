"""Bind the existing guarded reaching client to a ready research recording."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path

import yaml


def prepare_task(session, target_file):
    session = Path(session).resolve()
    manifest = json.loads((session / "session_manifest.json").read_text())
    if manifest.get("state") != "recording_ready":
        raise ValueError("coordinated recording must be recording_ready")
    if not manifest.get("command_output_enabled"):
        raise ValueError("recording controller output is disabled")
    content = Path(target_file).read_bytes()
    config = yaml.safe_load(content)
    generator = config.get("generator", {})
    if generator.get("type") != "absolute_targets_m":
        raise ValueError("research attribution requires reviewed frozen absolute targets")
    targets = generator.get("absolute_targets_m", generator.get("targets_m"))
    if not isinstance(targets, list) or not targets or any(
            not isinstance(target, list) or len(target) != 3
            or not all(isinstance(value, (float, int)) and math.isfinite(value)
                       for value in target) for target in targets):
        raise ValueError("targets must contain finite Cartesian XYZ points in metres")
    # Target schema validation stays with the guarded control_tasks client.
    if (session / "trials.jsonl").exists():
        raise ValueError("session already contains a task journal; allocate a new session")
    snapshot = session / "task_targets.yaml"
    with snapshot.open("xb") as stream:
        stream.write(content)
    protocol = {
        "schema_version": 1, "task": "two_axis_sparse_reaching",
        "target_source": str(Path(target_file).resolve()),
        "target_sha256": hashlib.sha256(content).hexdigest(),
        "targets_m": targets,
        "client": "control_tasks/catheter_sparse_point_experiment",
    }
    with (session / "task_manifest.json").open("x") as stream:
        json.dump(protocol, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return ["ros2", "run", "control_tasks", "catheter_sparse_point_experiment",
            str(snapshot), "--trial-records", str(session / "trials.jsonl")]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--targets", type=Path, required=True)
    parser.add_argument("--execute", action="store_true",
                        help="Explicitly authorize the guarded homing/reaching task")
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required: this task homes and actuates the robot")
    command = prepare_task(args.session, args.targets)
    # Replace this process so Ctrl+C reaches the canonical client's cancellation path.
    os.execvp(command[0], command)


if __name__ == "__main__":
    main()
