"""Isolated historical/promoted package A/B using the existing offline probes.

Never installs packages or opens an actuator/camera. Sequential replay is not
a scheduling qualification; paced replay adds shared Python compute contention.
Spatial mode checks numerical equivalence. ROS mode uses a disarmed production
callback probe on an isolated domain, not a full active-control stack.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import time
import tempfile
import zipfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def variant_order(selection, repeat):
    if selection == "both":
        return ("old", "new") if repeat % 2 == 0 else ("new", "old")
    if selection in ("old", "new"):
        return (selection,)
    raise ValueError(f"unknown package selection: {selection}")


def replay_windows(offsets, duration_s, repeats):
    """Bound fresh-prefix replay work; never skip history to accelerate a test."""
    import math
    if (not 1 <= len(offsets) <= 8 or len(set(offsets)) != len(offsets)
            or any(not math.isfinite(v) or v < 0 for v in offsets)
            or repeats*sum(v+duration_s for v in offsets) > 1800):
        raise ValueError("require 1..8 unique finite offsets and <=1800s total replay time")
    return list(enumerate(offsets, 1))


def ros_window_summary(runs, windows, duration_s):
    """Keep per-window distributions: pooled timing must not hide a bad window."""
    from catheter_control.safety.validation import percentile_metrics
    rows = []
    for index, offset in windows:
        selected = [row for row in runs if row['window_index'] == index]
        plans = [plan for row in selected for plan in row['report']['plans']]
        if not plans:
            raise RuntimeError(f"window {index} has no measured plans")
        metrics = {key: percentile_metrics([plan[key] for plan in plans if plan.get(key) is not None])
                   for key in ('elapsed_ms', 'snapshot_receive_ms', 'snapshot_transfer_ms',
                               'snapshot_source_age_ms', 'snapshot_header_age_ms',
                               'snapshot_arrival_age_at_plan_end_ms')
                   if any(plan.get(key) is not None for plan in plans)}
        rows.append({'window_index': index, 'start_offset_s': offset, 'duration_s': duration_s,
            'run_count': len(selected), 'plan_count': len(plans), 'timing_ms': metrics,
            'deadline_misses': sum(row['report']['deadline_misses'] for row in selected),
            'invalid_plans': sum(not plan['valid'] for plan in plans),
            'qualification_passed': all(row['report']['handoff_qualification']['passed'] for row in selected),
            'cadence_passed': all(row['report']['plan_coverage']['passed'] for row in selected),
            'plan_coverage': [row['report']['plan_coverage'] for row in selected],
            'recorded_position_window': selected[0]['report']['recorded_position_window'],
            'warmup_rows': [warm for row in selected for warm in row['report']['pre_window_warmup']]})
    return rows


def safe_member(name):
    path = Path(name)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"unsafe archive member: {name}")
    return path


def unpack_models(archive, destination, *, wheel=False):
    """Extract regular model/common files only; never activate editable hooks."""
    if wheel:
        with zipfile.ZipFile(archive) as source:
            for member in source.infolist():
                path = safe_member(member.filename)
                if path.parts[0] not in ("cr_meta_lnn", "cr_common") or member.is_dir():
                    continue
                if (member.external_attr >> 16) & 0o170000 == 0o120000:
                    raise ValueError("symlinks are not allowed in package fixtures")
                target = destination/path
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(source.read(member))
    else:
        with tarfile.open(archive, "r:gz") as source:
            for member in source.getmembers():
                path = safe_member(member.name)
                if path.parts[0] not in ("cr_meta_lnn", "cr_common") or member.isdir():
                    continue
                if not member.isfile():
                    raise ValueError("only regular package files are allowed")
                target = destination/path
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(source.extractfile(member).read())


def prepare_packages(backup, output):
    """Preserve rollback evidence and stage both variants without pip/global writes."""
    preserved = output/"promotion_backup"
    shutil.copytree(backup, preserved)
    old, new = output/"packages"/"old", output/"packages"/"new"
    old.mkdir(parents=True)
    new.mkdir()
    unpack_models(preserved/"previous-packages.tar.gz", old)
    models = list(preserved.glob("cr_meta_lnn-*.whl"))
    common = list(preserved.glob("cr_common-*.whl"))
    if len(models) != 1 or len(common) != 1:
        raise ValueError("backup must contain exactly one promoted model and common wheel")
    unpack_models(models[0], new, wheel=True)
    unpack_models(common[0], new, wheel=True)
    # The historical common distribution was editable. Use the preserved
    # candidate only when its FK hash matches the old recorded source hash.
    hashes = {}
    for line in (preserved/"previous-source.sha256").read_text().splitlines():
        sha, name = line.split(maxsplit=1)
        hashes[Path(name).name] = sha
    runtime = Path("cr_meta_lnn/deployment/v171_streaming_runtime.py")
    fk = Path("cr_common/kinematics.py")
    if digest(old/runtime) != hashes["v171_streaming_runtime.py"]:
        raise ValueError("historical runtime hash mismatch")
    if not (old/fk).exists():
        if digest(new/fk) != hashes["kinematics.py"]:
            raise ValueError("editable historical FK differs; cannot reconstruct old package")
        shutil.copytree(new/"cr_common", old/"cr_common")
    if digest(old/fk) != hashes["kinematics.py"]:
        raise ValueError("historical FK hash mismatch")
    return {name: {"root": str(root.resolve()), "runtime_sha256": digest(root/runtime),
                   "fk_sha256": digest(root/fk)} for name, root in (("old", old), ("new", new))}


def verify_imports(root):
    paths = {}
    for name in ("cr_meta_lnn.deployment.v171_streaming_runtime", "cr_common.kinematics"):
        path = Path(importlib.import_module(name).__file__).resolve()
        if not path.is_relative_to(root.resolve()):
            raise RuntimeError(f"package isolation failed: {name} -> {path}")
        paths[name] = {"path": str(path), "sha256": digest(path)}
    return paths


def worker(argv):
    root, mode, identity_file, *arguments = argv
    identity = verify_imports(Path(root))
    Path(identity_file).write_text(json.dumps(identity, indent=2)+"\n")
    if mode == "frozen":
        from .compare_spatial import evaluate
        return evaluate(*arguments)
    if mode == "ros":
        from .compare_ros import main as ros_main
        return ros_main(arguments)
    module = "compute_profile" if mode == "sequential" else "qualify_paced_compute"
    importlib.import_module("runtime_supervision."+module).main(arguments)


def run_logged(command, output, name, environment, timeout_s):
    """Bound one owned child, preserving logs and killing it on interruption."""
    print(f"starting {name}; log={output/(name+'.log')}", flush=True)
    started = time.monotonic()
    row = {"name": name, "command": command}
    with (output/(name+".log")).open("w") as log:
        process = subprocess.Popen(command, cwd=output.resolve(), env=environment,
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            while True:
                remaining = timeout_s-(time.monotonic()-started)
                if remaining <= 0:
                    process.kill()
                    process.wait()
                    row.update(error="bounded worker timeout", exit_code=-1)
                    break
                try:
                    row["exit_code"] = process.wait(timeout=min(15., remaining))
                    break
                except subprocess.TimeoutExpired:
                    print(f"running {name}: {time.monotonic()-started:.0f}s, pid={process.pid}", flush=True)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
    row["wall_s"] = time.monotonic()-started
    print(f"finished {name}: exit={row['exit_code']}, wall={row['wall_s']:.1f}s", flush=True)
    return row


def run_split_logged(command, output, name, environment, timeout_s):
    """Coordinate two owned, isolated workers; always reap both on failure."""
    run_output = output/name
    run_output.mkdir()
    barrier = run_output/"coordination"
    barrier.mkdir()
    children, logs = {}, []
    mailbox = tempfile.TemporaryDirectory(prefix="compute-replay-exchange-")
    started = time.monotonic()
    row = {"name": name, "children": {}}
    try:
        for role, domain in (("planner", "213"), ("estimator", "214")):
            child_command = list(command)
            for option, value in (("--output", str((run_output/role).resolve())),
                                  ("--ros-ablation", role+"-only"),
                                  ("--device", "cuda" if role == "planner" else "cpu")):
                child_command[child_command.index(option)+1] = value
            child_command[6] = str((run_output/(role+"_identity.json")).resolve())
            if role == "estimator":
                if "--ros-plan-fixture" in child_command:
                    index = child_command.index("--ros-plan-fixture")
                    del child_command[index:index+2]
            if "--ros-evolving-snapshots" in child_command:
                child_command += ["--exchange-dir", mailbox.name]
            child_command += ["--coordination-dir", str(barrier.resolve()), "--process-role", role]
            child_environment = dict(environment, ROS_DOMAIN_ID=domain)
            log_path = output/(name+"_"+role+".log")
            log = log_path.open("w")
            logs.append(log)
            children[role] = subprocess.Popen(child_command, cwd=output.resolve(), env=child_environment,
                                               stdout=log, stderr=subprocess.STDOUT)
            row["children"][role] = {"command": child_command, "log": str(log_path),
                                       "pid": children[role].pid}
        print(f"starting {name}: coordinated planner/estimator; logs={output}", flush=True)
        released = False
        next_log = started+15
        while True:
            now = time.monotonic()
            codes = {role: child.poll() for role, child in children.items()}
            if any(code not in (None, 0) for code in codes.values()):
                raise RuntimeError(f"split worker failed: {codes}")
            if not released:
                if any(code is not None for code in codes.values()):
                    raise RuntimeError("split worker exited before the start barrier")
                if all((barrier/(role+".ready")).exists() for role in children):
                    temporary = barrier/"epoch.tmp"
                    temporary.write_text(json.dumps({"epoch_monotonic_s": now+1.}))
                    temporary.replace(barrier/"epoch.json")
                    released = True
            elif all(code == 0 for code in codes.values()):
                break
            if now-started >= timeout_s:
                raise TimeoutError("bounded split worker timeout")
            if now >= next_log:
                print(f"running {name}: {now-started:.0f}s; barrier_released={released}", flush=True)
                next_log = now+15
            time.sleep(.05)
        reports = {role: json.loads((run_output/role/"report.json").read_text()) for role in children}
        if reports["planner"]["replay_epoch_monotonic_s"] != reports["estimator"]["replay_epoch_monotonic_s"]:
            raise RuntimeError("split replay epochs differ")
        if not reports["estimator"]["stages"].get("ros_marker_owner", {}).get("count", 0):
            raise RuntimeError("split estimator had no measured marker work")
        merged = dict(reports["planner"], isolated_estimator=reports["estimator"])
        if "--ros-evolving-snapshots" in command:
            if len({p['source_ns'] for p in merged['plans']}) < 2:
                raise RuntimeError("evolving replay did not consume changing snapshots")
            if not merged['handoff_qualification']['passed']:
                raise RuntimeError("snapshot handoff numerical qualification failed")
        (run_output/"report.json").write_text(json.dumps(merged, indent=2)+"\n")
        row["exit_code"] = 0
    except Exception as error:
        row.update(exit_code=-1, error=repr(error))
    finally:
        for child in children.values():
            if child.poll() is None:
                child.kill()
            child.wait()
        for role, child in children.items():
            row["children"][role]["exit_code"] = child.returncode
        for log in logs:
            log.close()
        mailbox.cleanup()
    row["wall_s"] = time.monotonic()-started
    return row


def main(argv=None):
    arguments = sys.argv[1:] if argv is None else argv
    if arguments and arguments[0] == "_worker":
        return worker(arguments[1:])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--backup", type=Path, required=True)
    parser.add_argument("--model-manifest", type=Path, required=True)
    parser.add_argument("--limits-file", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=("sequential", "paced", "spatial", "ros"), default="sequential")
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--estimator-device", choices=("cpu", "cuda"), help="ROS estimator device; float32")
    parser.add_argument("--variants", choices=("both", "old", "new"), default="both")
    from .compare_ros import ABLATIONS
    parser.add_argument("--ros-ablation", choices=ABLATIONS, default="combined")
    parser.add_argument("--ros-plan-fixture", type=Path)
    parser.add_argument("--ros-processes", choices=("shared", "split"), default="shared",
                        help="split: coordinated CPU estimator/GPU planner; frozen or evolving inputs")
    parser.add_argument("--ros-evolving-snapshots", action="store_true",
                        help="split replay with latest-only CPU snapshot handoff; no fixed fixture")
    window_args = parser.add_mutually_exclusive_group()
    window_args.add_argument("--start-offset-s", type=float, default=32.)
    window_args.add_argument("--ros-window-offsets", type=float, nargs='+',
                             help="1..8 receipt-relative windows for evolving split replay; seconds")
    parser.add_argument("--duration-s", type=float, default=2.)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--timeout-s", type=float, default=600.)
    parser.add_argument("--profiler", action="store_true", help="separate intrusive captures, not timing qualification")
    args = parser.parse_args(arguments)
    if args.ros_window_offsets is not None and not args.ros_evolving_snapshots:
        parser.error("multiple windows require evolving split replay")
    if args.ros_processes == "split" and not (
            args.mode == "ros" and args.variants == "new" and args.ros_ablation == "combined"
            and args.device == "cuda" and args.estimator_device == "cpu"
            and (bool(args.ros_plan_fixture) != args.ros_evolving_snapshots)):
        parser.error("split requires ROS/new/combined, CUDA planner, CPU estimator and either fixed fixture or evolving snapshots")
    if args.ros_evolving_snapshots and args.ros_processes != "split":
        parser.error("evolving snapshot handoff requires split processes")
    if args.mode != "ros" and (args.estimator_device is not None or args.variants != "both"):
        parser.error("estimator device and single-package selection require --mode ros")
    if args.mode != "ros" and args.ros_ablation != "combined":
        parser.error("--ros-ablation requires --mode ros")
    if args.ros_plan_fixture and (args.mode != "ros" or args.ros_ablation not in ("combined", "planner-only")):
        parser.error("--ros-plan-fixture requires ROS combined or planner-only")
    import math
    if (not all(math.isfinite(v) for v in (args.duration_s, args.start_offset_s, args.timeout_s))
            or not 0 < args.duration_s <= 10 or args.start_offset_s < 0
            or not 0 < args.timeout_s <= 3600 or not 1 <= args.repeats <= 4):
        parser.error("require bounded finite duration <=10s, timeout <=3600s and 1..4 repeats")
    if args.profiler and (args.mode != "sequential" or args.duration_s > 2):
        parser.error("intrusive profiling uses sequential replay of at most 2s")
    if args.mode == "paced" and args.device != "cuda":
        parser.error("paced comparison requires CUDA; no fallback")
    try:
        windows = (replay_windows(args.ros_window_offsets, args.duration_s, args.repeats)
                   if args.ros_window_offsets is not None else [(1, args.start_offset_s)])
    except ValueError as error:
        parser.error(str(error))
    if args.ros_window_offsets is not None and any(offset+args.duration_s+60 >= args.timeout_s for _, offset in windows):
        parser.error("worker timeout must exceed prefix + window + 60s startup/shutdown allowance")
    from .compute_profile import require_controller_idle
    require_controller_idle()
    args.output.mkdir(parents=True, exist_ok=False)
    packages = prepare_packages(args.backup, args.output)
    bag_files = sorted((args.session/"robot_bag").glob("*"))
    report = {"scope": __doc__, "packages": packages, "mode": args.mode,
              "intrusive_profiler": args.profiler, "runs": [],
              "inputs": {str(p.resolve()): digest(p) for p in
                         [args.model_manifest, args.limits_file, args.session/"trials.jsonl",
                          args.session/"controller_manifest.json", *bag_files] if p.is_file()},
              "limitations": ["Same recorded inputs, not identical estimator posterior states",
                  "Sequential fixture uses zero-width engaged belief and seeded fresh proposals",
                  "Paced fixture uses recorded gains but assumes engaged and zero gap",
                  "No ROS executor, camera GPU work, homing ownership or online belief replay",
                  "Does not establish hardware qualification or a confirmed root cause"]}
    if args.mode == "ros":
        report["ros_ablation"] = args.ros_ablation
        report["ros_processes"] = args.ros_processes
        report["ros_evolving_snapshots"] = args.ros_evolving_snapshots
        report["windows"] = [{'window_index': index, 'start_offset_s': offset,
                               'duration_s': args.duration_s} for index, offset in windows]
        report["variants"] = args.variants
        report["requested_devices"] = {"planner": args.device,
                                       "estimator": args.estimator_device or args.device,
                                       "estimator_dtype": "float32"}
        if args.ros_plan_fixture:
            report["inputs"][str(args.ros_plan_fixture.resolve())] = digest(args.ros_plan_fixture)
            metadata = args.ros_plan_fixture.with_suffix(".json")
            report["inputs"][str(metadata.resolve())] = digest(metadata)
        report["limitations"] = ["Disarmed production estimator/diagnostics plus discarded MPPI solves",
                                 "No active takeup, manager, serial driver or camera workload",
                                 "Recorded gains but all-engaged zero-gap planner fixture",
                                 "Does not establish hardware qualification or a confirmed root cause"]
    destination = args.output/"comparison.json"
    destination.write_text(json.dumps(report, indent=2)+"\n")
    for repeat in range(args.repeats):
        ordered_windows = windows if repeat % 2 == 0 else list(reversed(windows))
        for index, offset, variant in [(index, offset, variant) for index, offset in ordered_windows
                                      for variant in variant_order(args.variants, repeat)]:
            name = f"{repeat+1:02d}_{variant}"
            if args.ros_window_offsets is not None:
                name = f"w{index:02d}_s{offset:g}_"+name
            root = Path(packages[variant]["root"])
            run_output = args.output/name
            environment = os.environ.copy()
            environment["PYTHONNOUSERSITE"] = "1"
            environment["PYTHONPATH"] = str(root)+os.pathsep+environment.get("PYTHONPATH", "")
            if args.mode == "ros":
                environment["ROS_DOMAIN_ID"] = "213"
            command = [sys.executable, "-m", "runtime_supervision.compare_compute", "_worker",
                       str(root), "sequential" if args.mode == "spatial" else args.mode,
                       str(args.output/(name+"_identity.json")),
                       str(args.session.resolve()), "--model-manifest", str(args.model_manifest.resolve()),
                       "--limits-file", str(args.limits_file.resolve()), "--output", str(run_output.resolve()),
                       "--start-offset-s", str(offset), "--duration-s", str(args.duration_s)]
            if args.mode == "paced":
                command += ["--estimator-device", "cuda:0", "--estimator-dtype", "float32"]
            else:
                command += ["--device", args.device]
                if args.mode != "ros":
                    command += ["--samples", "512"]
            if args.mode == "spatial":
                command.append("--capture-states")
            if args.mode == "ros":
                command += ["--ros-ablation", args.ros_ablation]
                if args.ros_evolving_snapshots:
                    command.append("--ros-evolving-snapshots")
                if args.estimator_device is not None:
                    command += ["--estimator-device", args.estimator_device]
                if args.ros_plan_fixture:
                    command += ["--ros-plan-fixture", str(args.ros_plan_fixture.resolve())]
            if args.profiler:
                command.append("--profiler")
            runner = run_split_logged if args.ros_processes == "split" else run_logged
            row = runner(command, args.output, name, environment, args.timeout_s)
            row["variant"] = variant
            row['window_index'] = index
            row['start_offset_s'] = offset
            child_report = run_output/"report.json"
            if row["exit_code"] == 0 and child_report.is_file():
                row["report"] = json.loads(child_report.read_text())
            report["runs"].append(row)
            destination.write_text(json.dumps(report, indent=2)+"\n")
            if row["exit_code"]:
                raise RuntimeError(f"comparison stopped: inspect {name}.log; no fallback used")
    if args.ros_evolving_snapshots:
        report['window_summaries'] = ros_window_summary(report['runs'], windows, args.duration_s)
        report['timing_qualification_passed'] = all(
            row['qualification_passed'] and row['cadence_passed']
            and row['deadline_misses'] == 0 and row['invalid_plans'] == 0
            for row in report['window_summaries'])
    if args.mode in ("sequential", "spatial"):
        reference = report["runs"][0]["report"]
        report["matched_input_checks"] = {
            key: all(row["report"][key] == reference[key] for row in report["runs"])
            for key in ("window", "configuration", "input_sha256", "proposal_arrays_sha256")}
        reference_decisions = [(r["source_ns"], r["accepted"], r["reason"]) for r in reference["marker_updates"]]
        report["marker_decisions_equal"] = all(
            [(r["source_ns"], r["accepted"], r["reason"]) for r in row["report"]["marker_updates"]]
            == reference_decisions for row in report["runs"])
        report["comparison_has_planner_work"] = all(bool(row["report"]["plans"]) for row in report["runs"])
        report["matched_comparison_valid"] = (
            all(report["matched_input_checks"].values()) and report["comparison_has_planner_work"])
    elif args.mode == "paced":
        report["comparison_has_planner_work"] = all(
            any(bool(case["plans"]) for case in row["report"]["cases"].values()) for row in report["runs"])
    else:
        report["comparison_has_planner_work"] = all(bool(row["report"]["plans"]) for row in report["runs"])
    if args.ros_plan_fixture:
        expected = digest(args.ros_plan_fixture)
        report["matched_plan_fixture"] = all(
            row["report"]["plan_fixture_sha256"] == expected for row in report["runs"])
        if not report["matched_plan_fixture"]:
            raise RuntimeError("ROS planner fixture mismatch")
    if args.mode == "spatial":
        from .compare_spatial import cartesian_difference, frozen_difference
        first = {variant: next(row for row in report["runs"] if row["variant"] == variant)
                 for variant in ("old", "new")}
        report["replay_cartesian_drift"] = cartesian_difference(
            first["old"]["report"]["cartesian_trajectory"], first["new"]["report"]["cartesian_trajectory"])
        frozen = {}
        for variant in ("old", "new"):
            name = "frozen_"+variant
            environment = os.environ.copy()
            environment["PYTHONNOUSERSITE"] = "1"
            root = packages[variant]["root"]
            environment["PYTHONPATH"] = root+os.pathsep+environment.get("PYTHONPATH", "")
            result_path = args.output/(name+".json")
            command = [sys.executable, "-m", "runtime_supervision.compare_compute", "_worker", root,
                       "frozen", str(args.output/(name+"_identity.json")),
                       str((args.output/first["old"]["name"]/"report.json").resolve()),
                       str(args.model_manifest.resolve()), args.device, str(result_path.resolve())]
            execution = run_logged(command, args.output, name, environment, args.timeout_s)
            if execution["exit_code"]:
                raise RuntimeError(f"frozen comparison failed: inspect {name}.log")
            frozen[variant] = json.loads(result_path.read_text())
        report["frozen_rollout_difference"] = frozen_difference(frozen["old"], frozen["new"])
        report["intrusive_state_capture"] = True
        report["limitations"].append("Spatial capture timings are intrusive; use prior non-capture runs for performance")
    destination.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({"comparison": str(destination)}), flush=True)


if __name__ == "__main__":
    main()
