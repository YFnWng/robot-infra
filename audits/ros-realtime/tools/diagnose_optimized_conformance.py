"""Offline arithmetic ablations in disposable installed-package copies.

Never edits either supplied installation or the historical fixture. Reuses
the authoritative Phase-5 replay and comparisons in subprocesses.
"""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def method_span(source, name):
    tree = ast.parse(source)
    owner = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                 and node.name == "V171StreamingCatheterRuntime")
    method = next(node for node in owner.body if isinstance(node, ast.FunctionDef)
                  and node.name == name)
    start = min([method.lineno] + [node.lineno for node in method.decorator_list])
    return start-1, method.end_lineno


def replace_method(candidate, reference, name):
    lines = candidate.splitlines(keepends=True)
    first, last = method_span(candidate, name)
    old_first, old_last = method_span(reference, name)
    lines[first:last] = reference.splitlines(keepends=True)[old_first:old_last]
    return "".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--outdir", type=Path, required=True)
    args = parser.parse_args()
    args.outdir.mkdir(parents=True, exist_ok=False)
    relative = Path("cr_meta_lnn/deployment/v171_streaming_runtime.py")
    reference = (args.reference/relative).read_text()
    candidate = (args.candidate/relative).read_text()
    rows = {}
    for name, methods in {
        "optimized": (),
        "legacy_solve": ("_implicit_step",),
        "legacy_markers": ("_marker_points",),
        "legacy_both": ("_implicit_step", "_marker_points"),
    }.items():
        installation = args.outdir/name
        # Package trees only; no source-checkout imports or artifact changes.
        for package in ("cr_meta_lnn", "cr_common"):
            shutil.copytree(args.candidate/package, installation/package,
                            ignore=shutil.ignore_patterns("__pycache__"))
        source = candidate
        for method in methods:
            source = replace_method(source, reference, method)
        (installation/relative).write_text(source)
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(installation.resolve())+os.pathsep+environment.get("PYTHONPATH", "")
        report = installation/"conformance.json"
        manifest = installation/"cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json"
        completed = subprocess.run([
            sys.executable, "-m", "catheter_control.safety.conformance",
            "--model-manifest", str(manifest.resolve()), "--output", str(report.resolve())],
            cwd=installation, env=environment, capture_output=True, text=True)
        if not report.exists():
            raise RuntimeError(completed.stderr+completed.stdout)
        data = json.loads(report.read_text())
        rows[name] = {"returncode": completed.returncode,
                      "source_sha256": hashlib.sha256(source.encode()).hexdigest(),
                      "failures": data["failures"], "actual": data["actual"]}
        print(json.dumps({"variant": name, "failures": len(data["failures"])}), flush=True)
    output = args.outdir/"ablation.json"
    output.write_text(json.dumps(rows, indent=2)+"\n")
    print(json.dumps({"report": str(output)}))


if __name__ == "__main__":
    main()
