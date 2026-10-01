#!/usr/bin/env python3
"""Capture the source and artifact closure of an observed controller session.

This is a read-only source scanner. It follows Python imports across the active
catheter controller, cr_meta_lnn, cr-common, and control packages; records ROS
package/interface metadata; and verifies artifact hashes from a recorded
control.launch.py session manifest.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.util
import json
import subprocess
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_output(path: Path, *args: str, check: bool = True) -> str:
    result = subprocess.run(
        ["git", "-C", str(path), *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if check and result.returncode != 0:
        raise RuntimeError(
            f"git {' '.join(args)} failed at {path}: {result.stderr.strip()}"
        )
    return result.stdout.strip()


def repository_state(path: Path) -> dict:
    root_text = git_output(path, "rev-parse", "--show-toplevel", check=False)
    if not root_text:
        return {"path": str(path.resolve()), "is_git_repository": False}
    root = Path(root_text).resolve()
    status = git_output(root, "status", "--porcelain=v1").splitlines()
    return {
        "path": str(root),
        "is_git_repository": True,
        "branch": git_output(root, "branch", "--show-current"),
        "commit": git_output(root, "rev-parse", "HEAD"),
        "dirty": bool(status),
        "status_entry_count": len(status),
    }


def module_roots(workspace: Path) -> dict[str, Path]:
    return {
        "catheter_control": workspace / "robot-infra" / "src" / "catheter_control" / "catheter_control",
        "automation": workspace / "robot-infra" / "src" / "automation" / "automation",
        "control_interface_py": workspace / "robot-infra" / "src" / "control_interface" / "control_interface_py",
        "cr_meta_lnn": workspace / "cr_meta_lnn",
        "cr_common": workspace / "cr-common" / "cr_common",
        "control": workspace / "control",
        "shape_tracking": workspace / "catheter-shape-tracking" / "src" / "shape_tracking",
    }


def resolve_module(module: str, roots: dict[str, Path]) -> Path | None:
    for prefix, root in roots.items():
        if module == prefix:
            candidate = root / "__init__.py"
            return candidate.resolve() if candidate.is_file() else None
        if not module.startswith(prefix + "."):
            continue
        suffix = module[len(prefix) + 1:].split(".")
        file_candidate = root.joinpath(*suffix).with_suffix(".py")
        if file_candidate.is_file():
            return file_candidate.resolve()
        package_candidate = root.joinpath(*suffix) / "__init__.py"
        if package_candidate.is_file():
            return package_candidate.resolve()
    return None


def parent_package_modules(module: str) -> Iterable[str]:
    parts = module.split(".")
    for length in range(1, len(parts)):
        yield ".".join(parts[:length])


def imports_for(path: Path, module: str) -> tuple[set[str], set[str]]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported: set[str] = set()
    roots: set[str] = set()
    package = module if path.name == "__init__.py" else module.rpartition(".")[0]
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported.add(alias.name)
                roots.add(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                relative = "." * node.level + (node.module or "")
                try:
                    target = importlib.util.resolve_name(relative, package)
                except (ImportError, ValueError):
                    continue
            else:
                target = node.module or ""
            if target:
                imported.add(target)
                roots.add(target.split(".")[0])
                for alias in node.names:
                    if alias.name != "*":
                        imported.add(f"{target}.{alias.name}")
    return imported, roots


def trace_python_closure(
    entries: dict[str, str], roots: dict[str, Path]
) -> tuple[dict[Path, set[str]], set[str], set[str]]:
    queue: deque[tuple[str, str]] = deque(entries.items())
    visited_modules: set[str] = set()
    files: dict[Path, set[str]] = {}
    external_roots: set[str] = set()
    unresolved_local: set[str] = set()
    local_prefixes = set(roots)

    while queue:
        module, reason = queue.popleft()
        if module in visited_modules:
            path = resolve_module(module, roots)
            if path is not None:
                files.setdefault(path, set()).add(reason)
            continue
        visited_modules.add(module)

        path = resolve_module(module, roots)
        if path is None:
            top = module.split(".")[0]
            if top in local_prefixes:
                unresolved_local.add(module)
            else:
                external_roots.add(top)
            continue
        files.setdefault(path, set()).add(reason)

        for parent in parent_package_modules(module):
            parent_path = resolve_module(parent, roots)
            if parent_path is not None:
                queue.append((parent, f"parent package of {module}"))

        imported, imported_roots = imports_for(path, module)
        for imported_module in imported:
            imported_path = resolve_module(imported_module, roots)
            if imported_path is not None:
                queue.append((imported_module, f"imported by {module}"))
            else:
                top = imported_module.split(".")[0]
                if top in local_prefixes:
                    # A from-imported class/function is commonly represented as
                    # module.symbol. The containing module is already queued;
                    # report only unresolved module-like references.
                    containing = imported_module.rpartition(".")[0]
                    if not containing or resolve_module(containing, roots) is None:
                        unresolved_local.add(imported_module)
                else:
                    external_roots.add(top)
        external_roots.update(root for root in imported_roots if root not in local_prefixes)

    return files, external_roots, unresolved_local


def workspace_relative(path: Path, workspace: Path) -> str:
    try:
        return path.resolve().relative_to(workspace.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def source_record(path: Path, workspace: Path, reasons: Iterable[str], role: str) -> dict:
    resolved = path.resolve()
    record = {
        "path": workspace_relative(resolved, workspace),
        "absolute_path": str(resolved),
        "role": role,
        "reasons": sorted(set(reasons)),
        "exists": resolved.is_file(),
    }
    if not resolved.is_file():
        return record
    record.update({"bytes": resolved.stat().st_size, "sha256": sha256(resolved)})
    repo_text = git_output(resolved.parent, "rev-parse", "--show-toplevel", check=False)
    if repo_text:
        repo = Path(repo_text).resolve()
        relative = resolved.relative_to(repo).as_posix()
        tracked = subprocess.run(
            ["git", "-C", str(repo), "ls-files", "--error-unmatch", "--", relative],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode == 0
        status = git_output(repo, "status", "--porcelain=v1", "--", relative)
        record.update({
            "repository": repo.name,
            "repository_relative_path": relative,
            "tracked": tracked,
            "git_status": status,
        })
    return record


def explicit_source_files(workspace: Path, session: dict) -> list[tuple[Path, str]]:
    robot = workspace / "robot-infra"
    result: list[tuple[Path, str]] = [
        (robot / "src/catheter_control/launch/control.launch.py", "hardware launch"),
        (robot / "src/catheter_control/catheter_control/bootstrap.py", "selected console-script bootstrap"),
        (robot / "src/catheter_control/setup.py", "ROS Python entry points"),
        (robot / "src/catheter_control/setup.cfg", "ROS Python installation"),
        (robot / "src/catheter_control/package.xml", "ROS package metadata"),
        (robot / "src/catheter_control/resource/catheter_control", "ament resource"),
        (robot / "src/automation/config/catheter_limits.yaml", "source hardware limits"),
        (robot / "src/automation/setup.py", "perception/experiment entry points"),
        (robot / "src/automation/setup.cfg", "automation installation"),
        (robot / "src/automation/package.xml", "perception/experiment package metadata"),
        (robot / "src/automation/resource/automation", "automation ament resource"),
        (robot / "src/control_interface/CMakeLists.txt", "ROS interface generation"),
        (robot / "src/control_interface/launch/launch.py", "manager and serial launch"),
        (robot / "src/control_interface/package.xml", "ROS interface metadata"),
    ]
    controller = session.get("controller_config") or {}
    if controller.get("path"):
        result.append((Path(controller["path"]), "resolved controller profile"))
    performance = session.get("performance_config") or {}
    if performance.get("path"):
        result.append((Path(performance["path"]), "resolved performance profile"))
    interface_root = robot / "src/control_interface"
    for folder in ("msg", "srv", "action"):
        root = interface_root / folder
        if root.is_dir():
            result.extend((path, "ROS interface definition") for path in sorted(root.iterdir()) if path.is_file())
    return result


def artifact_records(session: dict) -> list[dict]:
    result: list[dict] = []
    for name, recorded in sorted((session.get("artifacts") or {}).items()):
        if not recorded:
            result.append({"name": name, "selected": False})
            continue
        path = Path(recorded["path"]).expanduser().resolve()
        current = {
            "exists": path.is_file(),
            "path": str(path),
        }
        if path.is_file():
            current.update({"bytes": path.stat().st_size, "sha256": sha256(path)})
        result.append({
            "name": name,
            "selected": True,
            "recorded": recorded,
            "current": current,
            "matches_recorded": (
                current.get("exists") is True
                and current.get("bytes") == recorded.get("bytes")
                and current.get("sha256") == recorded.get("sha256")
            ),
        })
    return result


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=here.parents[3])
    parser.add_argument("--session-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--baseline-name",
        default="grouped_no_rotation_hardware_20260929_175554",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    workspace = args.workspace.resolve()
    session_path = args.session_manifest.expanduser().resolve()
    session = json.loads(session_path.read_text(encoding="utf-8"))

    # Trace selected executables rather than every branch listed in each
    # package's console-script dispatcher.
    entries = {
        "catheter_control.node": "hardware catheter_mppi controller",
        "catheter_control.trajectory_action": "trajectory action node launched with controller",
        "catheter_control.path_action": "path action node launched with controller",
        "catheter_control.sparse_point_experiment": "observed sparse-point task client",
        "automation.marker_tracking.node": "online four-marker perception",
        "control_interface_py.manager": "command arbitration and safety manager",
        "control_interface_py.device_serial_com": "Teensy serial bridge",
        # Package exports are lazy by design. Trace the selected runtime
        # explicitly so static closure capture remains complete.
        "cr_meta_lnn.deployment.v171_streaming_runtime": (
            "active v171 learned-model runtime"),
    }
    roots = module_roots(workspace)
    python_files, external_modules, unresolved = trace_python_closure(entries, roots)

    sources: dict[Path, dict] = {}
    for path, reasons in python_files.items():
        sources[path] = source_record(path, workspace, reasons, "python_import_closure")
    for path, reason in explicit_source_files(workspace, session):
        resolved = path.expanduser().resolve()
        if resolved in sources:
            sources[resolved]["reasons"] = sorted(set(sources[resolved]["reasons"] + [reason]))
        else:
            sources[resolved] = source_record(resolved, workspace, [reason], "runtime_metadata")

    repositories = []
    for directory in (
        "robot-infra", "cr_meta_lnn", "cr-common", "control",
        "catheter-shape-tracking",
    ):
        path = workspace / directory
        if path.exists():
            repositories.append(repository_state(path))

    controller = session.get("controller_config") or {}
    controller_path = Path(controller["path"]).resolve() if controller.get("path") else None
    controller_current = None
    if controller_path is not None:
        controller_current = {
            "path": str(controller_path),
            "exists": controller_path.is_file(),
        }
        if controller_path.is_file():
            controller_current.update({
                "bytes": controller_path.stat().st_size,
                "sha256": sha256(controller_path),
                "matches_recorded": (
                    controller_path.stat().st_size == controller.get("bytes")
                    and sha256(controller_path) == controller.get("sha256")
                ),
            })

    output = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "baseline_name": args.baseline_name,
        "qualification_scope": {
            "observed_session": "20260929_175554_mppi_demo",
            "task": "four sparse two-axis targets",
            "rotation_enabled": False,
            "result": "all targets reached",
            "command_output_was_enabled": bool(
                (session.get("controller_parameters") or {}).get("command_output_enabled")
            ),
            "warning": (
                "This is an observed successful no-rotation qualification, not a "
                "claim that the three-axis continuous-path stack is qualified."
            ),
            "provenance_gap": (
                "The controller launch manifest does not record the exact sparse-point "
                "task YAML or the independently launched marker/control-interface "
                "profiles. Their executable source is included conservatively, but "
                "their resolved task/launch parameters require a future unified "
                "full-stack session manifest."
            ),
        },
        "anchor_session_manifest": {
            "path": str(session_path),
            "bytes": session_path.stat().st_size,
            "sha256": sha256(session_path),
        },
        "repositories": repositories,
        "controller_profile": {
            "recorded": controller,
            "current": controller_current,
        },
        "resolved_controller_parameters": session.get("controller_parameters"),
        "artifacts": artifact_records(session),
        "python_import_closure": {
            "entry_modules": entries,
            "files": sorted(sources.values(), key=lambda value: value["path"]),
            "external_module_roots": sorted(external_modules),
            "unresolved_local_references": sorted(unresolved),
            "conservative": True,
            "note": (
                "Static tracing includes conditional imports and package __init__ "
                "side effects. Artifact selection identifies which optional "
                "checkpoints were active in the observed session."
            ),
        },
    }

    target = args.output.expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(target)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
