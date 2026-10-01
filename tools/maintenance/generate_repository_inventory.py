#!/usr/bin/env python3
"""Generate a read-only cleanup inventory for the catheter repositories.

The script never modifies a repository. It records tracked, untracked, ignored,
and modified files so cleanup decisions can be reviewed before anything is
moved or deleted.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

SKIP_DIRECTORY_NAMES = {".git", "build", "install", "log"}
CACHE_PARTS = {"__pycache__", ".pytest_cache", ".ruff_cache", ".mypy_cache"}
GENERATED_ROOTS = {"cache", "checkpoints", "evaluation", "export", "figures"}
GENERATED_SUFFIXES = {
    ".bag", ".db3", ".h5", ".hdf5", ".jpeg", ".jpg", ".mcap",
    ".mp4", ".npy", ".npz", ".png", ".pt", ".pth", ".svo", ".svo2",
}
SOURCE_SUFFIXES = {
    ".c", ".cc", ".cpp", ".cu", ".cuh", ".h", ".hh", ".hpp", ".ino",
    ".msg", ".py", ".srv", ".action", ".sh",
}
CONFIG_SUFFIXES = {".json", ".toml", ".yaml", ".yml"}
DOC_SUFFIXES = {".md", ".rst", ".tex"}


def run_git(repo: Path, *args: str, input_bytes: bytes | None = None) -> bytes:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        input=input_bytes,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"git {' '.join(args)} failed for {repo}: {detail}")
    return result.stdout


def nul_paths(payload: bytes) -> list[str]:
    return [item.decode("utf-8", errors="surrogateescape")
            for item in payload.split(b"\0") if item]


def git_status(repo: Path) -> dict[str, str]:
    tokens = nul_paths(run_git(repo, "status", "--porcelain=v1", "-z"))
    result: dict[str, str] = {}
    index = 0
    while index < len(tokens):
        token = tokens[index]
        if len(token) < 4:
            index += 1
            continue
        code = token[:2]
        path = token[3:]
        result[path] = code
        if "R" in code or "C" in code:
            index += 1  # consume the second rename/copy path
        index += 1
    return result


def iter_files(repo: Path) -> Iterable[Path]:
    for root, directories, filenames in os.walk(repo, followlinks=False):
        directories[:] = [
            name for name in directories if name not in SKIP_DIRECTORY_NAMES
        ]
        root_path = Path(root)
        for filename in filenames:
            path = root_path / filename
            if path.is_file() or path.is_symlink():
                yield path


def ignored_paths(repo: Path, relative_paths: list[str]) -> set[str]:
    if not relative_paths:
        return set()
    payload = b"\0".join(
        path.encode("utf-8", errors="surrogateescape")
        for path in relative_paths
    ) + b"\0"
    process = subprocess.run(
        ["git", "-C", str(repo), "check-ignore", "-z", "--stdin"],
        input=payload,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if process.returncode not in (0, 1):
        detail = process.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"git check-ignore failed for {repo}: {detail}")
    return set(nul_paths(process.stdout))


def classify(path: str) -> str:
    value = Path(path)
    parts = set(value.parts)
    suffix = value.suffix.lower()
    name = value.name.lower()
    if parts & CACHE_PARTS or suffix in {".pyc", ".pyo"}:
        return "cache"
    if name.endswith(".orig") or name.endswith("~"):
        return "backup"
    if suffix in SOURCE_SUFFIXES:
        if "test" in parts or "tests" in parts or name.startswith("test_"):
            return "test_source"
        return "source"
    if value.parts[:2] == ("artifacts", "manifests"):
        return "configuration"
    if suffix in GENERATED_SUFFIXES or (value.parts and value.parts[0] in GENERATED_ROOTS):
        return "generated_artifact"
    if suffix in CONFIG_SUFFIXES:
        return "configuration"
    if suffix in DOC_SUFFIXES:
        return "documentation"
    if "test" in parts or "tests" in parts:
        return "test_data"
    return "other"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def inspect_repo(repo: Path, hash_max_bytes: int) -> tuple[dict, list[dict]]:
    if not (repo / ".git").exists():
        raise ValueError(f"not a Git repository: {repo}")

    tracked = set(nul_paths(run_git(repo, "ls-files", "-z")))
    status = git_status(repo)
    absolute_files = list(iter_files(repo))
    relative_files = [
        path.relative_to(repo).as_posix() for path in absolute_files
    ]
    ignored = ignored_paths(repo, relative_files)

    records: list[dict] = []
    for absolute, relative in zip(absolute_files, relative_files):
        try:
            stat = absolute.stat()
        except FileNotFoundError:
            continue
        is_tracked = relative in tracked
        is_ignored = relative in ignored
        state = status.get(relative)
        if state is None:
            state = "tracked_clean" if is_tracked else (
                "ignored" if is_ignored else "untracked"
            )
        digest = ""
        if hash_max_bytes > 0 and stat.st_size <= hash_max_bytes and not absolute.is_symlink():
            digest = sha256(absolute)
        records.append({
            "repository": repo.name,
            "path": relative,
            "category": classify(relative),
            "tracked": is_tracked,
            "ignored": is_ignored,
            "git_status": state,
            "size_bytes": stat.st_size,
            "sha256": digest,
        })

    # Include tracked deletions, which are absent from the filesystem scan.
    known = {record["path"] for record in records}
    for relative in sorted(tracked - known):
        records.append({
            "repository": repo.name,
            "path": relative,
            "category": classify(relative),
            "tracked": True,
            "ignored": False,
            "git_status": status.get(relative, "tracked_missing"),
            "size_bytes": 0,
            "sha256": "",
        })

    category_counts = Counter(record["category"] for record in records)
    category_bytes: dict[str, int] = defaultdict(int)
    for record in records:
        category_bytes[record["category"]] += int(record["size_bytes"])

    branch = run_git(repo, "branch", "--show-current").decode().strip()
    commit = run_git(repo, "rev-parse", "HEAD").decode().strip()
    summary = {
        "repository": repo.name,
        "path": str(repo),
        "branch": branch,
        "commit": commit,
        "dirty": bool(status),
        "tracked_files": sum(record["tracked"] for record in records),
        "untracked_nonignored_files": sum(
            not record["tracked"] and not record["ignored"] for record in records
        ),
        "ignored_files": sum(record["ignored"] for record in records),
        "status_counts": dict(sorted(Counter(status.values()).items())),
        "category_counts": dict(sorted(category_counts.items())),
        "category_bytes": dict(sorted(category_bytes.items())),
        "largest_files": sorted(
            ({"path": r["path"], "size_bytes": r["size_bytes"],
              "category": r["category"], "tracked": r["tracked"]}
             for r in records),
            key=lambda item: int(item["size_bytes"]),
            reverse=True,
        )[:50],
    }
    return summary, sorted(records, key=lambda item: (item["repository"], item["path"]))


def parse_args() -> argparse.Namespace:
    script = Path(__file__).resolve()
    default_workspace = script.parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=default_workspace)
    parser.add_argument(
        "--repos", nargs="+", default=["robot-infra", "cr_meta_lnn"],
        help="Repository directory names relative to --workspace.",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--hash-max-bytes", type=int, default=0,
        help="Hash files no larger than this many bytes; zero disables hashing.",
    )
    parser.add_argument(
        "--fail-on-untracked-source", action="store_true",
        help="Return 2 when non-ignored source or test files are untracked.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)

    summaries: list[dict] = []
    records: list[dict] = []
    for name in args.repos:
        summary, repo_records = inspect_repo(
            (args.workspace / name).resolve(), args.hash_max_bytes
        )
        summaries.append(summary)
        records.extend(repo_records)

    generated_at = datetime.now(timezone.utc).isoformat()
    document = {
        "schema_version": 1,
        "generated_at_utc": generated_at,
        "workspace": str(args.workspace.resolve()),
        "repositories": summaries,
        "files": records,
    }
    (output / "repository_inventory.json").write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    fields = [
        "repository", "path", "category", "tracked", "ignored",
        "git_status", "size_bytes", "sha256",
    ]
    with (output / "repository_inventory.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)

    (output / "repository_summary.json").write_text(
        json.dumps(
            {"schema_version": 1, "generated_at_utc": generated_at,
             "repositories": summaries},
            indent=2,
            sort_keys=True,
        ) + "\n",
        encoding="utf-8",
    )

    print(output / "repository_summary.json")
    print(output / "repository_inventory.csv")
    print(output / "repository_inventory.json")

    if args.fail_on_untracked_source:
        offenders = [
            record for record in records
            if not record["tracked"] and not record["ignored"]
            and record["category"] in {"source", "test_source"}
        ]
        if offenders:
            print(
                f"untracked source gate failed: {len(offenders)} file(s)",
                file=sys.stderr,
            )
            return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
