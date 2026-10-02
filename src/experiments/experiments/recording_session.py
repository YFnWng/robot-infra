"""Filesystem identity and finalized evidence for coordinated recordings."""
from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import re
import sqlite3

import yaml


SESSION_ROOT = "/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def session_token(value: str) -> str:
    """Accept explicit semantic names, not paths or silently rewritten names."""
    if not re.fullmatch(r"[a-z0-9][a-z0-9_-]{0,119}", value):
        raise ValueError("session label must be 1-120 lowercase letters/digits/_/-")
    return value


def allocate_session(root: str | Path, label: str, now=None) -> Path:
    label = session_token(label)
    now = now or datetime.now(timezone.utc)
    if now.tzinfo is None:
        raise ValueError("session time must include a timezone")
    suffix = now.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    parent = Path(root).expanduser().resolve()
    parent.mkdir(parents=True, exist_ok=True)
    session = parent / f"{label}_{suffix}"
    session.mkdir(exist_ok=False)
    return session


def write_manifest(path: Path, document: dict) -> None:
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def live_bag_ready(bag: Path, required_topics: list[str]) -> bool:
    """Confirm SQLite initialization and required recorder subscriptions.

    Humble installations differ in recorder-service availability. Registered
    topics in the owned live storage provide evidence without parsing console
    output; message counts are checked only after the write cache is finalized.
    """
    recorded_topics = set()
    try:
        for database in bag.glob("*.db3"):
            with sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True,
                                 timeout=0.05) as conn:
                recorded_topics.update(row[0] for row in conn.execute(
                    "SELECT name FROM topics"))
    except (OSError, sqlite3.Error):
        return False
    return bool(required_topics) and set(required_topics) <= recorded_topics


def qualify_recording(session: Path, manifest: dict) -> dict:
    """Read finalized storage; do not repair or modify raw recordings."""
    errors = []
    counts = {}
    bag = session / "robot_bag"
    try:
        metadata = yaml.safe_load((bag / "metadata.yaml").read_text())[
            "rosbag2_bagfile_information"]
        if metadata["storage_identifier"] != "sqlite3":
            raise ValueError("expected sqlite3 storage")
        files = metadata["relative_file_paths"]
        if not files:
            raise ValueError("no finalized storage files")
        for relative in files:
            database = (bag / relative).resolve()
            if database.parent != bag.resolve():
                raise ValueError("storage path escapes bag directory")
            with sqlite3.connect(database.as_uri() + "?mode=ro", uri=True) as conn:
                if conn.execute("PRAGMA quick_check").fetchone()[0] != "ok":
                    raise ValueError("sqlite integrity check failed")
                for topic, count in conn.execute(
                        "SELECT topics.name, COUNT(messages.id) FROM topics "
                        "LEFT JOIN messages ON topics.id=messages.topic_id "
                        "GROUP BY topics.id"):
                    counts[topic] = counts.get(topic, 0) + count
        for topic in manifest["required_topics"]:
            if counts.get(topic, 0) == 0:
                errors.append(f"missing recorded messages: {topic}")
    except (OSError, KeyError, TypeError, ValueError, sqlite3.Error,
            yaml.YAMLError) as exc:
        errors.append(f"bag finalization: {exc}")

    if manifest["video_enabled"]:
        video = Path(manifest["video_directory"])
        try:
            camera = json.loads((video / "session_metadata.json").read_text())
            reports = camera["recording_reports"]
            for rig in ("primary", "oblique"):
                if not reports.get(rig):
                    raise ValueError(f"missing finalized report: {rig}")
                report = reports[rig]
                if int(report["indexed_frames"]) <= 0:
                    raise ValueError(f"no indexed frames: {rig}")
                if report.get("playable_frames") == 0:
                    raise ValueError(f"no playable frames: {rig}")
                for name in (f"{rig}_{video.name}.svo2",
                             f"{rig}_frame_index.csv"):
                    if not (video / name).is_file() or (video / name).stat().st_size == 0:
                        raise ValueError(f"missing/empty video artifact: {name}")
            if int(camera["camera_sync"]["paired_frames"]) <= 0:
                raise ValueError("no paired camera frames")
            if not (video / "camera_frame_pairs.csv").is_file():
                raise ValueError("missing frame-pair index")
        except (OSError, KeyError, TypeError, ValueError) as exc:
            errors.append(f"video finalization: {exc}")
    return {"passed": not errors, "errors": errors, "topic_counts": counts}
