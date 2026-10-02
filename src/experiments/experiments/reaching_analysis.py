"""Offline, journal-based reaching metrics; never substitutes UKF for markers."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from .recording_session import qualify_recording, write_manifest


def trial_metrics(records, targets):
    rows = []
    for index, target in enumerate(targets, 1):
        events = [record for record in records if record.get("trial") == index]
        starts = [record for record in events if record["event"] == "trial_started"]
        ends = [record for record in events if record["event"] == "trial_completed"]
        if len(starts) > 1 or len(ends) > 1:
            raise ValueError(f"duplicate trial boundaries: {index}")
        start = starts[0] if starts else None
        end = ends[0] if ends else None
        if end and not start:
            raise ValueError(f"completion without start: {index}")
        duration = None
        error = None
        if end:
            duration = (end["monotonic_ns"] - start["monotonic_ns"]) / 1e9
            if duration < 0:
                raise ValueError("non-monotonic trial boundaries")
            tip = end.get("measured_tip_m")
            if tip is not None and len(tip) == 3 and all(math.isfinite(v) for v in tip):
                error = 1000 * math.sqrt(sum((a - b) ** 2 for a, b in zip(tip, target)))
        rows.append({
            "trial": index,
            "outcome": ("reached" if end["reached"] else "timed_out"
                        if end["timed_out"] else "completed_without_success") if end
                       else "interrupted_or_faulted" if start else "not_attempted",
            "time_to_result_s": duration,
            "measured_final_error_mm": error,
            "controller_final_error_mm": end.get("controller_final_error_mm") if end else None,
            "command_reversals": None, "encoder_reversals": None,
            "maximum_error_mm": None,
        })
    return rows


def analyze(session):
    session = Path(session).resolve()
    manifest = json.loads((session / "session_manifest.json").read_text())
    if manifest.get("state") not in {"complete", "partial", "failed"}:
        raise ValueError("stop coordinated recording before offline analysis")
    protocol = json.loads((session / "task_manifest.json").read_text())
    target_hash = hashlib.sha256((session / "task_targets.yaml").read_bytes()).hexdigest()
    if target_hash != protocol["target_sha256"]:
        raise ValueError("target snapshot no longer matches task manifest")
    records = [json.loads(line) for line in (session / "trials.jsonl").read_text().splitlines()]
    rows = trial_metrics(records, protocol["targets_m"])
    qualification = qualify_recording(session, manifest)
    output = session / "analysis"
    output.mkdir(exist_ok=True)
    write_manifest(output / "trial_metrics.json", {"metric_version": 1, "trials": rows})
    write_manifest(output / "integrity.json", {
        "recording": qualification,
        "task_completed": any(row["event"] == "task_completed" for row in records),
        "task_failures": [row for row in records if row["event"] in
                          {"task_failed", "task_interrupted"}],
        "limitations": [
            "Journal tip is the latest received marker, not source-time aligned.",
            "Reversals, maximum error and stream coverage require bag analysis; "
            "unavailable, not zero."],
    })
    with (output / "trial_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else [])
        writer.writeheader()
        writer.writerows(rows)
    lines = ["# Reaching session report", "", f"Session: `{session.name}`", "",
             f"Recording qualification passed: {qualification['passed']}", "",
             "| Trial | Outcome | Time to result (s) | Latest marker error (mm) |",
             "| --- | --- | --- | --- |"]
    for row in rows:
        duration = row["time_to_result_s"]
        error = row["measured_final_error_mm"]
        lines.append(f"| {row['trial']} | {row['outcome']} | "
                     f"{duration if duration is not None else 'unavailable'} | "
                     f"{error if error is not None else 'unavailable'} |")
    lines.extend(["", "Times exclude homing and include action admission/settling.",
                  "Latest marker errors are journal snapshots, not synchronized bag metrics.",
                  "Unattempted and interrupted trials are retained, not treated as successes.",
                  "Reversal counts and maximum errors are unavailable at this gate.", "",
                  "[Metrics](trial_metrics.csv) · [Integrity](integrity.json)", ""])
    (output / "report.md").write_text("\n".join(lines))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    args = parser.parse_args()
    print(analyze(args.session))


if __name__ == "__main__":
    main()
