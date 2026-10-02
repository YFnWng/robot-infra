"""Offline, journal-based reaching metrics; never substitutes UKF for markers."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import yaml

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


def analyze(session, bag_metrics=False):
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
    bag_issues = []
    if bag_metrics:
        from .reaching_bag import read_streams, window_metrics, write_trial_plot
        config = yaml.safe_load((session / "task_targets.yaml").read_text())
        streams, bag_issues = read_streams(session / "robot_bag", config)
        for row, target in zip(rows, protocol["targets_m"]):
            starts = [record for record in records if record.get("trial") == row["trial"]
                      and record["event"] == "trial_started"]
            if not starts:
                continue
            start = starts[0]["ros_time_ns"]
            ends = [record for record in records if record.get("trial") == row["trial"]
                    and record["event"] == "trial_completed"]
            terminal = [record for record in records if record["event"] in
                        {"task_failed", "task_interrupted"} and record["ros_time_ns"] >= start]
            if not ends and not terminal:
                bag_issues.append(f"trial {row['trial']}: no terminal timestamp; not analyzed")
                continue
            end = (ends or terminal)[0]["ros_time_ns"]
            metrics, windows = window_metrics(streams, start, end, target)
            row.update(metrics)
            write_trial_plot(output / f"trial_{row['trial']:02d}.png", windows, target, start)
    write_manifest(output / "trial_metrics.json", {"metric_version": 2, "trials": rows})
    write_manifest(output / "integrity.json", {
        "recording": qualification,
        "task_completed": any(row["event"] == "task_completed" for row in records),
        "task_failures": [row for row in records if row["event"] in
                          {"task_failed", "task_interrupted"}],
        "bag_analysis_issues": bag_issues,
        "bag_metrics_requested": bag_metrics,
        "limitations": [
            "Journal tip is the latest received marker, not source-time aligned.",
            "Bag metrics use source timestamps with a 250ms maximum gap; "
            "observed maximum error is not an upper bound across missing spans.",
            "Motor travel excludes gaps; command reversals use transmitted VEL only.",
            "Encoder reversal deadband is 2 raw counts; velocity deadband is "
            "0.02mm/s, 0.2deg/s, 0.02mm/s; two directional samples confirm a reversal."],
    })
    with (output / "trial_metrics.csv").open("w", newline="") as stream:
        fields = list(dict.fromkeys(key for row in rows for key in row))
        writer = csv.DictWriter(stream, fieldnames=fields)
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
                  "Source-time bag metrics and coverage flags are in the CSV/JSON when requested.", "",
                  "[Metrics](trial_metrics.csv) · [Integrity](integrity.json)", ""])
    if bag_metrics:
        lines.extend([f"![Trial {row['trial']}](trial_{row['trial']:02d}.png)" for row in rows
                      if (output / f"trial_{row['trial']:02d}.png").exists()])
    (output / "report.md").write_text("\n".join(lines))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--bag-metrics", action="store_true",
                        help="Decode finalized bag and generate source-time metrics/plots")
    args = parser.parse_args()
    print(analyze(args.session, bag_metrics=args.bag_metrics))


if __name__ == "__main__":
    main()
