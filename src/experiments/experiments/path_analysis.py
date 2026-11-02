"""Regenerate continuous-path reports from finalized coordinated sessions."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import textwrap

from .recording_session import qualify_recording, write_manifest
from .path_session import validate_acquisition


def plot_path_3d(output, marker_samples, target_knots_m, *, title="Path tracking"):
    """Plot raw tip observations in metres, without smoothing or bridging gaps."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from .reaching_bag import MAX_GAP_NS

    target = np.asarray(target_knots_m, dtype=float)
    samples = sorted(marker_samples, key=lambda sample: sample[0])
    tip = np.asarray([value for _, value in samples], dtype=float)
    if (target.ndim != 2 or target.shape[1] != 3 or len(target) < 2
            or not np.isfinite(target).all()):
        raise ValueError("target path must contain finite XYZ knots")
    if tip.shape != (len(samples), 3) or not len(samples) or not np.isfinite(tip).all():
        raise ValueError("recorded tip path must contain finite XYZ samples")
    target, tip = target * 1000, tip * 1000
    figure = plt.figure(figsize=(9, 8))
    axes = figure.add_subplot(111, projection="3d")
    axes.plot(*target.T, "k--", linewidth=2, label="Frozen target path")
    breaks = np.flatnonzero(np.diff([stamp for stamp, _ in samples]) > MAX_GAP_NS) + 1
    for index, segment in enumerate(np.split(tip, breaks)):
        axes.plot(*segment.T, color="C0", linewidth=1, marker=".", markersize=2,
                  label="Recorded raw tip" if index == 0 else None)
    axes.scatter(*tip[0], color="green", marker="o", label="Tip start")
    axes.scatter(*tip[-1], color="red", marker="x", label="Tip end")
    # Equal spatial scale: do not exaggerate small out-of-plane deviations.
    points = np.concatenate((target, tip))
    center = (points.min(axis=0) + points.max(axis=0)) / 2
    radius = max(float(np.ptp(points, axis=0).max()) / 2 * 1.05, .5)
    for setter, coordinate in zip((axes.set_xlim, axes.set_ylim, axes.set_zlim), center):
        setter(coordinate - radius, coordinate + radius)
    axes.set_box_aspect((1, 1, 1))
    axes.set(xlabel="X (mm)", ylabel="Y (mm)", zlabel="Z (mm)", title=title)
    axes.legend()
    figure.tight_layout()
    try:
        figure.savefig(output, dpi=180)
    finally:
        plt.close(figure)
    return Path(output)


def path_metrics(result):
    feedback = result.get("feedback", [])
    state_seconds = {}
    for previous, current in zip(feedback, feedback[1:]):
        dt = current["elapsed_s"]-previous["elapsed_s"]
        # Do not invent occupancy across missing feedback spans.
        if 0 <= dt <= .25:
            state = previous["governor_state"]
            state_seconds[state] = state_seconds.get(state, 0.0)+dt
    return {
        "success": result.get("success", False),
        "position_objective": ("global_xz" if result.get("xz_only", False) else
                               "global_x_only" if result.get("x_only", False) else "xyz"),
        "message": result.get("message", "missing result"),
        "time_to_result_s": (
            (result["ended_monotonic_ns"]-result["started_monotonic_ns"])/1e9
            if "started_monotonic_ns" in result else None),
        "final_error_mm": result.get("final_error_mm"),
        "rms_closest_path_error_mm": result.get("rms_closest_path_error_mm"),
        "p95_closest_path_error_mm": result.get("p95_closest_path_error_mm"),
        # Legacy actions populated completed_arc_length_m from the reference
        # clock. Never relabel those historical values as measured coverage.
        "completed_arc_length_m": (result.get("completed_arc_length_m")
                                   if "path_coverage" in result else None),
        "reference_arc_length_m": result.get(
            "reference_arc_length_m", result.get("completed_arc_length_m")),
        "path_coverage": result.get("path_coverage"),
        "traversal_complete": result.get("traversal_complete"),
        "last_progress": feedback[-1]["progress"] if feedback else None,
        "feedback_samples": len(feedback),
        "governor_state_seconds": state_seconds,
    }


def analyze(session):
    session = Path(session).resolve()
    manifest = json.loads((session / "session_manifest.json").read_text())
    if manifest.get("state") not in {"complete", "partial", "failed"}:
        raise ValueError("stop coordinated recording before offline analysis")
    protocol = json.loads((session / "task_manifest.json").read_text())
    if hashlib.sha256((session / "task_path.yaml").read_bytes()).hexdigest() != protocol["path_sha256"]:
        raise ValueError("path snapshot no longer matches task manifest")
    source = session / "path_result.json"
    # Abrupt process death can leave an incomplete exclusive result file.
    try:
        result = json.loads(source.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        result = {"success": False, "message": "missing/incomplete task result"}
    metrics = path_metrics(result)
    acquisition = protocol.get("initial_acquisition")
    acquisition_result = {"required": acquisition is not None}
    if acquisition is not None:
        try:
            validate_acquisition(session / acquisition["journal"], acquisition["target_m"])
            acquisition_result.update(passed=True, reason="start acquired")
        except (OSError, ValueError, RuntimeError) as error:
            acquisition_result.update(passed=False, reason=str(error))
        acquisition_result.update(target_m=acquisition["target_m"],
                                  journal=acquisition["journal"])
    output = session / "analysis"
    output.mkdir(exist_ok=True)
    write_manifest(output / "path_metrics.json", {"metric_version": 2, **metrics})
    write_manifest(output / "acquisition_result.json", acquisition_result)
    qualification = qualify_recording(session, manifest)
    write_manifest(output / "integrity.json", {
        "recording": qualification,
        "limitations": [
            ("Scalar global-X task: action error/coverage/settling ignore Y/Z; the raw 3D plot records their drift."
             if result.get("x_only", False) else
             "Global X/Z task: error/coverage/settling ignore Y; raw 3D plots retain Y drift."
             if result.get("xz_only", False) else "XYZ positional task."),
            "Action metrics use repeated governor samples of latest measured markers.",
            ("Acquisition is separate from the unchanged frozen tracking knots; see acquisition_result.json."
             if acquisition is not None else
             "Legacy approach and circle are included together; submitted knots are in path_result.json."),
            "This report does not independently source-time align the bag or infer missing metrics."]})
    feedback = result.get("feedback", [])
    with (output / "path_feedback.csv").open("w", newline="") as stream:
        if feedback:
            writer = csv.DictWriter(stream, fieldnames=list(feedback[0]))
            writer.writeheader()
            writer.writerows(feedback)
    lines = ["# Continuous path report", "", f"Session: `{session.name}`", "",
             f"Recording passed: {qualification['passed']}", "",
             "| Metric | Value |", "| --- | --- |"]
    lines.extend(f"| {key} | {value} |" for key, value in metrics.items())
    lines.extend(["", "Action metrics are governor-sampled measured-marker errors, not independent camera frames.",
                  "Missing/faulted results are retained. Legacy reference progress is not measured coverage.",
                  "", "[Feedback](path_feedback.csv) · [Integrity](integrity.json) · [Acquisition](acquisition_result.json)"])
    if feedback:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, sharex=True)
        times = [row["elapsed_s"] for row in feedback]
        for key in ("reference_error_mm", "closest_path_error_mm"):
            axes[0].plot(times, [row[key] for row in feedback], label=key)
        axes[0].set_ylabel("Error (mm)")
        axes[0].legend()
        axes[1].plot(times, [row["progress"] for row in feedback])
        axes[1].lines[-1].set_label("Reference progress")
        if all("path_coverage" in row for row in feedback):
            axes[1].plot(times, [row["path_coverage"] for row in feedback],
                         label="Measured ordered coverage")
        axes[1].legend()
        axes[1].set_ylabel("Fraction")
        axes[1].set_xlabel("Elapsed (s)")
        fig.tight_layout()
        fig.savefig(output / "path_tracking.png")
        plt.close(fig)
        lines.extend(["", "![Tracking](path_tracking.png)"])
    if all(key in result for key in ("started_ros_ns", "ended_ros_ns", "path_knots_m")):
        import yaml
        from .reaching_bag import read_streams
        config = yaml.safe_load((session / "task_path.yaml").read_text())
        streams, issues = read_streams(session / "robot_bag", config)
        samples = [(stamp, tip) for stamp, tip in streams["markers"]
                   if result["started_ros_ns"] <= stamp <= result["ended_ros_ns"]]
        write_manifest(output / "path_3d_evidence.json", {
            "marker_samples": len(samples), "bag_analysis_issues": issues,
            "start_ros_ns": result["started_ros_ns"], "end_ros_ns": result["ended_ros_ns"],
            "limitations": ["Raw marker source timestamps restricted to the action window; acquisition excluded.",
                            "No smoothing; gaps over 250ms are not connected."]})
        if samples:
            plot_path_3d(output / "path_3d.png", samples, result["path_knots_m"],
                         title="\n".join(textwrap.wrap(session.name.replace("_", " "), 65)))
            lines.extend(["", "![Target and recorded tip in 3D](path_3d.png)",
                          "Raw marker observations during tracking only, with equal XYZ scale (mm).",
                          "[3D plot evidence](path_3d_evidence.json)"])
    (output / "report.md").write_text("\n".join(lines)+"\n")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    print(analyze(parser.parse_args().session))
