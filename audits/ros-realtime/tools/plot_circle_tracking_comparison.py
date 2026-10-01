#!/usr/bin/env python3
"""Compare hardware tip paths using geometric distance to a reference circle."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import sqlite3
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


@dataclass
class Trial:
    label: str
    targets_s: np.ndarray
    targets_mm: np.ndarray
    tip_s: np.ndarray
    tip_mm: np.ndarray
    progress: np.ndarray
    circle_error_mm: np.ndarray


def _percentile(values: np.ndarray, percentile: float) -> float:
    return float(np.percentile(values, percentile))


def _marker_ids(message) -> list[int]:
    for channel in message.channels:
        if channel.name == "marker_id":
            return [int(round(value)) for value in channel.values]
    return []


def _load_bag(path: Path, label: str, circle_start: int) -> Trial:
    connection = sqlite3.connect(path)
    topics = {
        topic_id: (name, get_message(type_name))
        for topic_id, name, type_name in connection.execute(
            "select id,name,type from topics"
        )
    }
    start_ns = connection.execute("select min(timestamp) from messages").fetchone()[0]
    targets: list[tuple[float, list[float]]] = []
    tips: list[tuple[float, list[float]]] = []
    feedback_times: list[float] = []

    selected_topics = {
        "/catheter_mppi/target_tip",
        "/catheter_mppi/track_tip_trajectory/_action/feedback",
        "/shape_tracking/markers",
    }
    for topic_id, stamp_ns, payload in connection.execute(
        "select topic_id,timestamp,data from messages order by timestamp"
    ):
        topic_name, message_type = topics[topic_id]
        if topic_name not in selected_topics:
            continue
        stamp_s = (stamp_ns - start_ns) * 1e-9
        message = deserialize_message(payload, message_type)
        if topic_name == "/catheter_mppi/target_tip":
            targets.append(
                (stamp_s, [message.point.x, message.point.y, message.point.z])
            )
        elif topic_name.endswith("/_action/feedback"):
            feedback_times.append(stamp_s)
        else:
            ids = _marker_ids(message)
            for marker_id, point in zip(ids, message.points):
                if marker_id == 3:
                    tips.append((stamp_s, [point.x, point.y, point.z]))
                    break
    connection.close()

    if len(targets) <= circle_start:
        raise RuntimeError(f"{path}: expected more than {circle_start} targets")
    if not feedback_times:
        raise RuntimeError(f"{path}: trajectory feedback is absent")

    target_s = np.asarray([item[0] for item in targets], dtype=float)
    target_mm = 1000.0 * np.asarray([item[1] for item in targets], dtype=float)
    action_start = target_s[0]
    action_end = max(feedback_times)
    selected_tips = [item for item in tips if action_start <= item[0] <= action_end]
    tip_s = np.asarray([item[0] for item in selected_tips], dtype=float)
    tip_mm = 1000.0 * np.asarray([item[1] for item in selected_tips], dtype=float)

    circle_points = target_mm[circle_start:]
    # The trajectory repeats its initial circle point at the end. Exclude that
    # duplicate when estimating center and radius from the recorded goal.
    unique_circle = circle_points[:-1] if np.allclose(circle_points[0], circle_points[-1]) else circle_points
    center = np.asarray(
        [float(np.mean(unique_circle[:, 0])),
         float(np.mean(unique_circle[:, 1])),
         float(np.mean(unique_circle[:, 2]))]
    )
    radial = np.linalg.norm(unique_circle[:, 1:3] - center[None, 1:3], axis=1)
    radius = float(np.mean(radial))
    plane_error = tip_mm[:, 0] - center[0]
    radial_error = np.linalg.norm(tip_mm[:, 1:3] - center[None, 1:3], axis=1) - radius
    circle_error = np.sqrt(plane_error * plane_error + radial_error * radial_error)

    progress = np.empty(len(tip_s), dtype=float)
    for sample_index, stamp_s in enumerate(tip_s):
        target_index = bisect.bisect_right(target_s.tolist(), stamp_s) - 1
        target_index = max(circle_start, min(target_index, len(target_s) - 1))
        if target_index + 1 < len(target_s):
            interval = target_s[target_index + 1] - target_s[target_index]
            fraction = 0.0 if interval <= 0.0 else (stamp_s - target_s[target_index]) / interval
            fraction = min(max(fraction, 0.0), 1.0)
        else:
            fraction = 0.0
        progress[sample_index] = target_index - circle_start + fraction

    circle_mask = tip_s >= target_s[circle_start]
    return Trial(
        label=label,
        targets_s=target_s,
        targets_mm=target_mm,
        tip_s=tip_s,
        tip_mm=tip_mm,
        progress=progress[circle_mask],
        circle_error_mm=circle_error[circle_mask],
    )


def _rolling_median(values: np.ndarray, window: int = 15) -> np.ndarray:
    result = np.empty_like(values)
    half = window // 2
    for index in range(len(values)):
        result[index] = np.median(values[max(0, index-half):min(len(values), index+half+1)])
    return result


def _equal_3d_limits(axes, trials: list[Trial]) -> None:
    all_points = np.concatenate(
        [np.concatenate([trial.targets_mm, trial.tip_mm], axis=0) for trial in trials],
        axis=0,
    )
    low = np.min(all_points, axis=0)
    high = np.max(all_points, axis=0)
    center = 0.5 * (low + high)
    radius = 0.52 * float(np.max(high - low))
    for axis in axes:
        axis.set_xlim(center[0]-radius, center[0]+radius)
        axis.set_ylim(center[1]-radius, center[1]+radius)
        axis.set_zlim(center[2]-radius, center[2]+radius)
        axis.set_box_aspect((1.0, 1.0, 1.0))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--previous-bag", type=Path, required=True)
    parser.add_argument("--current-bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metrics", type=Path)
    parser.add_argument("--circle-start", type=int, default=5)
    arguments = parser.parse_args()

    trials = [
        _load_bag(arguments.previous_bag, "No backlash compensation (fixed J)", arguments.circle_start),
        _load_bag(arguments.current_bag, "Backlash compensation (fixed J)", arguments.circle_start),
    ]
    if not np.allclose(trials[0].targets_mm, trials[1].targets_mm):
        raise RuntimeError("the two bags do not contain the same target trajectory")

    figure = plt.figure(figsize=(14.0, 10.0), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, height_ratios=(1.25, 0.85))
    path_axes = [figure.add_subplot(grid[0, index], projection="3d") for index in range(2)]
    error_axes = [figure.add_subplot(grid[1, index]) for index in range(2)]
    colors = ("#2b6cb0", "#c05621")

    for column, (trial, color) in enumerate(zip(trials, colors)):
        axis = path_axes[column]
        target = trial.targets_mm
        axis.plot(target[:arguments.circle_start+1, 0], target[:arguments.circle_start+1, 1],
                  target[:arguments.circle_start+1, 2], color="0.35", linestyle="--",
                  linewidth=1.3, label="target approach")
        axis.plot(target[arguments.circle_start:, 0], target[arguments.circle_start:, 1],
                  target[arguments.circle_start:, 2], color="black", linewidth=2.0,
                  label="target circle")
        axis.plot(trial.tip_mm[:, 0], trial.tip_mm[:, 1], trial.tip_mm[:, 2],
                  color=color, linewidth=1.4, alpha=0.9, label="measured tip")
        axis.scatter(*trial.tip_mm[0], color=color, marker="o", s=32, label="measured start")
        axis.scatter(*trial.tip_mm[-1], color=color, marker="x", s=42, label="measured end")
        axis.set_title(trial.label)
        axis.set_xlabel("base x [mm]")
        axis.set_ylabel("base y [mm]")
        axis.set_zlabel("base z [mm]")
        axis.legend(loc="upper left", fontsize=8)
        axis.view_init(elev=24, azim=-52)

        error_axis = error_axes[column]
        error_axis.plot(trial.progress, trial.circle_error_mm, color=color,
                        alpha=0.28, linewidth=0.8, label="per-frame error")
        error_axis.plot(trial.progress, _rolling_median(trial.circle_error_mm),
                        color=color, linewidth=2.0, label="15-frame median")
        error_axis.axhline(1.8, color="black", linestyle="--", linewidth=1.1,
                           label="1.8 mm tolerance")
        error_axis.set_xlim(0.0, len(target)-arguments.circle_start-1)
        error_axis.set_ylim(bottom=0.0)
        error_axis.set_xlabel("circle waypoint progress")
        error_axis.set_ylabel("distance to closest circle point [mm]")
        error_axis.grid(alpha=0.25)
        error_axis.legend(loc="upper right", fontsize=8)

    _equal_3d_limits(path_axes, trials)
    figure.suptitle(
        "Real-hardware circle tracking: geometric path error\n"
        "Error is Euclidean distance to the closest point on the continuous target circle",
        fontsize=14,
    )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(arguments.output, dpi=220)
    plt.close(figure)

    metrics_path = arguments.metrics or arguments.output.with_suffix(".csv")
    with metrics_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["trial", "samples", "mean_mm", "rms_mm", "median_mm", "p95_mm", "max_mm"])
        for trial in trials:
            values = trial.circle_error_mm
            writer.writerow([
                trial.label,
                len(values),
                float(np.mean(values)),
                math.sqrt(float(np.mean(values * values))),
                _percentile(values, 50),
                _percentile(values, 95),
                float(np.max(values)),
            ])
    print(arguments.output)
    print(metrics_path)


if __name__ == "__main__":
    main()
