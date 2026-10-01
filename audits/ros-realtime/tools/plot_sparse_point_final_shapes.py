#!/usr/bin/env python3
"""Plot target and reconstructed final shapes for a sparse-point ROS bag.

The bag records the UKF interface pose and distal strain, rather than a dense
centerline.  This tool reconstructs the centerline with the same frozen v171
kinematics used by the live camera overlay.  For timed-out targets it also
plots the shape at closest approach, so a failure to settle is not mistaken
for geometric unreachability.
"""

from __future__ import annotations

import argparse
import csv
import sqlite3
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import torch


TARGET_TOPIC = "/catheter_mppi/target_tip"
TRACE_TOPIC = "/catheter_mppi/estimator_trace"
FEEDBACK_TOPIC = "/catheter_mppi/track_tip_trajectory/_action/feedback"
MARKER_TOPIC = "/shape_tracking/markers"


@dataclass
class Sample:
    timestamp_ns: int
    observed_tip_mm: np.ndarray
    estimated_tip_mm: np.ndarray
    pose: np.ndarray
    strain: np.ndarray
    markers_mm: np.ndarray | None = None


@dataclass
class PointResult:
    index: int
    target_mm: np.ndarray
    reached: bool
    final_error_mm: float
    minimum_error_mm: float
    home: Sample
    final: Sample
    closest: Sample


def _load_model(checkpoint: Path):
    from cr_meta_lnn.networks.hybrid.distal_first_order import (
        build_first_order_distal_nominal,
    )

    saved = torch.load(checkpoint, map_location="cpu", weights_only=False)
    if saved.get("experiment") != "real_distal_first_order_standalone_em":
        raise ValueError(f"unexpected checkpoint schema: {saved.get('experiment')}")
    model = build_first_order_distal_nominal(saved["config"]["model"])
    model.load_state_dict(saved["model"], strict=True)
    return model.to(device="cpu", dtype=torch.float32).eval().requires_grad_(False)


def _marker_ids(message) -> list[int]:
    for channel in message.channels:
        if channel.name == "marker_id":
            return [int(round(value)) for value in channel.values]
    return list(range(len(message.points)))


def _nearest_index(timestamps: np.ndarray, query_ns: int) -> int:
    insertion = int(np.searchsorted(timestamps, query_ns))
    candidates = [max(0, min(len(timestamps) - 1, insertion))]
    if insertion > 0:
        candidates.append(insertion - 1)
    return min(candidates, key=lambda index: abs(int(timestamps[index]) - query_ns))


def _read_bag(path: Path) -> tuple[list[tuple[int, np.ndarray]], list[Sample], list[tuple[int, object]], list[tuple[int, np.ndarray]]]:
    connection = sqlite3.connect(path)
    topics = {
        topic_id: (name, get_message(type_name))
        for topic_id, name, type_name in connection.execute(
            "select id,name,type from topics"
        )
    }
    targets: list[tuple[int, np.ndarray]] = []
    traces: list[Sample] = []
    feedback: list[tuple[int, object]] = []
    markers: list[tuple[int, np.ndarray]] = []
    selected = {TARGET_TOPIC, TRACE_TOPIC, FEEDBACK_TOPIC, MARKER_TOPIC}
    for topic_id, timestamp_ns, payload in connection.execute(
        "select topic_id,timestamp,data from messages order by timestamp"
    ):
        name, message_type = topics[topic_id]
        if name not in selected:
            continue
        message = deserialize_message(payload, message_type)
        if name == TARGET_TOPIC:
            targets.append((timestamp_ns, 1000.0 * np.asarray([
                message.point.x, message.point.y, message.point.z], dtype=float)))
        elif name == TRACE_TOPIC:
            traces.append(Sample(
                timestamp_ns=timestamp_ns,
                observed_tip_mm=1000.0 * np.asarray(message.observed_tip_m, dtype=float),
                estimated_tip_mm=1000.0 * np.asarray(message.estimated_tip_m, dtype=float),
                pose=np.asarray(message.interface_pose, dtype=float).reshape(4, 4),
                strain=np.asarray(message.distal_strain, dtype=float),
            ))
        elif name == FEEDBACK_TOPIC:
            feedback.append((timestamp_ns, message.feedback))
        else:
            ids = _marker_ids(message)
            ordered = sorted(zip(ids, message.points), key=lambda pair: pair[0])
            markers.append((timestamp_ns, 1000.0 * np.asarray(
                [[point.x, point.y, point.z] for _, point in ordered], dtype=float)))
    connection.close()
    if not targets or not traces or not feedback:
        raise RuntimeError("bag lacks targets, estimator traces, or action feedback")
    return targets, traces, feedback, markers


def _associate_results(targets, traces, feedback, markers) -> list[PointResult]:
    trace_ns = np.asarray([sample.timestamp_ns for sample in traces], dtype=np.int64)
    marker_ns = np.asarray([item[0] for item in markers], dtype=np.int64)
    bag_end_ns = max(trace_ns[-1], feedback[-1][0]) + 1
    results: list[PointResult] = []
    for index, (start_ns, target_mm) in enumerate(targets):
        end_ns = targets[index + 1][0] if index + 1 < len(targets) else bag_end_ns
        interval_feedback = [item for item in feedback if start_ns <= item[0] < end_ns]
        interval_trace_indices = np.flatnonzero((trace_ns >= start_ns) & (trace_ns < end_ns))
        if not interval_feedback or not len(interval_trace_indices):
            raise RuntimeError(f"point {index + 1} has incomplete recorded streams")

        final_feedback_ns, final_feedback = interval_feedback[-1]
        interval_trace_indices = interval_trace_indices[
            trace_ns[interval_trace_indices] <= final_feedback_ns
        ]
        if not len(interval_trace_indices):
            raise RuntimeError(f"point {index + 1} lacks estimator traces before action completion")
        home_candidates = np.flatnonzero(trace_ns <= start_ns)
        home_index = (
            int(home_candidates[-1]) if len(home_candidates)
            else _nearest_index(trace_ns, start_ns)
        )
        final_index = int(interval_trace_indices[-1])
        interval_errors = np.asarray([
            np.linalg.norm(traces[i].observed_tip_mm - target_mm)
            for i in interval_trace_indices
        ])
        closest_index = int(interval_trace_indices[int(np.argmin(interval_errors))])
        home_sample = traces[home_index]
        final_sample = traces[final_index]
        closest_sample = traces[closest_index]
        for sample in (home_sample, final_sample, closest_sample):
            if len(marker_ns):
                marker_index = _nearest_index(marker_ns, sample.timestamp_ns)
                sample.markers_mm = markers[marker_index][1]
        results.append(PointResult(
            index=index + 1,
            target_mm=target_mm,
            reached=bool(final_feedback.within_tolerance),
            final_error_mm=float(np.linalg.norm(final_sample.observed_tip_mm - target_mm)),
            minimum_error_mm=float(interval_errors.min()),
            home=home_sample,
            final=final_sample,
            closest=closest_sample,
        ))
    return results


def _centerline(model, sample: Sample) -> np.ndarray:
    with torch.inference_mode():
        points = model.points(
            torch.as_tensor(sample.pose, dtype=torch.float32),
            torch.as_tensor(sample.strain, dtype=torch.float32),
        )
    return 1000.0 * points.detach().cpu().numpy()


def _set_common_limits(axes, point_cloud: np.ndarray) -> None:
    low = point_cloud.min(axis=0)
    high = point_cloud.max(axis=0)
    center = 0.5 * (low + high)
    radius = max(2.0, 0.56 * float(np.max(high - low)))
    for axis in axes:
        axis.set_xlim(center[0] - radius, center[0] + radius)
        axis.set_ylim(center[1] - radius, center[1] + radius)
        axis.set_zlim(center[2] - radius, center[2] + radius)
        axis.set_box_aspect((1, 1, 1))
        axis.set_xlabel("x (mm)")
        axis.set_ylabel("y (mm)")
        axis.set_zlabel("z (mm)")
        axis.view_init(elev=22, azim=-58)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--cr-meta-root", type=Path, required=True)
    parser.add_argument("--cr-common-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary-csv", type=Path)
    arguments = parser.parse_args()

    for source_root in (arguments.cr_meta_root.parent, arguments.cr_common_root):
        if str(source_root) not in sys.path:
            sys.path.insert(0, str(source_root))

    targets, traces, feedback, markers = _read_bag(arguments.bag.resolve())
    results = _associate_results(targets, traces, feedback, markers)
    model = _load_model(arguments.checkpoint.resolve())

    figure = plt.figure(figsize=(14.5, 11.5), constrained_layout=True)
    axes = [figure.add_subplot(2, 2, index, projection="3d") for index in range(1, 5)]
    all_geometry: list[np.ndarray] = []
    colors = ("#2563eb", "#059669", "#d97706", "#dc2626")
    for axis, result, color in zip(axes, results, colors):
        home_shape = _centerline(model, result.home)
        final_shape = _centerline(model, result.final)
        closest_shape = _centerline(model, result.closest)
        all_geometry.extend([
            home_shape, final_shape, closest_shape, result.target_mm[None, :]
        ])

        axis.plot(*home_shape.T, color="#7c3aed", linewidth=2.1,
                  linestyle=":", label="homed UKF shape")
        axis.scatter(*result.home.observed_tip_mm, color="#7c3aed", s=42,
                     marker="^", label="homed observed tip")
        if not result.reached:
            axis.plot(*closest_shape.T, color="#64748b", linewidth=2.0,
                      linestyle="--", label="closest-approach shape")
            axis.scatter(*result.closest.observed_tip_mm, color="#64748b", s=36,
                         marker="o", label="closest observed tip")
        axis.plot(*final_shape.T, color=color, linewidth=3.0,
                  label="final UKF shape")
        axis.scatter(*result.final.pose[:3, 3] * 1000.0, color=color, s=35,
                     marker="s", label="interface")
        axis.scatter(*result.final.observed_tip_mm, color="black", s=48,
                     marker="o", label="final observed tip")
        axis.scatter(*result.target_mm, color="#e11d48", s=130,
                     marker="*", label="target")
        if result.final.markers_mm is not None:
            axis.scatter(*result.final.markers_mm.T, color="black", s=22,
                         marker="x", linewidths=1.0, label="final observed markers")

        outcome = "reached" if result.reached else "timed out"
        axis.set_title(
            f"Point {result.index}: {outcome}\n"
            f"final error {result.final_error_mm:.2f} mm; "
            f"minimum {result.minimum_error_mm:.2f} mm"
        )
        axis.grid(alpha=0.3)

    _set_common_limits(axes, np.concatenate(all_geometry, axis=0))
    handles, labels = axes[2].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    figure.legend(unique.values(), unique.keys(), loc="outside lower center",
                  ncol=4, frameon=False)
    figure.suptitle(
        "Sparse-point outcomes: recorded UKF state reconstructed with v171 kinematics\n"
        "Dashed gray = closest approach for timed-out points",
        fontsize=15,
    )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(arguments.output, dpi=180)
    plt.close(figure)

    if arguments.summary_csv is not None:
        arguments.summary_csv.parent.mkdir(parents=True, exist_ok=True)
        with arguments.summary_csv.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "point", "outcome", "target_x_mm", "target_y_mm", "target_z_mm",
                "home_tip_x_mm", "home_tip_y_mm", "home_tip_z_mm",
                "final_tip_x_mm", "final_tip_y_mm", "final_tip_z_mm",
                "final_error_mm", "minimum_error_mm",
            ])
            for result in results:
                writer.writerow([
                    result.index, "reached" if result.reached else "timed_out",
                    *result.target_mm.tolist(), *result.home.observed_tip_mm.tolist(),
                    *result.final.observed_tip_mm.tolist(),
                    result.final_error_mm, result.minimum_error_mm,
                ])

    for result in results:
        print(
            f"point {result.index}: {'reached' if result.reached else 'timed out'}, "
            f"final={result.final_error_mm:.3f} mm, min={result.minimum_error_mm:.3f} mm"
        )


if __name__ == "__main__":
    main()
