#!/usr/bin/env python3
"""Plot Phase-2e distal curves at static tendon states and all insertions."""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import h5py
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import evaluate_causal_proximal_experiment as causal  # noqa: E402


EPISODE_RE = re.compile(
    r"compensated_bend_insertion_p(?P<pass>[12])_level_"
    r"(?P<level>[0-9.]+)_(?P<speed>slow|fast)_"
    r"(?P<branch>high_first|low_first)")
TARGETS_MM = (0.0, 7.5, 15.0)
COLORS = {0.0: "#277DA1", 7.5: "#F8961E", 15.0: "#D62828"}


def _equal_3d_limits(axes, curves, relative):
    cloud = []
    for curve in curves:
        value = curve - curve[0] if relative else curve
        cloud.append(value)
    cloud = np.concatenate(cloud, axis=0)
    low = np.nanmin(cloud, axis=0)
    high = np.nanmax(cloud, axis=0)
    center = 0.5 * (low + high)
    half = 0.52 * float(np.max(high - low))
    for ax in axes:
        ax.set_xlim(center[0] - half, center[0] + half)
        ax.set_ylim(center[1] - half, center[1] + half)
        ax.set_zlim(center[2] - half, center[2] + half)
        ax.set_box_aspect((1, 1, 1))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--shape-h5", type=Path, required=True)
    parser.add_argument("--robot-session", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--position-tolerance-mm", type=float, default=0.25)
    parser.add_argument("--static-velocity-tolerance", type=float, default=1e-6)
    args = parser.parse_args()

    _, messages = causal._read(args.robot_session)
    traces = causal._episode_rows(messages)
    trace_t = np.asarray([row["timestamp_ns"] for row in traces], dtype=np.int64)
    with h5py.File(args.shape_h5, "r") as h5:
        t = h5["frames/timestamp_ns"][:]
        learning = h5["frames/learning_valid"][:].astype(bool)
        points = h5["distal/points_base_mm"][:]

    nearest = np.searchsorted(trace_t, t)
    nearest = np.clip(nearest, 1, len(trace_t) - 1)
    left = nearest - 1
    use_left = np.abs(t - trace_t[left]) <= np.abs(trace_t[nearest] - t)
    nearest[use_left] = left[use_left]
    alignment_ms = np.abs(t - trace_t[nearest]) / 1e6
    names = np.asarray([traces[index]["name"] for index in nearest], dtype=object)
    positions = np.asarray([traces[index]["position"] for index in nearest])
    commands = np.asarray([traces[index]["commanded"] for index in nearest])

    selected = {}
    metadata = {"selection": {}, "coordinate_frame": "robot_base"}
    for name in dict.fromkeys(names.tolist()):
        match = EPISODE_RE.fullmatch(str(name))
        if not match:
            continue
        level = float(match.group("level"))
        visit = int(match.group("pass"))
        episode = (names == name) & learning & (alignment_ms <= 20.0)
        for target in TARGETS_MM:
            mask = (episode
                    & (np.abs(positions[:, 2] - target)
                       <= args.position_tolerance_mm)
                    & (np.abs(commands[:, 2])
                       <= args.static_velocity_tolerance))
            indices = np.flatnonzero(mask)
            if not indices.size:
                raise RuntimeError(
                    f"no static frames for {name}, tendon={target:g} mm")
            curves = points[indices]
            key = (level, target)
            selected.setdefault(key, []).append({
                "visit": visit,
                "episode": name,
                "curves": curves,
                "median": np.nanmedian(curves, axis=0),
            })
            metadata["selection"][f"{level:g}mm_tendon_{target:g}mm_visit_{visit}"] = {
                "frames": int(indices.size),
                "measured_tendon_mm_median": float(
                    np.median(positions[indices, 2])),
                "measured_insertion_mm_median": float(
                    np.median(positions[indices, 0])),
                "trace_alignment_ms_p95": float(
                    np.percentile(alignment_ms[indices], 95)),
            }

    cartesian_rows = []
    for level in sorted({key[0] for key in selected}):
        by_visit = {
            target: {record["visit"]: record for record in selected[(level, target)]}
            for target in TARGETS_MM
        }
        for visit in sorted(set(by_visit[0.0]) & set(by_visit[15.0])):
            low = by_visit[0.0][visit]["median"]
            high = by_visit[15.0][visit]["median"]
            low = low - low[0]
            high = high - high[0]
            delta = high - low
            low_tangent = np.nanmean(np.diff(low[-8:], axis=0), axis=0)
            high_tangent = np.nanmean(np.diff(high[-8:], axis=0), axis=0)
            cosine = np.dot(low_tangent, high_tangent) / (
                np.linalg.norm(low_tangent) * np.linalg.norm(high_tangent))
            cartesian_rows.append({
                "level_mm": level,
                "visit": visit,
                "episode": by_visit[0.0][visit]["episode"],
                "curve_rms_displacement_mm": float(np.sqrt(
                    np.nanmean(np.sum(delta * delta, axis=1)))),
                "tip_displacement_mm": float(np.linalg.norm(delta[-1])),
                "maximum_point_displacement_mm": float(np.nanmax(
                    np.linalg.norm(delta, axis=1))),
                "terminal_tangent_change_deg": float(np.degrees(
                    np.arccos(np.clip(cosine, -1.0, 1.0)))),
            })
    metadata["cartesian_response_0_to_15_mm_tendon"] = cartesian_rows
    metadata["cartesian_response_plateau_medians"] = {}
    for level in sorted({row["level_mm"] for row in cartesian_rows}):
        rows = [row for row in cartesian_rows if row["level_mm"] == level]
        metadata["cartesian_response_plateau_medians"][str(level)] = {
            key: float(np.median([row[key] for row in rows]))
            for key in (
                "curve_rms_displacement_mm", "tip_displacement_mm",
                "maximum_point_displacement_mm",
                "terminal_tangent_change_deg")
        }

    levels = sorted({key[0] for key in selected})
    fig = plt.figure(figsize=(19, 9), constrained_layout=True)
    absolute_axes = []
    relative_axes = []
    absolute_curves = []
    relative_curves = []
    for column, level in enumerate(levels):
        ax_abs = fig.add_subplot(2, len(levels), column + 1, projection="3d")
        ax_rel = fig.add_subplot(
            2, len(levels), len(levels) + column + 1, projection="3d")
        absolute_axes.append(ax_abs)
        relative_axes.append(ax_rel)
        for target in TARGETS_MM:
            records = selected[(level, target)]
            visit_medians = []
            for record in records:
                curve = record["median"]
                visit_medians.append(curve)
                absolute_curves.append(curve)
                relative_curves.append(curve - curve[0])
                linestyle = "--" if record["visit"] == 1 else ":"
                ax_abs.plot(*curve.T, color=COLORS[target], alpha=.42,
                            linewidth=1.4, linestyle=linestyle)
                relative = curve - curve[0]
                ax_rel.plot(*relative.T, color=COLORS[target], alpha=.42,
                            linewidth=1.4, linestyle=linestyle)
            robust = np.nanmedian(np.asarray(visit_medians), axis=0)
            ax_abs.plot(*robust.T, color=COLORS[target], linewidth=3,
                        label=f"tendon {target:g} mm")
            robust_relative = robust - robust[0]
            ax_rel.plot(*robust_relative.T, color=COLORS[target], linewidth=3)
            ax_abs.scatter(*robust[0], color=COLORS[target], s=18)
            ax_rel.scatter(*robust_relative[0], color=COLORS[target], s=18)
        ax_abs.set_title(f"Insertion {level:g} mm")
        ax_rel.set_title(f"Insertion {level:g} mm")
        for ax in (ax_abs, ax_rel):
            ax.set_xlabel("x (mm)", labelpad=3)
            ax.set_ylabel("y (mm)", labelpad=3)
            ax.set_zlabel("z (mm)", labelpad=3)
            ax.view_init(elev=24, azim=-58)
            ax.grid(alpha=.25)

    _equal_3d_limits(absolute_axes, absolute_curves, relative=False)
    _equal_3d_limits(relative_axes, relative_curves, relative=False)
    absolute_axes[0].legend(loc="upper left", fontsize=9)
    fig.text(.012, .735, "Base-frame curves", rotation=90,
             va="center", fontsize=13, fontweight="bold")
    fig.text(.012, .275, "Proximal-point aligned", rotation=90,
             va="center", fontsize=13, fontweight="bold")
    fig.suptitle(
        "Static distal shapes across insertion and tendon actuation\n"
        "thick: median across visits; dashed/dotted: visit medians",
        fontsize=16)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=190)
    if args.metadata:
        args.metadata.write_text(json.dumps(metadata, indent=2) + "\n",
                                 encoding="utf-8")
    print(json.dumps({"figure": str(args.output),
                      "metadata": (str(args.metadata)
                                   if args.metadata else None)}, indent=2))


if __name__ == "__main__":
    main()
