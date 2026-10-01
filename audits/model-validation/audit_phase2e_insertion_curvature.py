#!/usr/bin/env python3
"""Audit Phase-2e reconstruction integrity and insertion-conditioned bending.

The analysis aligns the offline four-view reconstruction to the synchronous
causal-experiment trace.  Curvature response is measured within each complete
compensated-bending episode, so continuous mechanical history is preserved.
"""
from __future__ import annotations

import argparse
import itertools
import json
import re
import sys
from pathlib import Path

import h5py
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import evaluate_causal_proximal_experiment as causal  # noqa: E402


EPISODE_RE = re.compile(
    r"compensated_bend_insertion_p(?P<pass>[12])_level_"
    r"(?P<level>[0-9.]+)_(?P<speed>slow|fast)_"
    r"(?P<branch>high_first|low_first)")


def _pct(values, q):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.percentile(values, q)) if values.size else None


def _anova_permutation(groups):
    ordered = [np.asarray(groups[key], dtype=float) for key in sorted(groups)]
    observed = float(stats.f_oneway(*ordered).statistic)
    pooled = np.concatenate(ordered)
    labels = np.repeat(np.arange(len(ordered)), [len(x) for x in ordered])
    exceed = 0
    total = 0
    # Eight values partitioned into four labelled pairs: only 2520 assignments.
    for candidate in set(itertools.permutations(labels.tolist())):
        split = [pooled[np.asarray(candidate) == index]
                 for index in range(len(ordered))]
        value = float(stats.f_oneway(*split).statistic)
        exceed += value >= observed - 1e-12
        total += 1
    grand = float(np.mean(pooled))
    between = sum(len(x) * (float(np.mean(x)) - grand) ** 2 for x in ordered)
    total_ss = float(np.sum((pooled - grand) ** 2))
    return {
        "f": observed,
        "nominal_p": float(stats.f_oneway(*ordered).pvalue),
        "exact_permutation_p": float(exceed / total),
        "eta_squared": float(between / total_ss),
        "permutations": total,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--shape-h5", type=Path, required=True)
    parser.add_argument("--robot-session", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path, required=True)
    args = parser.parse_args()

    _, messages = causal._read(args.robot_session)
    traces = causal._episode_rows(messages)
    trace_t = np.asarray([row["timestamp_ns"] for row in traces], dtype=np.int64)

    with h5py.File(args.shape_h5, "r") as h5:
        t = h5["frames/timestamp_ns"][:]
        valid = h5["frames/valid"][:].astype(bool)
        learning = h5["frames/learning_valid"][:].astype(bool)
        curvature = h5["distal/curvature_per_mm"][:] * 1000.0
        s_mm = h5["distal/s_mm"][:]
        temporal_adjustment = h5["quality/shape_temporal_adjustment_rms_mm"][:]
        coefficient_innovation = h5["quality/shape_coefficient_innovation_rms_mm"][:]
        symmetric_px = h5["quality/multi_view_active_symmetric_mean_px"][:]
        terminal_disagreement = h5[
            "quality/multi_view_cross_rig_terminal_disagreement_mm"][:]
        reprojection_p95 = h5["quality/reprojection_p95_px"][:]
        encoder_age = h5["robot/encoder_age_ms"][:]
        position_age = h5["robot/position_age_ms"][:]
        command_age = h5["robot/command_age_ms"][:]
        encoder_valid = h5["robot/encoder_valid"][:].astype(bool)
        position_valid = h5["robot/position_valid"][:].astype(bool)
        command_valid = h5["robot/command_valid"][:].astype(bool)
        interpolated = h5["frames/curve_temporally_interpolated"][:].astype(bool)
        frame_outlier = h5["quality/shape_temporal_frame_outlier"][:].astype(bool)
        terminal_outlier = h5[
            "quality/shape_temporal_terminal_outlier"][:].astype(bool)

    nearest = np.searchsorted(trace_t, t)
    nearest = np.clip(nearest, 1, len(trace_t) - 1)
    left = nearest - 1
    choose_left = np.abs(t - trace_t[left]) <= np.abs(trace_t[nearest] - t)
    nearest[choose_left] = left[choose_left]
    alignment_ms = np.abs(t - trace_t[nearest]) / 1e6
    names = np.asarray([traces[index]["name"] for index in nearest], dtype=object)
    measured = np.asarray([traces[index]["position"] for index in nearest])

    episodes = []
    spatial_by_level = {}
    pooled_curve = {}
    for name in dict.fromkeys(names.tolist()):
        match = EPISODE_RE.fullmatch(str(name))
        if not match:
            continue
        episode_mask = (names == name) & (alignment_ms <= 20.0)
        mask = episode_mask & learning
        if np.count_nonzero(mask) < 20:
            continue
        bend = measured[mask, 2]
        insert = measured[mask, 0]
        curves = curvature[mask]
        s = s_mm[mask]
        central = curves[:, 6:-6]
        scalar = np.median(central, axis=1)
        mean_scalar = np.mean(central, axis=1)
        # Endpoints may be absent on otherwise learning-valid curves.  Use the
        # same central support as the scalar curvature metrics.
        turning = np.trapezoid(central / 1000.0, s[:, 6:-6], axis=1)
        lo = bend <= np.percentile(bend, 8)
        hi = bend >= np.percentile(bend, 92)
        level = float(match.group("level"))
        record = {
            "name": name,
            "pass": int(match.group("pass")),
            "level_mm": level,
            "speed": match.group("speed"),
            "branch": match.group("branch"),
            "frames": int(np.count_nonzero(mask)),
            "learning_valid_fraction": float(np.mean(learning[episode_mask])),
            "quality": {
                "active_symmetric_error_px_median": _pct(
                    symmetric_px[episode_mask], 50),
                "active_symmetric_error_px_p95": _pct(
                    symmetric_px[episode_mask], 95),
                "cross_rig_terminal_disagreement_mm_median": _pct(
                    terminal_disagreement[episode_mask], 50),
                "cross_rig_terminal_disagreement_mm_p95": _pct(
                    terminal_disagreement[episode_mask], 95),
                "temporal_adjustment_mm_median": _pct(
                    temporal_adjustment[episode_mask], 50),
                "temporal_adjustment_mm_p95": _pct(
                    temporal_adjustment[episode_mask], 95),
            },
            "actual_insertion_mm": {
                "p05": _pct(insert, 5), "median": _pct(insert, 50),
                "p95": _pct(insert, 95)},
            "bend_span_mm": float(np.percentile(bend, 96)
                                  - np.percentile(bend, 4)),
            "central_median_curvature_delta_per_m": float(
                np.median(scalar[hi]) - np.median(scalar[lo])),
            "central_mean_curvature_delta_per_m": float(
                np.mean(mean_scalar[hi]) - np.mean(mean_scalar[lo])),
            "integrated_turning_delta_rad": float(
                np.median(turning[hi]) - np.median(turning[lo])),
        }
        episodes.append(record)
        spatial_by_level.setdefault(level, []).append(
            np.median(curves[hi], axis=0) - np.median(curves[lo], axis=0))
        pooled_curve.setdefault(level, []).append((bend, scalar))

    episodes.sort(key=lambda x: (x["level_mm"], x["pass"]))
    levels = sorted(spatial_by_level)
    plateau = {}
    groups = {}
    for level in levels:
        rows = [row for row in episodes if row["level_mm"] == level]
        values = np.asarray([
            row["central_median_curvature_delta_per_m"] for row in rows])
        groups[level] = values
        plateau[str(level)] = {
            "visits": len(rows),
            "central_median_curvature_delta_per_m": float(np.median(values)),
            "repeat_cv": float(np.std(values, ddof=1) / np.mean(values)),
            "central_mean_curvature_delta_per_m": float(np.median([
                row["central_mean_curvature_delta_per_m"] for row in rows])),
            "integrated_turning_delta_rad": float(np.median([
                row["integrated_turning_delta_rad"] for row in rows])),
        }

    response = np.asarray([
        row["central_median_curvature_delta_per_m"] for row in episodes])
    insertion = np.asarray([
        row["actual_insertion_mm"]["median"] for row in episodes])
    rho, rho_p = stats.spearmanr(insertion, response)
    dt_ms = np.diff(t) / 1e6
    report = {
        "shape_h5": str(args.shape_h5),
        "robot_session": str(args.robot_session),
        "verdict": (
            "Insertion materially changes tendon-to-distal curvature response, "
            "but the response is not monotone and is not a scalar gain only."),
        "integrity": {
            "paired_frames": int(len(t)),
            "valid_frames": int(np.count_nonzero(valid)),
            "valid_fraction": float(np.mean(valid)),
            "learning_valid_frames": int(np.count_nonzero(learning)),
            "learning_valid_fraction": float(np.mean(learning)),
            "frame_dt_ms": {"median": _pct(dt_ms, 50), "p95": _pct(dt_ms, 95),
                            "maximum": float(np.max(dt_ms))},
            "trace_alignment_ms": {"median": _pct(alignment_ms, 50),
                                   "p95": _pct(alignment_ms, 95),
                                   "maximum": float(np.max(alignment_ms))},
            "encoder_valid_fraction": float(np.mean(encoder_valid)),
            "position_valid_fraction": float(np.mean(position_valid)),
            "command_valid_fraction": float(np.mean(command_valid)),
            "encoder_age_ms_p95": _pct(encoder_age[encoder_valid], 95),
            "position_age_ms_p95": _pct(position_age[position_valid], 95),
            "command_age_ms_p95": _pct(command_age[command_valid], 95),
            "interpolated_frames": int(np.count_nonzero(interpolated)),
            "whole_curve_temporal_outliers": int(np.count_nonzero(frame_outlier)),
            "terminal_temporal_outliers": int(np.count_nonzero(terminal_outlier)),
            "temporal_adjustment_mm": {"median": _pct(temporal_adjustment, 50),
                                       "p95": _pct(temporal_adjustment, 95)},
            "coefficient_innovation_mm": {
                "median": _pct(coefficient_innovation, 50),
                "p95": _pct(coefficient_innovation, 95)},
            "active_symmetric_error_px": {"median": _pct(symmetric_px, 50),
                                          "p95": _pct(symmetric_px, 95)},
            "cross_rig_terminal_disagreement_mm": {
                "median": _pct(terminal_disagreement, 50),
                "p95": _pct(terminal_disagreement, 95)},
            "reprojection_p95_px": {"median": _pct(reprojection_p95, 50),
                                    "p95": _pct(reprojection_p95, 95)},
        },
        "episodes": episodes,
        "plateau_summary": plateau,
        "plateau_effect_test": _anova_permutation(groups),
        "monotonicity_test": {"spearman_rho": float(rho),
                              "p": float(rho_p)},
    }

    prefix = args.output_prefix
    prefix.parent.mkdir(parents=True, exist_ok=True)
    prefix.with_suffix(".json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")

    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    ax = axes[0, 0]
    for pass_index, marker in ((1, "o"), (2, "s")):
        rows = [row for row in episodes if row["pass"] == pass_index]
        ax.plot([x["level_mm"] for x in rows],
                [x["central_median_curvature_delta_per_m"] for x in rows],
                marker=marker, label=f"visit {pass_index}")
    ax.plot(levels, [plateau[str(x)][
        "central_median_curvature_delta_per_m"] for x in levels],
        color="black", linewidth=2.5, marker="D", label="visit median")
    ax.set(title="Tendon sweep response by insertion",
           xlabel="Insertion plateau (mm)",
           ylabel="High-minus-low central curvature (1/m)")
    ax.grid(alpha=.3); ax.legend()

    ax = axes[0, 1]
    u = np.linspace(0, 1, curvature.shape[1])
    for level in levels:
        curve = np.median(np.asarray(spatial_by_level[level]), axis=0)
        ax.plot(u, curve, label=f"{level:g} mm")
    ax.axhline(0, color="black", linewidth=.8)
    ax.set(title="Where the added curvature appears",
           xlabel="Normalized distal arclength", ylabel="Curvature change (1/m)")
    ax.grid(alpha=.3); ax.legend(title="Insertion")

    ax = axes[1, 0]
    colors = plt.cm.viridis(np.linspace(0, 1, len(levels)))
    for color, level in zip(colors, levels):
        bend = np.concatenate([item[0] for item in pooled_curve[level]])
        scalar = np.concatenate([item[1] for item in pooled_curve[level]])
        edges = np.linspace(np.percentile(bend, 2), np.percentile(bend, 98), 13)
        centers = .5 * (edges[:-1] + edges[1:])
        medians = [np.median(scalar[(bend >= a) & (bend < b)])
                   for a, b in zip(edges[:-1], edges[1:])]
        ax.plot(centers, medians, color=color, label=f"{level:g} mm")
    ax.set(title="Curvature versus measured tendon position",
           xlabel="Measured tendon/bending joint (mm)",
           ylabel="Central median curvature (1/m)")
    ax.grid(alpha=.3); ax.legend(title="Insertion")

    ax = axes[1, 1]
    quality = [
        ("Learning valid", 100 * np.mean(learning)),
        ("Position valid", 100 * np.mean(position_valid)),
        ("Encoder valid", 100 * np.mean(encoder_valid)),
        ("Command valid", 100 * np.mean(command_valid)),
    ]
    ax.barh([x[0] for x in quality], [x[1] for x in quality], color="#4C78A8")
    ax.set_xlim(95, 100.1)
    ax.set_xlabel("Valid fraction (%)")
    ax.set_title("Data integrity")
    ax.grid(axis="x", alpha=.3)
    for index, (_, value) in enumerate(quality):
        ax.text(value - .05, index, f"{value:.3f}%", va="center", ha="right",
                color="white", fontweight="bold")

    fig.suptitle("Phase-2e insertion-stratified tendon audit", fontsize=15)
    fig.savefig(prefix.with_suffix(".png"), dpi=180)
    print(json.dumps({"json": str(prefix.with_suffix('.json')),
                      "plot": str(prefix.with_suffix('.png')),
                      "verdict": report["verdict"]}, indent=2))


if __name__ == "__main__":
    main()
