"""Read-only comparison of MPPI first-step intent, serial TX, and encoders.

Run with the ROS overlay sourced. No ROS entities or device links are created.
TX bag receipt is a post-write proxy; ENC headers are host receive timestamps.
Neither timestamp is a firmware application/acquisition acknowledgement.
"""
from __future__ import annotations

import argparse
import json
import sqlite3
from pathlib import Path

import numpy as np
from catheter_control.safety.hardware_contract import (
    HardwareContract, MOTOR_AXIS_UNITS_PER_ENCODER_COUNT,
)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def realized_velocity(logical):
    physical = HardwareContract.logical_to_motor_axis_velocity(logical)
    return HardwareContract.quantize_motor_velocity(physical)[2]


def integral_at(times, values, query):
    """Exact integral of a right-continuous piecewise-constant signal."""
    cumulative = np.vstack((np.zeros(values.shape[1]),
                            np.cumsum(np.diff(times)[:, None] * values[:-1], axis=0)))
    index = np.clip(np.searchsorted(times, query, side="right") - 1, 0, len(times)-1)
    return cumulative[index] + (np.asarray(query)-times[index])[..., None]*values[index]


def stats(values):
    values = np.asarray(values)
    if not len(values):
        return {"n": 0}
    return {"n": len(values), "bias": float(np.mean(values)),
            "rmse": float(np.sqrt(np.mean(values**2))),
            "median_abs": float(np.median(np.abs(values))),
            "p95_abs": float(np.percentile(np.abs(values), 95)),
            "max_abs": float(np.max(np.abs(values)))}


def interpolate_encoder(times, positions, queries):
    indices = np.searchsorted(times, queries, side="right")
    valid = (indices > 0) & (indices < len(times))
    indices = np.clip(indices, 1, len(times)-1)
    valid &= times[indices]-times[indices-1] <= 0.05
    output = np.column_stack([np.interp(queries, times, positions[:, axis])
                              for axis in range(6)])
    return output, valid


def analyze(session):
    parameters = json.loads((session / "controller_manifest.json").read_text())["controller_parameters"]
    dt = float(parameters["mppi_point_rollout_step_s"])
    if dt <= 0:
        dt = float(parameters["rollout_step_s"])
    plans, tx, encoders, events = [], [], [], []
    classes = {}
    for bag in sorted((session / "robot_bag").glob("*.db3")):
        with sqlite3.connect(bag.as_uri()+"?mode=ro", uri=True) as connection:
            topics = {row[0]: row[1:] for row in connection.execute("SELECT id,name,type FROM topics")}
            wanted = [key for key, (name, _) in topics.items() if name in (
                "/catheter_mppi/planned_control", "/device/command_tx", "/device/state", "/device/event")]
            placeholders = ",".join("?" for _ in wanted)
            for topic, timestamp, data in connection.execute(
                    f"SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ({placeholders}) ORDER BY timestamp", wanted):
                name, kind = topics[topic]
                if kind not in classes:
                    classes[kind] = get_message(kind)
                message = deserialize_message(data, classes[kind])
                stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
                if name.endswith("planned_control"):
                    plans.append((stamp, realized_velocity(message.joint_vel)))
                elif name.endswith("command_tx") and message.predicate in (80, 86):
                    velocity = realized_velocity(message.data) if message.predicate == 86 else np.zeros(6)
                    tx.append((timestamp, stamp, velocity, message.predicate == 80))
                elif name.endswith("state") and message.predicate == 69:
                    encoders.append((stamp, np.asarray(message.data)*MOTOR_AXIS_UNITS_PER_ENCODER_COUNT))
                elif name.endswith("event") and "MOTION_" in message.text:
                    events.append((stamp, message.text, list(message.data)))
    plans.sort(key=lambda row: row[0]); tx.sort(key=lambda row: row[0]); encoders.sort(key=lambda row: row[0])
    origin = min(plans[0][0], tx[0][0], encoders[0][0])
    pt = np.array([(row[0]-origin)*1e-9 for row in plans]); pv = np.array([row[1] for row in plans])
    tt = np.array([(row[0]-origin)*1e-9 for row in tx]); tv = np.array([row[2] for row in tx])
    th = np.array([(row[1]-origin)*1e-9 for row in tx])
    et = np.array([(row[0]-origin)*1e-9 for row in encoders]); ep = np.array([row[1] for row in encoders])
    et, unique = np.unique(et, return_index=True); ep = ep[unique]
    start, valid_start = interpolate_encoder(et, ep, pt)
    end, valid_end = interpolate_encoder(et, ep, pt+dt)
    valid = valid_start & valid_end & (pt >= tt[0]) & (pt+dt <= tt[-1])
    # Exclude any overlap with a position transaction, including one already active.
    position_mode = np.array([row[3] for row in tx])
    stable = np.zeros(len(pt), dtype=bool)
    for index, time in enumerate(pt):
        first = max(0, np.searchsorted(tt, time, side="right")-1)
        last = np.searchsorted(tt, time+dt, side="left")
        if np.any(position_mode[first:last]):
            valid[index] = False
        boundaries = np.r_[time, tt[first+1:last], time+dt]
        matching = np.all(np.isclose(tv[first:last], pv[index], atol=1e-8), axis=1)
        stable[index] = np.sum(np.diff(boundaries)*matching) >= 0.9*dt
    intended = pv*dt
    transmitted = integral_at(tt, tv, pt+dt)-integral_at(tt, tv, pt)
    observed = end-start
    # Header-time sensitivity: sort/deduplicate because multiple sources can interleave.
    order = np.argsort(th, kind="stable"); hs, hi = np.unique(th[order], return_index=True)
    hv = tv[order][hi]
    header_travel = integral_at(hs, hv, pt+dt)-integral_at(hs, hv, pt)
    axes = {}
    for axis, name in ((0, "chassis_mm"), (2, "tendon_knob_mm")):
        axes[name] = {}
        for cohort, mask in (("all", valid), ("plan_matching", valid & stable),
                             ("modified", valid & ~stable)):
            # Nonzero intent only; pure-hold windows would dilute relative tracking error.
            mask = mask & (np.abs(intended[:, axis]) >= 0.01)
            axes[name][cohort] = {
                "intended_abs_mm": stats(np.abs(intended[mask, axis])),
                "encoder_minus_plan_mm": stats(observed[mask, axis]-intended[mask, axis]),
                "tx_minus_plan_mm": stats(transmitted[mask, axis]-intended[mask, axis]),
                "encoder_minus_tx_mm": stats(observed[mask, axis]-transmitted[mask, axis]),
                "tx_header_vs_receipt_mm": stats(header_travel[mask, axis]-transmitted[mask, axis]),
                "opposite_encoder_percent": float(100*np.mean(observed[mask, axis]*intended[mask, axis] < -0.0001)) if np.any(mask) else None,
            }
        # Constant-command plateau interiors: discard 150 ms after a change and 50 ms before it.
        changes = np.r_[True, np.any(tv[1:] != tv[:-1], axis=1) | (position_mode[1:] != position_mode[:-1])]
        ct = tt[changes]; cv = tv[changes]; cm = position_mode[changes]
        a = ct[:-1]+0.15; b = ct[1:]-0.05
        es, vs = interpolate_encoder(et, ep, a); ee, ve = interpolate_encoder(et, ep, b)
        good = (b-a >= 0.1) & vs & ve & ~cm[:-1] & (np.abs(cv[:-1, axis]) > 1e-6)
        desired = cv[:-1, axis]*(b-a); actual = ee[:, axis]-es[:, axis]
        axes[name]["plateau_interiors"] = {
            "error_mm": stats(actual[good]-desired[good]),
            "actual_over_requested": stats(actual[good]/desired[good]),
            "largest_errors": [{"start_seconds": float(a[i]),
                                "duration_s": float(b[i]-a[i]),
                                "requested_mm": float(desired[i]),
                                "observed_mm": float(actual[i]),
                                "ratio": float(actual[i]/desired[i])}
                               for i in sorted(np.flatnonzero(good),
                                               key=lambda i: abs(actual[i]-desired[i]), reverse=True)[:3]],
        }
        event_times = [(stamp-origin)*1e-9 for stamp, _, data in events
                       if len(data) > 3 and int(data[3]) == axis]
        affected = np.array([any(ct[i] <= t <= ct[i+1] for t in event_times)
                             for i in range(len(a))])
        clean = good & ~affected
        axes[name]["plateau_interiors_without_motion_events"] = {
            "error_mm": stats(actual[clean]-desired[clean]),
            "actual_over_requested": stats(actual[clean]/desired[clean]),
            "excluded_plateaus": int(np.sum(good & affected)),
        }
        # Descriptive effective lag, not hardware latency: fit first half, score second half.
        middle = (et[0]+et[-1])/2
        scores = []
        for lag in np.arange(0, 0.15001, 0.005):
            lag_travel = integral_at(tt, tv, pt+dt-lag)-integral_at(tt, tv, pt-lag)
            fit = valid & (pt < middle) & (pt-lag >= tt[0]) & (np.abs(transmitted[:, axis]) >= 0.01)
            score = np.mean((observed[fit, axis]-lag_travel[fit, axis])**2) if np.any(fit) else float("inf")
            scores.append((score, lag, lag_travel))
        _, lag, lag_travel = min(scores, key=lambda row: row[0])
        test = valid & (pt >= middle) & (pt-lag >= tt[0]) & (np.abs(transmitted[:, axis]) >= 0.01)
        axes[name]["lag_sensitivity"] = {
            "first_half_best_lag_ms": float(lag*1000),
            "second_half_zero_lag_error_mm": stats(observed[test, axis]-transmitted[test, axis]),
            "second_half_shifted_error_mm": stats(observed[test, axis]-lag_travel[test, axis]),
        }
    return {"session": session.name, "point_step_s": dt, "plan_rate_hz": parameters["plan_rate_hz"],
            "timestamp_origin_ns": origin,
            "plan_count": len(plans), "valid_velocity_windows": int(np.sum(valid)),
            "plan_matching_windows": int(np.sum(valid & stable)), "encoder_samples": len(et),
            "encoder_interval_ms": stats(np.diff(et)*1000),
            "plan_interval_ms": stats(np.diff(pt)*1000),
            "tx_receipt_minus_source_ms": stats((tt-th)*1000),
            "motion_events": [{"seconds": (stamp-origin)*1e-9, "text": text,
                               "axis": int(data[3]) if len(data)>3 else None}
                              for stamp, text, data in events], "axes": axes}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sessions", nargs="+", type=Path)
    args = parser.parse_args()
    print(json.dumps([analyze(session.resolve()) for session in args.sessions], indent=2))
