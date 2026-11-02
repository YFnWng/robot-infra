"""Read-only geometry diagnosis of tangent-only replay rejections; no ROS nodes.

Recover the scalar preview speed from the original recorded positions, then
evaluate the submitted PCHIP curve at the exact rollout arc coordinates. This
is a diagnostic reconstruction, never a replacement reference or controller.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3

import numpy as np
from scipy.optimize import minimize_scalar
from rclpy.serialization import deserialize_message
from control_interface.msg import TipReferenceHorizon
from catheter_control.planning.path_tracking import ArcLengthPath, ReferenceHorizon


SESSIONS = {
    "grouped_takeup": "continuous_twoaxis_circle_home_grouped_takeup_v171_n512_speed1_mount01_repeat01_20261005T235706766686Z",
    "grouped_no_takeup": "continuous_twoaxis_circle_home_grouped_no_takeup_v171_n512_speed1_mount01_repeat01_20261006T000017898562Z",
}


def tangent_only(row):
    evidence = row["path_commit"]
    before, after = evidence["captured_reference"], evidence["current_reference"]
    return (not evidence["reference_compatible"]
            and evidence["gate_error"] is None
            and evidence["comparison_error"] is None
            and not before["final_hold"]
            and all(before[key] == after[key] for key in (
                "path_id", "total_length_m", "final_hold", "progress_paused"))
            and after["sequence"] >= before["sequence"]
            and -1e-6 <= after["progress_m"]-before["progress_m"] <= 1e-4
            and evidence["position_max_coordinate_change_mm"] <= .01
            and evidence["tangent_max_component_change"] > .002)


def load_previews(session, wanted):
    result = {}
    for database in sorted((session / "robot_bag").glob("*.db3")):
        with sqlite3.connect(f"{database.as_uri()}?mode=ro", uri=True) as connection:
            query = ("SELECT data FROM messages WHERE topic_id IN "
                     "(SELECT id FROM topics WHERE name='/catheter_mppi/reference_horizon')")
            for (payload,) in connection.execute(query):
                message = deserialize_message(payload, TipReferenceHorizon)
                key = (message.path_id, message.sequence)
                if key in wanted:
                    if key in result:
                        raise ValueError(f"duplicate preview identity: {key}")
                    result[key] = message
    if set(result) != wanted:
        raise ValueError("bag lacks selected replay preview identities")
    return result


def reconstruct(path, message, evidence, root_ns, steps, step_s, history_s):
    points = np.array([[p.x, p.y, p.z] for p in message.positions])
    tangents = np.array([[p.x, p.y, p.z] for p in message.tangents])
    offsets = np.arange(len(points))*message.sample_period_s-history_s
    def objective(speed):
        predicted, _ = path.evaluate(message.progress_m+speed*offsets)
        return float(np.sum((predicted-points)**2))
    fit = minimize_scalar(objective, bounds=(0., message.nominal_speed_m_s),
                          method="bounded", options={"xatol": 1e-15})
    if not fit.success or np.sqrt(fit.fun) > 1e-9:
        raise ValueError("recorded preview does not match scalar-speed path reconstruction")
    # Reproduce the replay epoch too: floating-point interpolation at Unix
    # timestamps can depend on the epoch's fractional-second representation.
    stamp = evidence["source_timestamp_ns"]
    root = root_ns
    query = (root-stamp)/1e9+step_s*np.arange(1, steps+1)
    arcs = np.clip(message.progress_m+fit.x*(query-history_s), 0., path.length_m)
    exact_points, exact_tangents = path.evaluate(arcs)
    reference = ReferenceHorizon(
        message.path_id, message.sequence, stamp, 100., message.sample_period_s,
        points, message.expiry_s, message.progress_m, message.total_length_m,
        message.final_hold, message.progress_paused, tangents)
    sampled_points, sampled_tangents = reference.sample(root, steps, step_s, 100.)
    return {
        "sequence": message.sequence,
        "recovered_preview_speed_mm_s": fit.x*1000,
        "generation_residual_norm_um": np.sqrt(fit.fun)*1e6,
        "query_arcs_mm": (arcs*1000).tolist(),
        "position_interpolation_max_coordinate_error_um": float(np.max(np.abs(sampled_points-exact_points))*1e6),
        "tangent_interpolation_max_component_error": float(np.max(np.abs(sampled_tangents-exact_tangents))),
    }, exact_points, exact_tangents, sampled_points, sampled_tangents


def diagnose(suite, sessions_root, history_s):
    rows = []
    for name, session_name in SESSIONS.items():
        report_file = suite / name / "report.json"
        report = json.loads(report_file.read_text())
        if report["ever_armed"] or report["command_output_enabled"]:
            raise ValueError("expected non-actuating qualification report")
        selected = [(index, row) for index, row in enumerate(report["plans"]) if tangent_only(row)]
        session = sessions_root / session_name
        task_file = session / "path_result.json"
        task = json.loads(task_file.read_text())
        path = ArcLengthPath(task["path_knots_m"])
        wanted = {(row["path_commit"][key]["path_id"], row["path_commit"][key]["sequence"])
                  for _, row in selected for key in ("captured_reference", "current_reference")}
        messages = load_previews(session, wanted)
        config = report["planner_fixture_contract"]["planner_config"]
        for index, row in selected:
            evidence = row["path_commit"]
            pair = []
            for key in ("captured_reference", "current_reference"):
                record = evidence[key]
                pair.append(reconstruct(path, messages[(record["path_id"], record["sequence"])],
                                        record, evidence["root_timestamp_ns"],
                                        config["horizon_steps"], config["step_s"], history_s))
            # Prove original message reconstruction reproduces recorded comparisons.
            sampled_position_delta = float(np.max(np.abs(pair[0][3]-pair[1][3]))*1000)
            sampled_tangent_delta = float(np.max(np.abs(pair[0][4]-pair[1][4])))
            if (not np.isclose(sampled_position_delta, evidence["position_max_coordinate_change_mm"], atol=1e-8, rtol=0.)
                    or not np.isclose(sampled_tangent_delta, evidence["tangent_max_component_change"], atol=1e-8, rtol=0.)):
                raise ValueError("original previews do not reproduce replay comparison")
            rows.append({
                "variant": name, "plan_index": index,
                "report_sha256": hashlib.sha256(report_file.read_bytes()).hexdigest(),
                "task_sha256": hashlib.sha256(task_file.read_bytes()).hexdigest(),
                "approach_join_arc_mm": path.arc_knots[1]*1000,
                "approach_join_derivative_norm": float(np.linalg.norm(path.derivatives[1])),
                "recorded_position_change_mm": sampled_position_delta,
                "recorded_tangent_change": sampled_tangent_delta,
                "exact_position_change_mm": float(np.max(np.abs(pair[0][1]-pair[1][1]))*1000),
                "exact_tangent_change": float(np.max(np.abs(pair[0][2]-pair[1][2]))),
                "captured": pair[0][0], "current": pair[1][0],
            })
    return {"scope": "Recorded approach-join geometry and temporal preview interpolation only; no rollout or actuation",
            "preview_history_s": history_s, "cases": rows,
            "limitations": ["Speed is reverse fitted from the whole recorded preview, not causally estimated",
                            "Reconstruction rejects residual norm above 0.001 um",
                            "No command, cost, governor, or compatibility policy is modified"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", type=Path, required=True)
    parser.add_argument("--sessions-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--preview-history-s", type=float, default=.2)
    args = parser.parse_args()
    if not np.isfinite(args.preview_history_s) or args.preview_history_s <= 0:
        parser.error("preview history must be finite and positive")
    report = diagnose(args.suite.resolve(), args.sessions_root.resolve(), args.preview_history_s)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"report": str(args.output), "cases": len(report["cases"])}))


if __name__ == "__main__":
    main()
