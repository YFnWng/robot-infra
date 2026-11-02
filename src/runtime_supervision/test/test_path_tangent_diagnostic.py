"""Geometry diagnostic characterization; no ROS node or actuating entities."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import Point, Vector3

tool = Path(__file__).resolve().parents[3] / "audits/ros-realtime/tools/diagnose_path_tangents.py"
spec = importlib.util.spec_from_file_location("diagnose_path_tangents", tool)
diagnostic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostic)


def preview():
    path = diagnostic.ArcLengthPath([
        [.025854, .022463, .068156], [.026774, .023490, .065864],
        [.026785, .023501, .066217], [.026818, .023534, .066568]])
    progress = path.arc_knots[1]-.0001
    offsets = np.arange(41)*.02-.2
    points, tangents = path.evaluate(progress+.00085*offsets)
    message = SimpleNamespace(
        positions=[Point(x=x, y=y, z=z) for x, y, z in points],
        tangents=[Vector3(x=x, y=y, z=z) for x, y, z in tangents],
        path_id="test", sequence=1, progress_m=progress, nominal_speed_m_s=.001,
        sample_period_s=.02, expiry_s=.15, total_length_m=path.length_m,
        final_hold=False, progress_paused=False)
    return path, message


def test_reconstruction_recovers_preview_and_exposes_tangent_interpolation_error():
    path, message = preview()
    source = 1_791_256_349_100_000_000
    row, exact_points, exact_tangents, sampled_points, sampled_tangents = diagnostic.reconstruct(
        path, message, {"source_timestamp_ns": source}, source+267_000_000, 4, .04, .2)
    assert row["recovered_preview_speed_mm_s"] == pytest.approx(.85, abs=1e-7)
    assert row["generation_residual_norm_um"] < .001
    assert np.max(np.abs(exact_points-sampled_points)) < 1e-6
    assert np.max(np.abs(exact_tangents-sampled_tangents)) > .002


def test_wrong_history_does_not_silently_explain_recording():
    path, message = preview()
    with pytest.raises(ValueError, match="does not match"):
        diagnostic.reconstruct(path, message, {"source_timestamp_ns": 1_000_000_000},
                               1_260_000_000, 4, .04, .1)


@pytest.mark.parametrize("field,value", [
    ("position_max_coordinate_change_mm", .011),
    ("tangent_max_component_change", .001),
    ("gate_error", "stale"), ("comparison_error", "missing"),
])
def test_selector_excludes_other_rejections(field, value):
    reference = dict(path_id="test", total_length_m=.02, sequence=1, progress_m=.001,
                     final_hold=False, progress_paused=False)
    evidence = dict(captured_reference=reference,
                    current_reference={**reference, "sequence": 2, "progress_m": .00103},
                    reference_compatible=False, gate_error=None, comparison_error=None,
                    position_max_coordinate_change_mm=.0001, tangent_max_component_change=.2)
    assert diagnostic.tangent_only({"path_commit": evidence})
    evidence[field] = value
    assert not diagnostic.tangent_only({"path_commit": evidence})
