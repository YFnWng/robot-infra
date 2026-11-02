import textwrap

import numpy as np
import pytest

from control_tasks.path_file import _arguments, load_path_file


def _path_yaml(tmp_path):
    source = tmp_path / "path.yaml"
    source.write_text(textwrap.dedent("""
        frame_id: robot_base
        nominal_speed_mm_s: 2.0
        total_timeout_s: 90.0
        generator:
          type: circle_yz_from_current_tip
          radius_mm: 10.0
          center_x_offset_mm: 10.0
          waypoint_count: 18
          close_circle: true
        """), encoding="utf-8")
    return source


def test_runtime_speed_and_timeout_overrides_do_not_change_geometry(tmp_path):
    source = _path_yaml(tmp_path)
    tip = np.array([0.02, 0.01, 0.07])
    baseline = load_path_file(source, tip)
    overridden = load_path_file(
        source, tip, speed_override=1.0, total_timeout_override=150.0)

    np.testing.assert_allclose(overridden.knots_m, baseline.knots_m)
    assert overridden.nominal_speed_mm_s == 1.0
    assert overridden.total_timeout_s == 150.0
    assert baseline.nominal_speed_mm_s == 2.0
    assert baseline.total_timeout_s == 90.0


@pytest.mark.parametrize("option", ["--speed-mm-s", "--total-timeout-s"])
@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_runtime_path_overrides_must_be_finite_and_positive(
        tmp_path, option, value):
    with pytest.raises(SystemExit):
        _arguments(["catheter_tip_path_file", str(_path_yaml(tmp_path)),
                    option, value])


def test_runtime_path_override_arguments(tmp_path):
    parsed = _arguments([
        "catheter_tip_path_file", str(_path_yaml(tmp_path)),
        "--speed-mm-s", "3", "--total-timeout-s", "75"])
    assert parsed.speed_mm_s == 3.0
    assert parsed.total_timeout_s == 75.0


def test_twoaxis_circle_starts_at_tip_and_stays_in_empirical_plane(tmp_path):
    from pathlib import Path
    source = Path(__file__).resolve().parents[2] / "catheter_control/config/experiments/twoaxis_circle.yaml"
    tip = np.array([.026, .023, .066])
    spec = load_path_file(source, tip)
    np.testing.assert_allclose(spec.knots_m[0], tip)
    np.testing.assert_allclose(spec.knots_m[-1], tip)
    delta = spec.knots_m-tip
    np.testing.assert_allclose(delta[:, 0], delta[:, 1], atol=1e-12)
    center = tip + .004*np.array([2**-.5, 2**-.5, 0])
    np.testing.assert_allclose(np.linalg.norm(spec.knots_m-center, axis=1), .004)
    assert spec.nominal_speed_mm_s == 1


def test_insertion_line_is_measured_tip_anchored_out_and_back():
    from pathlib import Path
    source = Path(__file__).resolve().parents[2] / "catheter_control/config/experiments/insertion_line.yaml"
    tip = np.array([.026, .023, .066])
    spec = load_path_file(source, tip)
    np.testing.assert_allclose(spec.knots_m, [tip, tip + [0, 0, .005], tip])
    shifted = load_path_file(source, tip + [.001, -.002, .003])
    np.testing.assert_allclose(shifted.knots_m-spec.knots_m,
                               np.tile([.001, -.002, .003], (3, 1)))
    assert spec.nominal_speed_mm_s == 1


def test_global_x_line_is_scalar_and_tip_anchored():
    from pathlib import Path
    source = Path(__file__).resolve().parents[2] / "catheter_control/config/experiments/global_x_line.yaml"
    tip = np.array([.026, .023, .066])
    spec = load_path_file(source, tip)
    assert spec.x_only
    np.testing.assert_allclose(spec.knots_m, [tip, tip+[.003, 0, 0], tip])


def test_global_xz_line_is_longer_and_tip_anchored():
    from pathlib import Path
    source = Path(__file__).resolve().parents[2] / "catheter_control/config/experiments/global_xz_line.yaml"
    tip = np.array([.026, .023, .066])
    spec = load_path_file(source, tip)
    assert spec.xz_only and not spec.x_only
    np.testing.assert_allclose(spec.knots_m, [tip, tip+[.006, 0, 0], tip])


@pytest.mark.parametrize("changes", [
    {"xz_only": "true"}, {"xz_only": True, "x_only": True},
    {"x_only": "true"}, {"x_only": True, "frame_id": "camera"},
    {"x_only": True, "path_knots_m": [[0, 0, 0], [0, .003, 0]]}])
def test_invalid_scalar_x_task_is_rejected(tmp_path, changes):
    import yaml
    source = tmp_path/"x.yaml"
    document = {"path_knots_m": [[0, 0, 0], [.003, 0, 0]], **changes}
    source.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="x_only|xz_only"):
        load_path_file(source, [0, 0, 0])


@pytest.mark.parametrize("length", [0, -1, float("nan"), float("inf")])
def test_insertion_line_rejects_invalid_length(tmp_path, length):
    import yaml
    source = tmp_path / "line.yaml"
    source.write_text(yaml.safe_dump({"generator": {
        "type": "insertion_line_out_and_back_from_current_tip",
        "length_mm": length}}))
    with pytest.raises(ValueError, match="length_mm"):
        load_path_file(source, [0, 0, 0])


@pytest.mark.parametrize("basis", [None, [[1, 0, 0], [1, 0, 0]],
                                   [[2, 0, 0], [0, 0, 1]]])
def test_plane_circle_rejects_invalid_basis(basis):
    from control_tasks.trajectory_file import _circle_waypoints
    with pytest.raises(ValueError, match="plane_basis"):
        _circle_waypoints({"type": "circle_plane_from_current_tip",
                           "plane_basis": basis, "radius_mm": 4}, [0, 0, 0])


def test_freeze_command_never_waits_for_server_or_sends_goal(tmp_path, monkeypatch):
    import time
    from types import SimpleNamespace
    from control_tasks import path_file

    class FakeClient:
        def __init__(self, *_):
            self.tip = np.array([.02, .01, .07])
            self.tip_received_at = time.monotonic()
            self.feedback_records = []
            # Any server/client access would fail the test.
        def get_logger(self):
            return SimpleNamespace(info=lambda *_: None)
        def destroy_node(self):
            pass

    monkeypatch.setattr(path_file, "PathFileClient", FakeClient)
    monkeypatch.setattr(path_file.rclpy, "init", lambda **_: None)
    monkeypatch.setattr(path_file.rclpy, "ok", lambda: False)
    frozen = tmp_path / "frozen.yaml"
    path_file.main(["path_file", str(_path_yaml(tmp_path)),
                    "--freeze-path", str(frozen)])
    import yaml
    content = yaml.safe_load(frozen.read_text())
    assert "generator" not in content
    assert len(content["path_knots_m"]) > 2
    with pytest.raises(FileExistsError):
        path_file.main(["path_file", str(_path_yaml(tmp_path)),
                        "--freeze-path", str(frozen)])


def test_missing_feedback_metrics_are_null():
    from control_tasks.path_file import _finite_json
    assert _finite_json({"feedback": [{"error": float("nan")} ]}) == {
        "feedback": [{"error": None}]}


def test_preserve_knots_does_not_splice_measured_tip_into_frozen_circle(tmp_path):
    import yaml
    knots = [[.026, .023, .066], [.030, .023, .070], [.026, .023, .066]]
    source = tmp_path / "frozen.yaml"
    source.write_text(yaml.safe_dump({"path_knots_m": knots}))
    tip = [.026, .023, .068]
    legacy = load_path_file(source, tip)
    preserved = load_path_file(source, tip, preserve_knots=True)
    assert len(legacy.knots_m) == 4
    np.testing.assert_array_equal(preserved.knots_m, knots)
    np.testing.assert_array_equal(legacy.knots_m[0], tip)
    assert _arguments(["path_file", str(source), "--preserve-knots"]).preserve_knots


def test_preserved_path_rejects_unacquired_start_before_server_access(tmp_path, monkeypatch):
    import time
    from types import SimpleNamespace
    import yaml
    from control_tasks import path_file
    source = tmp_path / "frozen.yaml"
    source.write_text(yaml.safe_dump({"path_knots_m": [[0, 0, .06], [.004, 0, .06]],
                                      "final_tolerance_mm": 1.8}))
    class FakeClient:
        def __init__(self, *_):
            self.tip = np.array([0, 0, .065])
            self.tip_received_at = time.monotonic()
            self.feedback_records = []
        def destroy_node(self):
            pass
    monkeypatch.setattr(path_file, "PathFileClient", FakeClient)
    monkeypatch.setattr(path_file.rclpy, "init", lambda **_: None)
    monkeypatch.setattr(path_file.rclpy, "ok", lambda: False)
    with pytest.raises(RuntimeError, match="start not acquired"):
        path_file.main(["path_file", str(source), "--preserve-knots"])
