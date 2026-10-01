import textwrap

import numpy as np
import pytest

from catheter_control.path_file import _arguments, load_path_file


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
