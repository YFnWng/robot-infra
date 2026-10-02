import json

import numpy as np
import pytest

from runtime_supervision.qualify_paced_compute import recorded_belief, recorded_config


def test_recorded_configuration_preserves_actual_workload():
    config = recorded_config({"samples": 512, "horizon_steps": 4, "rollout_step_s": .04,
                              "mppi_point_rollout_step_s": .2, "mppi_engaged_gain_scenarios": True,
                              "mppi_engaged_gain_maximum_first_step_shift": 8., "mppi_capture_hold_s": .3})
    assert config.samples == 512 and config.horizon_steps == 4
    assert config.engaged_gain_scenarios and config.step_s == .04
    assert config.point_rollout_step_s == .2 and config.capture_hold_s == .3


def test_recorded_gain_preserves_mean_lower_upper_and_direction():
    values = {"engaged_gain_mean": json.dumps([[1., 1.]]*3),
              "engaged_gain_lower": json.dumps([[.5, .6]]*3),
              "engaged_gain_upper": json.dumps([[1.5, 1.6]]*3),
              "engaged_gain_status": json.dumps([["UNCERTAIN", "CONFIDENT"]]*3),
              "backlash_engaged_direction": "[1,0,-1]"}
    belief = recorded_belief(values)
    np.testing.assert_array_equal(belief.engaged_gain.scenarios(2, -1), [1., .5, 1.5])
    np.testing.assert_array_equal(belief.engaged_gain.scenarios(2, 1), [1., .6, 1.6])
    np.testing.assert_array_equal(belief.engaged_direction, [1, 0, -1])
    with pytest.raises(ValueError, match="required"):
        recorded_belief({})
