"""Offline arithmetic checks; no ROS nodes or hardware access."""
import numpy as np

from audit_mppi_motor_displacement import (
    integral_at, interpolate_encoder, realized_velocity,
)


def test_piecewise_integral_crosses_stop_and_reversal():
    times = np.array([0., .1, .2, .4])
    values = np.array([[2.], [0.], [-1.], [0.]])
    np.testing.assert_allclose(integral_at(times, values, [.05, .15, .3, .4]),
                               [[.1], [.2], [.1], [0.]], atol=1e-15)


def test_interpolation_rejects_missing_encoder_support():
    positions = np.tile(np.array([0., .02, .1])[:, None], (1, 6))
    result, valid = interpolate_encoder(np.array([0., .02, .1]), positions,
                                        np.array([-.01, .01, .06, .11]))
    assert valid.tolist() == [False, True, False, False]
    np.testing.assert_allclose(result[1], .01)


def test_knob_translation_is_not_chassis_motion():
    # Equal logical insertion and knob actuation cancel in physical chassis.
    rate = realized_velocity([2., 0., 2., 0., 0., 0.])
    assert rate[0] == 0.
    np.testing.assert_allclose(rate[2], 2.00421875)
