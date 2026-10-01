import numpy as np
import pytest

from catheter_control.sim_perception import _quaternion_from_rotation


def test_rotation_matrix_to_ros_quaternion_round_trip_cases():
    assert _quaternion_from_rotation(np.eye(3)) == pytest.approx(
        [0.0, 0.0, 0.0, 1.0])
    half_turn_x = np.diag([1.0, -1.0, -1.0])
    quaternion = _quaternion_from_rotation(half_turn_x)
    assert np.abs(quaternion) == pytest.approx([1.0, 0.0, 0.0, 0.0])

    quarter_turn_z = np.asarray([[0.0, -1.0, 0.0],
                                 [1.0, 0.0, 0.0],
                                 [0.0, 0.0, 1.0]])
    assert _quaternion_from_rotation(quarter_turn_z) == pytest.approx(
        [0.0, 0.0, 2**-0.5, 2**-0.5])
