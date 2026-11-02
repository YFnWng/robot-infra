"""Timestamp ownership regression; no node construction or hardware access."""
from types import SimpleNamespace

import pytest
import torch
import numpy as np

from catheter_control.node import CatheterControlNode
from catheter_control.transmission.backlash import BacklashStateEstimator


def bind_timestamp_owner(estimator):
    from types import MethodType
    estimator.observe_at_timestamp=MethodType(BacklashStateEstimator.observe_at_timestamp,estimator)
    return estimator


@pytest.mark.parametrize("armed,source_owned", [(False, True), (True, False)])
def test_external_observation_cannot_restore_or_confirm_transaction(armed, source_owned):
    calls = []
    estimator = SimpleNamespace(
        reset_observation_history=lambda: calls.append("reset_visual_history"),
        snapshot=lambda: "unchanged_belief",
        restore_state=lambda _: pytest.fail("external state must not be restored"))
    bind_timestamp_owner(estimator)
    node = SimpleNamespace(
        armed=armed, backlash_estimator=estimator,
        backlash_snapshot="previous", _backlash_history=[[100, None, None, source_owned]])
    result = CatheterControlNode._observe_backlash_at_timestamp(
        node, 100, None, None, None,
        observed_interface_position_m=[0., 0., 0.], observed_marker_points_m=np.zeros((4, 3)))
    assert result == "unchanged_belief"
    assert calls == []  # A late foreign observation cannot erase current evidence.


def test_owned_observation_forwards_source_time_interface_marker():
    from unittest.mock import Mock
    import numpy as np
    estimator = Mock()
    bind_timestamp_owner(estimator)
    estimator.snapshot.return_value = "corrected_belief"
    node = SimpleNamespace(
        armed=True, engagement_control_enabled=True,
        runtime=SimpleNamespace(mechanics=SimpleNamespace(mode=torch.ones(24))),
        backlash_estimator=estimator, backlash_snapshot="previous",
        _backlash_history=[[100, np.zeros(3), {}, True]])
    position = np.array([.001, .002, .003])
    model = SimpleNamespace(jacobian=np.zeros((6, 3)), state_scale=np.ones(6))
    result = CatheterControlNode._observe_backlash_at_timestamp(
        node, 100, np.eye(4), model, np.zeros(24),
        observed_interface_position_m=position, observed_marker_points_m=np.zeros((4, 3)))
    assert result == "corrected_belief"
    kwargs = estimator.observe_response.call_args.kwargs
    np.testing.assert_array_equal(kwargs["observed_interface_position_m"], position)
    np.testing.assert_array_equal(kwargs["observed_marker_points_m"], np.zeros((4, 3)))
    assert kwargs["timestamp_ns"] == 100


def test_response_knob_angle_is_interpolated_at_camera_source_time():
    from unittest.mock import Mock
    import numpy as np
    estimator = Mock()
    bind_timestamp_owner(estimator)
    node = SimpleNamespace(
        armed=True, engagement_control_enabled=True,
        runtime=SimpleNamespace(mechanics=SimpleNamespace(mode=torch.ones(24))),
        backlash_estimator=estimator, backlash_snapshot="previous",
        _backlash_history=[[100, np.array([0., 0., 0.]), {}, True],
                           [200, np.array([1., 0., -3.]), {}, True]])
    model = SimpleNamespace(jacobian=np.zeros((6, 3)), state_scale=np.ones(6))
    CatheterControlNode._observe_backlash_at_timestamp(
        node, 150, np.eye(4), model, None, observed_interface_position_m=[0., 0., 0.],
        observed_marker_points_m=np.zeros((4, 3)))
    np.testing.assert_allclose(estimator.observe_response.call_args.args[0], [.5, 0., -1.5])
    estimator.advance_motor.assert_called_once()
    np.testing.assert_allclose(estimator.advance_motor.call_args.args[0], [1., 0., -3.])


@pytest.mark.parametrize("following", [None, [300_000_101, [1., 0., -3.], {}, True],
                                        [200, [1., 0., -3.], {}, False]])
def test_unbracketed_response_does_not_restore_or_confirm(following):
    from unittest.mock import Mock
    import numpy as np
    estimator = Mock()
    bind_timestamp_owner(estimator)
    history = [[100, np.zeros(3), {}, True]]
    if following is not None:
        history.append(following)
    node = SimpleNamespace(armed=True, engagement_control_enabled=True,
        backlash_estimator=estimator, backlash_snapshot="previous", _backlash_history=history)
    CatheterControlNode._observe_backlash_at_timestamp(
        node, 150, None, None, None, observed_interface_position_m=[0., 0., 0.],
        observed_marker_points_m=np.zeros((4, 3)))
    estimator.restore_state.assert_not_called()
    estimator.observe_response.assert_not_called()


def test_baseline_encoder_owner_passes_raw_knob_and_transmitted_chassis():
    from collections import deque
    from threading import RLock
    import numpy as np
    from catheter_control.transmission.backlash import BacklashConfig, BacklashStateEstimator
    from catheter_control.safety.hardware_contract import ENCODER_RADIANS_PER_COUNT
    inputs = []
    runtime = SimpleNamespace(state=None, diagnostics=lambda: {})

    def advance(timestamp, counts, *, interface_motor_angle_rad):
        inputs.append((np.array(counts), interface_motor_angle_rad.copy()))
        runtime.state = SimpleNamespace(health="TRACKING", accepted_observations=0,
                                       consecutive_rejections=0)
        return runtime.state

    runtime.initialize = runtime.advance_encoder = advance
    observer = BacklashStateEstimator(BacklashConfig(width_rad=(10., 1., 10.)))
    node = SimpleNamespace(runtime=runtime, engagement_control_enabled=False,
        armed=True, _lock=RLock(), _steady=lambda: 0.,
        _timing=SimpleNamespace(record_seconds=lambda *args: None),
        _raw_encoder_history=deque(maxlen=256), _backlash_history=deque(maxlen=256),
        backlash_compensation_enabled=True, backlash_estimator=observer,
        backlash_snapshot=observer.snapshot(),
        _fault_locked=lambda reason: pytest.fail(reason),
        _publish_planner_snapshot=lambda *args: None)
    assert CatheterControlNode._process_encoder_sample(node, (100, np.zeros(3), 0.))
    runtime.state.timestamp_ns = 100
    counts = np.array([.2, 0., -.3])/ENCODER_RADIANS_PER_COUNT
    assert CatheterControlNode._process_encoder_sample(node, (200, counts, 0.))
    np.testing.assert_allclose(inputs[-1][0], counts)  # Distal history keeps raw encoders.
    np.testing.assert_allclose(inputs[-1][1], [0., 0., -.3])
    assert observer.phase[0] == "TAKEUP" and observer.phase[2] == "TAKEUP"
