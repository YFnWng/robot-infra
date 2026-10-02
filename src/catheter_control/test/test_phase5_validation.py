from types import SimpleNamespace

import numpy as np
import pytest
import torch

from catheter_control.transmission.backlash import TakeupTransactionArbiter
from catheter_control.node import CatheterControlNode
from catheter_control.safety.validation import percentile_metrics, response_metrics


def test_percentile_metrics_reports_milliseconds():
    result = percentile_metrics([1.0, 2.0, 3.0, 4.0])

    assert result["mean_ms"] == 2.5
    assert result["p50_ms"] == 2.5
    assert result["max_ms"] == 4.0
    assert result["p95_ms"] == pytest.approx(3.85)
    assert result["p99_ms"] == pytest.approx(3.97)


def test_response_metrics_exact_and_scaled_motion():
    predicted = np.asarray([[1.0, 0.0, -2.0], [0.5, 1.0, 0.0]])
    exact = response_metrics(predicted, predicted)
    scaled = response_metrics(0.5*predicted, predicted)

    assert exact == {
        "rmse": 0.0,
        "max_error": 0.0,
        "direction_cosine": pytest.approx(1.0),
        "signed_gain": 1.0,
    }
    assert scaled["direction_cosine"] == pytest.approx(1.0)
    assert scaled["signed_gain"] == pytest.approx(0.5)


def test_response_metrics_rejects_bad_shapes_and_nonfinite_values():
    with pytest.raises(ValueError):
        response_metrics([[1.0]], [1.0])
    with pytest.raises(ValueError):
        response_metrics([[np.nan]], [[0.0]])
    with pytest.raises(ValueError):
        percentile_metrics([])


def test_estimator_trace_uses_observation_time_posterior_state():
    published = []
    state = SimpleNamespace(
        timestamp_ns=1_234_000_000,
        motor_angle_rad=torch.tensor([0.1, -0.2, 0.3]),
        downstream=torch.tensor([1.0, 2.0, 3.0]),
        interface_pose=torch.eye(4),
        strain=torch.arange(24, dtype=torch.float32),
        estimator_covariance=torch.eye(10),
        accepted_observations=17,
        consecutive_rejections=0,
        health="TRACKING",
        adaptive_jacobian=SimpleNamespace(jacobian=np.arange(18).reshape(6, 3)),
        last_rls_reason="disabled",
        last_rls_weight=0.0,
        last_rls_update_norm=0.0,
        last_rls_window_frames=4,
    )
    result = SimpleNamespace(
        reason="accepted", rms_before_mm=0.4, rms_after_mm=0.2,
        innovation_nis=3.0, innovation_nis_per_dof=0.3,
        observable_rank=9)
    fake = SimpleNamespace(
        frame_id="robot_base",
        estimator_trace_pub=SimpleNamespace(publish=published.append),
        runtime=SimpleNamespace(
            markers_for_state=lambda state: torch.zeros((4, 3))),
        backlash_snapshot=SimpleNamespace(
            remaining_rad=np.zeros(3),
            remaining_lower_rad=np.zeros(3),
            remaining_upper_rad=np.ones(3),
            width_positive_lower_rad=np.zeros(3),
            width_positive_upper_rad=np.ones(3),
            width_negative_lower_rad=np.zeros(3),
            width_negative_upper_rad=np.ones(3),
            reversal_start_motor_rad=np.full(3, np.nan),
            engagement_anchor_motor_rad=np.full(3, np.nan),
            accumulated_takeup_rad=np.zeros(3),
            effective_motor_rad=np.zeros(3),
            effective_motor_uncertainty_rad=np.zeros(3),
            last_evidence_timestamp_ns=np.zeros(3, dtype=np.int64),
            motion_direction=np.zeros(3, dtype=np.int8),
            engaged_direction=np.zeros(3, dtype=np.int8),
            confidence=np.zeros(3),
            confirmation_count=np.zeros(3, dtype=np.int32),
            phase=("UNKNOWN",)*3,
            response_classification=("NOT_EVALUATED",)*3,
            inferred_transmitted_increment_rad=np.array([.1, 0.0, -.2]),
            response_evidence=np.array([.9, .1, .8]),
            joint_response_residual=.03,
            distal_bending_increment=.12,
            tendon_distal_response_evidence=1.0,
            tendon_distal_response_confirmed=True),
        _optional_float=CatheterControlNode._optional_float)

    CatheterControlNode._publish_estimator_trace(
        fake, 1_234_000_000, state, result, {
            "adaptation_enabled": False,
            "marker_timing_rewind_ms": 0.1,
            "marker_timing_correction_ms": 1.2,
            "marker_timing_replay_ms": 0.3,
            "marker_timing_total_ms": 1.6,
        }, raw_encoder_counts=np.array([101.0, -202.0, 303.0]))

    message = published[0]
    assert message.header.stamp.sec == 1
    assert message.header.stamp.nanosec == 234_000_000
    assert message.state_timestamp_ns == 1_234_000_000
    assert message.interface_pose == pytest.approx(np.eye(4).reshape(-1))
    assert message.interface_jacobian == pytest.approx(np.arange(18))
    assert message.estimator_covariance_diagonal == pytest.approx(np.ones(10))
    assert message.raw_encoder_counts == pytest.approx([101.0, -202.0, 303.0])
    assert message.motor_angle_rad == pytest.approx([0.1, -0.2, 0.3])
    assert message.backlash_inferred_transmitted_increment_rad == pytest.approx(
        [.1, 0.0, -.2])
    assert message.backlash_response_evidence == pytest.approx([.9, .1, .8])
    assert message.backlash_joint_response_residual == pytest.approx(.03)
    assert message.backlash_distal_bending_increment == pytest.approx(.12)
    assert message.backlash_tendon_distal_response_evidence == pytest.approx(
        1.0)
    assert message.backlash_tendon_distal_response_confirmed


@pytest.mark.parametrize(
    "transaction_state, expect_reset",
    [
        (TakeupTransactionArbiter.REPLAN_REQUIRED, False),
        (TakeupTransactionArbiter.SATURATED_REPLAN, True),
    ])
def test_takeup_replan_preserves_memory_except_after_saturation(
        transaction_state, expect_reset):
    class FakeArbiter:
        def __init__(self):
            self.state = transaction_state

        def release_for_replan(self):
            self.state = TakeupTransactionArbiter.READY_TO_PLAN
            return True

    planner = SimpleNamespace(reset_calls=0)

    def reset():
        planner.reset_calls += 1

    planner.reset = reset
    previous = np.array([1.0, -2.0, 3.0, 0.0, 0.0, 0.0])
    fake = SimpleNamespace(
        takeup_arbiter=FakeArbiter(),
        planner=planner,
        last_effective_command=previous.copy())

    released, completed = (
        CatheterControlNode._release_takeup_for_replan_locked(fake))

    assert released
    assert completed == transaction_state
    assert planner.reset_calls == int(expect_reset)
    np.testing.assert_array_equal(
        fake.last_effective_command,
        np.zeros(6) if expect_reset else previous)


def test_saturation_direction_block_expires_after_fresh_engagement():
    released = []
    fake = SimpleNamespace(
        blocked_motor_direction=np.array([1, 0, -1], dtype=np.int8),
        takeup_arbiter=SimpleNamespace(
            physical_direction_vector_is_feasible=lambda *args: False),
        backlash_snapshot=SimpleNamespace(
            phase=("ENGAGED", "TAKEUP", "ENGAGED"),
            remaining_rad=np.zeros(3)),
        contract=object(),
        _clear_takeup_saturation_locked=released.append)

    CatheterControlNode._refresh_blocked_motor_directions_locked(
        fake, np.zeros(6))

    assert released == ["engagement_revalidated"]
