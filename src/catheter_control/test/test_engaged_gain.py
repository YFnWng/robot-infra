import numpy as np
import pytest

from catheter_control.engaged_gain import EngagedGainConfig, EngagedGainEstimator


def test_gain_updates_only_after_confirmed_engagement():
    estimator = EngagedGainEstimator(EngagedGainConfig(
        enabled=True, observation_std=0.02,
        minimum_nominal_increment=0.01))

    assert not estimator.observe(2, 1, 0.0, 0.0, 1, engaged=False)
    assert estimator.snapshot().update_count[2, 1] == 0

    # First credible engaged sample is an anchor, not a fabricated gain datum.
    assert not estimator.observe(2, 1, 0.0, 0.0, 2, engaged=True)
    for index in range(1, 16):
        estimator.observe(
            2, 1, 0.1*index, 0.06*index, 2+index*50_000_000,
            engaged=True)

    belief = estimator.snapshot()
    assert belief.update_count[2, 1] > 5
    assert 0.45 < belief.mean[2, 1] < 0.80
    assert belief.lower[2, 1] <= belief.mean[2, 1] <= belief.upper[2, 1]
    assert belief.status[2][1] in ("LEARNING", "CONFIDENT")


def test_directional_gains_are_independent_and_reversal_inflates_uncertainty():
    estimator = EngagedGainEstimator(EngagedGainConfig(
        enabled=True, prior_log_std=0.2, reversal_log_std=0.8,
        observation_std=0.02, minimum_nominal_increment=0.01))
    estimator.observe(2, 1, 0.0, 0.0, 1, engaged=True)
    estimator.observe(2, 1, 0.2, 0.1, 2, engaged=True)
    before = estimator.snapshot()
    negative_before = before.mean[2, 0]

    estimator.start_takeup(2, -1)
    after = estimator.snapshot()
    assert after.mean[2, 0] == pytest.approx(negative_before)
    assert after.log_variance[2, 0] >= 0.8**2
    assert after.update_count[2, 1] == before.update_count[2, 1]
    assert after.last_reason[2] == "awaiting_engagement"


def test_gain_state_is_rewindable():
    estimator = EngagedGainEstimator(EngagedGainConfig(
        enabled=True, observation_std=0.02,
        minimum_nominal_increment=0.01))
    estimator.observe(2, 1, 0.0, 0.0, 1, engaged=True)
    checkpoint = estimator.clone_state()
    estimator.observe(2, 1, 0.2, 0.1, 2, engaged=True)
    assert estimator.snapshot().update_count[2, 1] == 1

    estimator.restore_state(checkpoint)
    restored = estimator.snapshot()
    assert restored.update_count[2, 1] == 0
    assert np.isnan(estimator.last_nominal[0])


def test_rejected_gain_innovation_widens_and_reanchors():
    estimator = EngagedGainEstimator(EngagedGainConfig(
        enabled=True, observation_std=0.02,
        minimum_nominal_increment=0.01,
        maximum_normalized_innovation=4.0,
        contradiction_log_std=0.7))
    estimator.confirm(2, 1, 0.0, 0.0, 1)
    estimator.log_variance[2, 1] = 1e-4
    estimator.update_count[2, 1] = 20

    assert not estimator.observe(
        2, 1, 0.1, 1.0, 2, engaged=True)
    rejected = estimator.snapshot()
    assert rejected.last_reason[2] == "innovation_rejected"
    assert rejected.log_variance[2, 1] >= 0.7**2
    assert rejected.update_count[2, 1] == 0
    assert estimator.last_nominal[2] == pytest.approx(0.1)
    assert estimator.last_observed[2] == pytest.approx(1.0)

    # The next datum is compared with the new local anchor, not the stale
    # pre-transient sample.
    assert estimator.observe(
        2, 1, 0.2, 1.1, 3, engaged=True)
    assert estimator.snapshot().update_count[2, 1] == 1
