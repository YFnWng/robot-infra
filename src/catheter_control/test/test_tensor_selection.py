import pytest
import torch

from catheter_control.planning.tensor_selection import weighted_selection


@pytest.mark.parametrize("count", [1, 8, 512])
@pytest.mark.parametrize("temperature", [.1, 1., 3.])
@pytest.mark.parametrize("fixed_shape", [False, True])
def test_weighting_matches_original(count, temperature, fixed_shape):
    generator = torch.Generator().manual_seed(71)
    costs = torch.randn(count, generator=generator)*100
    tips = torch.randn(count, 4, 3, generator=generator)
    logical = torch.randn(count, 4, 6, generator=generator)
    eligible = torch.arange(count) % 3 != 1
    selection = torch.where(eligible, costs, torch.inf)
    scale = costs[eligible].std(unbiased=False).clamp_min(1e-6)
    logits = -(costs-selection.min())/(temperature*scale)
    expected = torch.softmax(torch.where(eligible, logits, -torch.inf), 0)
    best, weights, weighted_tips, feasible = weighted_selection(
        costs, eligible, tips, logical, temperature, fixed_shape=fixed_shape)
    assert int(best) == int(selection.argmin())
    torch.testing.assert_close(weights, expected)
    torch.testing.assert_close(weighted_tips, (expected[:, None, None]*tips).sum(0))
    torch.testing.assert_close(feasible, (expected[:, None, None]*logical[..., :3]).sum(0))


def test_tie_uses_first_candidate_and_equal_costs():
    best, weights, _, _ = weighted_selection(
        torch.ones(8), torch.ones(8, dtype=torch.bool),
        torch.zeros(8, 4, 3), torch.zeros(8, 4, 6), 1.)
    assert int(best) == 0
    torch.testing.assert_close(weights, torch.ones(8)/8)
