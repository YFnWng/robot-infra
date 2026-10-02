"""Fixed-shape MPPI weighting; eligibility and capture remain CPU policy."""
import torch


def weighted_selection(costs, eligible, mean_tips, logical, temperature, *, fixed_shape=False):
    """Return the already-scored winner and feasible weighted proposal.

    The caller must validate that at least one candidate is eligible, all
    costs are finite, and temperature is positive before executing a command.
    No projections, reversal approvals, capture rules, or safety edits here.
    """
    selection_costs = torch.where(eligible, costs, torch.full_like(costs, torch.inf))
    best = torch.argmin(selection_costs)
    # Masked population std is equivalent to costs[eligible].std(unbiased=False)
    # but has no data-dependent output shape, permitting fixed-shape execution.
    if fixed_shape:
        count = eligible.sum().clamp_min(1)
        center = torch.where(eligible, costs, 0).sum()/count
        variance = torch.where(eligible, (costs-center).square(), 0).sum()/count
        scale = variance.sqrt().clamp_min(1e-6)
    else:
        # Preserve the reference reduction/order for the unpromoted ROS path.
        scale = costs[eligible].std(unbiased=False).clamp_min(1e-6)
    logits = -(costs-selection_costs.min())/(temperature*scale)
    weights = torch.softmax(torch.where(
        eligible, logits, torch.full_like(logits, -torch.inf)), dim=0)
    tips = (weights[:, None, None]*mean_tips).sum(0)
    feasible = (weights[:, None, None]*logical[..., :3]).sum(0)
    return best, weights, tips, feasible
