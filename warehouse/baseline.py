"""Baseline for comparison: re-run the whole prioritized MAPF after a disruption."""
from .planner import prioritized_plan


def full_replan(grid, rt, agents, t, rng, first=None, retries=10):
    """Replan every robot that still has work, from its state at time t.

    Priority: the emergency robot (if any), broken robots (their stay is
    fixed), then by id - the same order as the initial plan.
    Parked robots keep their reservations.
    """
    movable = [a for a in agents if a.legs]
    order = sorted(movable, key=lambda a: (a is not first, not a.is_broken(t + 1), a.id))
    return prioritized_plan(grid, rt, order, t, rng=rng, retries=retries)
