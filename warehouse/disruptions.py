"""Disruption events and scenario generation."""
import math
import random
from dataclasses import dataclass

from .agent import Agent


@dataclass
class Event:
    t: int
    kind: str                 # 'blockage' | 'breakdown' | 'emergency'
    duration: int = 0
    cell: tuple = None        # blockage cell (None = pick one on some robot's path at fire time)
    agent: int = None         # breakdown / emergency robot (None = pick at fire time)
    pickup: tuple = None      # emergency task
    drop: tuple = None
    seed: int = 0             # rng seed for choices made at fire time
    permanent: bool = False   # blockage / breakdown that never ends (duration ignored)

    def describe(self):
        if self.kind == "blockage":
            return f"cell {self.cell} blocked " + ("permanently" if self.permanent else f"for {self.duration}")
        if self.kind == "breakdown":
            return f"robot {self.agent} broke down " + ("permanently" if self.permanent else f"for {self.duration}")
        return f"robot {self.agent} emergency {self.pickup}->{self.drop}"


def make_agents(grid, n_agents, tasks_per_agent, rng):
    if n_agents > len(grid.homes):
        raise ValueError(f"map has only {len(grid.homes)} home cells")
    homes = rng.sample(grid.homes, n_agents)
    agents = []
    tid = 0
    for i, home in enumerate(homes):
        tasks = []
        for _ in range(tasks_per_agent):
            tasks.append((tid, rng.choice(grid.pickups), rng.choice(grid.stations)))
            tid += 1
        agents.append(Agent(i, home, tasks))
    return agents


def _poisson(rng, lam):
    # Knuth's method; lam is small here
    if lam <= 0:
        return 0
    L, k, p = math.exp(-lam), 0, 1.0
    while True:
        p *= rng.random()
        if p <= L:
            return k
        k += 1


def make_events(grid, n_agents, rng, density=0.01, horizon=300, block_dur=(10, 20),
                n_breakdowns=2, n_emergencies=1, window=(10, 80), fail_dur=(10, 20)):
    """Random disruption schedule.

    density: expected fraction of free (non-home) cells that are blocked at
    any instant. Blockages arrive as a Poisson process with rate
    density * n_free / mean_duration per step, so the steady-state number of
    simultaneously blocked cells is density * n_free.
    """
    homes = set(grid.homes)
    free = [c for c in grid.cells if c not in homes]
    mean_dur = (block_dur[0] + block_dur[1]) / 2
    lam = density * len(free) / mean_dur
    events = []
    for t in range(1, horizon):
        for _ in range(_poisson(rng, lam)):
            events.append(Event(t, "blockage", rng.randint(*block_dur), cell=rng.choice(free)))
    for _ in range(n_breakdowns):
        events.append(Event(rng.randint(*window), "breakdown", rng.randint(*fail_dur),
                            agent=rng.randrange(n_agents)))
    for _ in range(n_emergencies):
        events.append(Event(rng.randint(*window), "emergency", agent=rng.randrange(n_agents),
                            pickup=rng.choice(grid.pickups), drop=rng.choice(grid.stations)))
    events.sort(key=lambda e: e.t)
    return events


def single_event(kind, rng, window=(10, 30), dur=(10, 20)):
    """One disruption whose target is chosen at fire time so that it really
    hits a moving robot (used to measure the impact of a single disruption)."""
    return Event(rng.randint(*window), kind, rng.randint(*dur) if kind != "emergency" else 0,
                 seed=rng.randrange(1 << 30))
