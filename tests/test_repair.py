import random

from warehouse.agent import Agent
from warehouse.disruptions import Event, make_agents, make_events
from warehouse.grid import Grid, make_warehouse
from warehouse.planner import first_invalid, prioritized_plan
from warehouse.repair import RepairEngine
from warehouse.reservation import ReservationTable
from warehouse.simulator import Simulator


def test_blockage_changes_only_the_blocked_agent():
    g = Grid(7, 7, shelves=(), homes=[(0, 0), (0, 6), (6, 0), (6, 6)])
    a = Agent(0, (0, 0), [(0, (3, 3), (6, 6))])
    b = Agent(1, (0, 6), [(1, (0, 5), (0, 4))])
    sim = Simulator(g, [a, b], [], check=True)
    prioritized_plan(g, sim.rt, sim.agents, 0)  # initial plan only, then inject the event
    b_before = list(b.path)
    cell = a.path[3]
    rec = sim.fire(Event(1, "blockage", 5, cell=cell), 1)
    assert rec.disrupted == [0]
    assert rec.changed == [0]
    assert b.path == b_before
    assert first_invalid(g, sim.rt, 0, a.path, 1) is None


def corridor():
    # row 1 is a one-wide corridor with a niche above and below column 3
    shelves = {(0, c) for c in range(7) if c != 3} | {(2, c) for c in range(7) if c != 3}
    return Grid(3, 7, shelves)


def test_negotiation_moves_only_the_blocking_neighbour():
    g = corridor()
    rt = ReservationTable()
    A = Agent(0, (1, 6), [])            # must get from (1,0) to its home (1,6)
    B = Agent(1, (1, 5), [])            # parked in the corridor, in A's way
    D = Agent(2, (2, 3), [])            # parked in the lower niche, not in the way
    A.path = [(1, c) for c in range(7)]  # original plan drives straight through B
    for x in (B, D):
        x.legs = []
        rt.set_path(x.id, x.path, 0)
    rt.set_path(A.id, A.path, 0)
    d_before = list(D.path)
    eng = RepairEngine(g, rt, [A, B, D], radius=6)
    eng.begin()
    assert first_invalid(g, rt, 0, A.path, 0) is not None
    assert eng.repair_agent(A, 0, A.path)
    levels = dict(eng.levels)
    assert levels[0].startswith("L2") and levels[1] == "yield"
    assert D.path == d_before           # untouched
    assert B.path[-1] == B.home and (0, 3) in B.path  # stepped into the niche and came back
    for x in (A, B, D):
        assert first_invalid(g, rt, x.id, x.path, 0) is None
    assert eng.messages > 0


def test_rollback_restores_everything():
    g = corridor()
    rt = ReservationTable()
    A, B = Agent(0, (1, 6), []), Agent(1, (1, 5), [])
    A.path = [(1, c) for c in range(7)]
    B.legs = []
    rt.set_path(1, B.path, 0)
    rt.set_path(0, A.path, 0)
    eng = RepairEngine(g, rt, [A, B])
    eng.begin()
    m = eng.mark()
    eng.commit(B, [(1, 5), (1, 4), (1, 3), (0, 3)], 0, "yield")
    eng.rollback(m)
    assert B.path == [(1, 5)] and rt.record(1)[0] == [(1, 5)]
    assert eng.levels == []


def run(strategy, n, density, seed):
    rng = random.Random(seed)
    g = make_warehouse()
    agents = make_agents(g, n, 3, rng)
    events = make_events(g, n, random.Random(seed + 1), density=density,
                         n_breakdowns=3, n_emergencies=2)
    return Simulator(g, agents, events, strategy=strategy, seed=seed, check=True).run()


def test_full_simulation_is_collision_free_and_completes():
    for strategy in ("repair", "replan"):
        r = run(strategy, 25, 0.03, 5)
        assert r.collisions == 0
        assert not r.timeout
        assert r.tasks_done == r.tasks_total
        assert len(r.records) > 10


def test_repair_alters_fewer_plans_than_full_replan():
    rep = run("repair", 30, 0.01, 2).summary()
    base = run("replan", 30, 0.01, 2).summary()
    assert rep["mean_changed"] < base["mean_changed"]
    assert rep["fallbacks"] == 0


def test_permanent_breakdown_drops_only_unfinishable_tasks():
    rng = random.Random(3)
    g = make_warehouse()
    agents = make_agents(g, 20, 3, rng)
    events = [Event(20, "breakdown", permanent=True, agent=4),
              Event(10, "blockage", permanent=True, cell=g.stations[0])]
    r = Simulator(g, agents, events, seed=3, check=True).run()
    assert r.collisions == 0 and not r.timeout and not r.halted
    assert r.robots_lost == 1 and agents[4].lost == "dead"
    assert r.tasks_lost > 0 and r.tasks_done + r.tasks_lost == r.tasks_total
    assert set(r.lost_by) <= {"dead", "trapped", "cell blocked", "walled off"}


def test_purely_local_repair_halts_cleanly_when_stuck():
    rng = random.Random(1)
    g = make_warehouse()
    agents = make_agents(g, 40, 3, rng)
    events = make_events(g, 40, random.Random(2), density=0.2, n_breakdowns=0, n_emergencies=0)
    r = Simulator(g, agents, events, seed=1, fallback=False, check=True).run()
    assert r.halted and r.halt_t is not None
    assert r.tasks_done < r.tasks_total
    assert r.collisions == 0
    assert sum(rec.fallback for rec in r.records) == 0  # the global planner was never called
