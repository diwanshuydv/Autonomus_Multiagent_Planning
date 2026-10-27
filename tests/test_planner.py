import random

from warehouse.agent import Leg
from warehouse.disruptions import make_agents
from warehouse.grid import Grid, make_warehouse
from warehouse.planner import plan_route, prioritized_plan, space_time_astar
from warehouse.reservation import ReservationTable


def collisions(agents, horizon):
    bad = 0
    for t in range(1, horizon):
        pos = [a.pos(t) for a in agents]
        bad += len(pos) - len(set(pos))
        for i, a in enumerate(agents):
            for b in agents[i + 1:]:
                if a.pos(t - 1) == b.pos(t) and b.pos(t - 1) == a.pos(t) and a.pos(t) != a.pos(t - 1):
                    bad += 1
    return bad


def test_astar_optimal_on_empty_grid():
    g = Grid(5, 5, shelves=())
    rt = ReservationTable()
    seq, c = space_time_astar(g, rt, 0, (0, 0), 0, (4, 4), "home")
    assert len(seq) - 1 == 8 and c == 0


def test_service_dwell_added():
    g = Grid(1, 4, shelves=())
    rt = ReservationTable()
    path, _ = plan_route(g, rt, 0, (0, 0), 0, [Leg((0, 3), "pickup"), Leg((0, 0), "home")])
    assert path == [(0, 0), (0, 1), (0, 2), (0, 3), (0, 3), (0, 2), (0, 1), (0, 0)]


def test_vertex_and_swap_reservations_respected():
    g = Grid(2, 5, shelves=())
    rt = ReservationTable()
    # agent 1 drives along row 0 from right to left
    other = [(0, 4), (0, 3), (0, 2), (0, 1), (0, 0)]
    rt.set_path(1, other, 0)
    seq, _ = space_time_astar(g, rt, 0, (0, 0), 0, (0, 4), "home")
    for t in range(1, len(seq)):
        assert seq[t] != (other[t] if t < len(other) else other[-1])
        assert not (seq[t - 1] == other[min(t, 4)] and seq[t] == other[min(t - 1, 4)])


def test_soft_reservations_are_counted():
    g = Grid(1, 3, shelves=())
    rt = ReservationTable()
    rt.set_path(1, [(0, 1)], 0)  # parked in the only corridor
    assert space_time_astar(g, rt, 0, (0, 0), 0, (0, 2), "home") is None
    seq, c = space_time_astar(g, rt, 0, (0, 0), 0, (0, 2), "home", soft=frozenset({1}), penalty=5)
    assert seq[-1] == (0, 2) and c >= 1


def test_initial_plan_collision_free():
    rng = random.Random(3)
    g = make_warehouse()
    agents = make_agents(g, 30, 3, rng)
    rt = ReservationTable()
    prioritized_plan(g, rt, agents, 0, rng=rng)
    horizon = max(len(a.path) for a in agents) + 2
    assert collisions(agents, horizon) == 0
    for a in agents:
        assert a.path[0] == a.home and a.path[-1] == a.home
