"""Space-Time A* and prioritized (cooperative) multi-agent planning."""
import heapq
import itertools
import random

from .grid import INF


class PlanningFailure(Exception):
    pass


def _goal_extra(grid, rt, aid, goal, kind, t, soft):
    """Conflicts added by finishing a leg at (goal, t), or None if impossible.

    pickup/drop need one dwell step at the goal (t+1); 'home' needs the cell
    to stay free forever so the robot can park.
    """
    if kind == "home":
        if grid.blocked_after(goal, t):
            return None
        owners = rt.future_owners(goal, t, aid)
    else:
        if grid.is_blocked(goal, t + 1):
            return None
        owners = rt.vertex_owners(goal, t + 1, aid)
    if not owners:
        return 0
    if soft and owners <= soft:
        return len(owners)
    return None


def space_time_astar(grid, rt, aid, start, t0, goal, kind, soft=frozenset(),
                     penalty=0.0, max_expand=500_000):
    """Single-agent A* over (cell, time) states. Actions: 4 moves + wait.

    Reservations of agents in `soft` may be violated at `penalty` per
    conflict (used during negotiation); everything else is a hard constraint.
    Returns (cells from t0 to leg completion, n_conflicts) or None.
    """
    dist = grid.dist_table(goal)
    h0 = dist.get(start, INF)
    if h0 == INF:
        return None
    # After t_static nothing in the world changes any more, so (cell, t) and
    # (cell, t') are equivalent: collapsing them bounds the search.
    t_static = max(rt.max_t, grid.max_block_t, t0) + 1
    tie = itertools.count()
    # heap entry: (f, conflicts, -t, tie, cell, t, conflicts, parent_node)
    heap = [(h0, 0, -t0, next(tie), start, t0, 0, None)]
    closed = set()
    expanded = 0
    while heap:
        _, _, _, _, cell, t, c, parent = heapq.heappop(heap)
        key = (cell, t if t < t_static else t_static)
        if key in closed:
            continue
        closed.add(key)
        node = (cell, parent)
        if cell == goal:
            extra = _goal_extra(grid, rt, aid, goal, kind, t, soft)
            if extra is not None:
                seq = []
                n = node
                while n is not None:
                    seq.append(n[0])
                    n = n[1]
                seq.reverse()
                if kind != "home":
                    seq.append(goal)  # service dwell
                return seq, c + extra
        expanded += 1
        if expanded > max_expand:
            return None
        nt = t + 1
        for nc in grid.neighbors(cell) + [cell]:
            nkey = (nc, nt if nt < t_static else t_static)
            if nkey in closed or grid.is_blocked(nc, nt):
                continue
            add = 0
            owners = rt.vertex_owners(nc, nt, aid)
            if owners:
                if not (soft and owners <= soft):
                    continue
                add += len(owners)
            if nc != cell:
                owners = rt.swap_owners(cell, nc, t, aid)
                if owners:
                    if not (soft and owners <= soft):
                        continue
                    add += len(owners)
            nc_c = c + add
            f = (nt - t0) + dist.get(nc, INF) + penalty * nc_c
            heapq.heappush(heap, (f, nc_c, -nt, next(tie), nc, nt, nc_c, node))
    return None


def plan_route(grid, rt, aid, start, t0, legs, soft=frozenset(), penalty=0.0):
    """Chain STA* over the agent's remaining legs.

    Returns (cells from t0 until parked at home, total conflicts) or None.
    """
    path = [start]
    total = 0
    for leg in legs:
        res = space_time_astar(grid, rt, aid, path[-1], t0 + len(path) - 1, leg.cell,
                               leg.kind, soft, penalty)
        if res is None:
            return None
        seg, c = res
        path.extend(seg[1:])
        total += c
    return path, total


def conflicting_agents(grid, rt, aid, path, t_from, park=True):
    """Other agents whose reservations clash with path[t_from+1:]."""
    out = set()
    for t in range(t_from + 1, len(path)):
        o = rt.vertex_owners(path[t], t, aid)
        if o:
            out |= o
        if path[t - 1] != path[t]:
            o = rt.swap_owners(path[t - 1], path[t], t - 1, aid)
            if o:
                out |= o
    if park:
        o = rt.future_owners(path[-1], len(path) - 1, aid)
        if o:
            out |= o
    return out


def first_invalid(grid, rt, aid, path, t_from, park=True):
    """First time index > t_from where path is infeasible, len(path) for a
    parking conflict, or None if the whole suffix is valid."""
    for t in range(t_from + 1, len(path)):
        cell = path[t]
        if grid.is_blocked(cell, t) or rt.vertex_owners(cell, t, aid):
            return t
        if path[t - 1] != cell and rt.swap_owners(path[t - 1], cell, t - 1, aid):
            return t
    if park:
        goal = path[-1]
        if grid.blocked_after(goal, len(path) - 1) or rt.future_owners(goal, len(path) - 1, aid):
            return len(path)
    return None


def _reset(rt, a, t):
    """Drop a's plan from time t; a broken robot keeps its forced stay."""
    if a.broken_until > t:
        rt.set_path(a.id, a.prefix(a.broken_until), t, park=False)
    else:
        rt.clear(a.id)


def prioritized_plan(grid, rt, agents, t, rng=None, retries=10):
    """Cooperative A*: agents plan one after another (list order = priority)
    from their state at time t, each treating earlier plans as moving
    obstacles. On failure the failing agent is promoted and the rest shuffled.
    Returns the number of retries used.
    """
    rng = rng or random.Random(0)
    order = list(agents)
    for a in order:
        _reset(rt, a, t)
    for attempt in range(retries):
        failed = None
        planned = []
        for a in order:
            ts = max(t, a.broken_until)
            res = plan_route(grid, rt, a.id, a.pos(ts), ts, a.planning_legs())
            if res is None:
                failed = a
                break
            a.path = a.prefix(ts)[:ts] + res[0]
            rt.set_path(a.id, a.path, t)
            planned.append(a)
        if failed is None:
            return attempt
        for a in planned:
            _reset(rt, a, t)
        rest = [x for x in order if x is not failed]
        rng.shuffle(rest)
        order = [failed] + rest
    raise PlanningFailure(f"prioritized planning failed at t={t}")
