"""Local, negotiation-based plan repair.

Repair never re-runs the multi-agent planner. For each agent whose plan a
disruption made invalid it escalates:

  L0 wait      keep the original route, insert waits until it is valid again
  L1 detour    single-agent Space-Time A* against everybody else's plans
  L2 negotiate ask neighbours within a communication radius to give way:
               plan with the neighbours' reservations as soft constraints
               (cost = time + penalty * conflicts), then every neighbour hit
               by the new path must replan itself (L0/L1, recursively L2 up
               to a depth limit). All-or-nothing: on any refusal the whole
               coalition is rolled back and the radius is widened.

Only the disrupted agent and the neighbours that accept a proposal change
their plans; everyone else keeps their original plan untouched.
"""
from dataclasses import dataclass

from .planner import conflicting_agents, first_invalid, plan_route


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


@dataclass
class Candidate:
    path: list
    level: str
    delay: int


class RepairEngine:
    def __init__(self, grid, rt, agents, radius=4, depth=2, tau=8, yield_cap=30,
                 emergency_tol=2, emergency_penalty=1.0, max_waits=40):
        self.grid, self.rt = grid, rt
        self.agents = {a.id: a for a in agents}
        self.radius, self.depth, self.tau = radius, depth, tau
        self.yield_cap = yield_cap
        self.emergency_tol = emergency_tol
        self.emergency_penalty = emergency_penalty
        self.max_waits = max_waits
        self.log = []      # undo log of commits: (agent, old_path, old_reservation)
        self.levels = []   # (aid, level) in commit order
        self.messages = 0

    # ---- transactions -------------------------------------------------------
    def commit(self, a, path, t, level, park=True):
        self.log.append((a, a.path, self.rt.record(a.id)))
        self.levels.append((a.id, level))
        a.path = path
        self.rt.set_path(a.id, path, t, park=park)

    def mark(self):
        return len(self.log), len(self.levels)

    def rollback(self, mark):
        n_log, n_lev = mark
        while len(self.log) > n_log:
            a, old_path, rec = self.log.pop()
            a.path = old_path
            if rec is None:
                self.rt.clear(a.id)
            else:
                self.rt.set_path(a.id, rec[0], rec[1], park=rec[2])
        del self.levels[n_lev:]

    def begin(self):
        self.log.clear()
        self.levels.clear()
        self.messages = 0

    # ---- single-agent repair (L0 / L1) ---------------------------------------
    def wait_insert(self, a, path, ts):
        path = list(path)
        start = ts
        for _ in range(self.max_waits + 1):
            k = first_invalid(self.grid, self.rt, a.id, path, start)
            if k is None:
                return path
            if k >= len(path) or path[k] == path[k - 1]:
                return None  # parking clash, or someone runs into us while waiting
            path.insert(k, path[k - 1])
            start = max(ts, k - 2)
        return None

    def solo(self, a, t, old_path, allow_wait=True):
        """Best of L0 (wait) and L1 (detour) for `a`, touching nobody else."""
        ts = max(t, a.broken_until)
        pre = a.prefix(ts)
        old_end = max(len(old_path) - 1, ts)
        best = None
        if allow_wait:
            p = self.wait_insert(a, pre + old_path[t + 1:], ts)
            if p is not None:
                best = Candidate(p, "L0 wait", len(p) - 1 - old_end)
        res = plan_route(self.grid, self.rt, a.id, pre[-1], ts, a.planning_legs())
        if res is not None:
            p = pre[:-1] + res[0]
            if best is None or len(p) < len(best.path):
                best = Candidate(p, "L1 detour", len(p) - 1 - old_end)
        return best

    # ---- negotiation (L2) -----------------------------------------------------
    def negotiate(self, a, t, depth, coalition, penalty, radius, level="L2 negotiate"):
        ts = max(t, a.broken_until)
        pre = a.prefix(ts)
        me = pre[-1]
        nbrs = frozenset(
            b.id for b in self.agents.values()
            if b.id != a.id and b.id not in coalition and not b.is_broken(t)
            and manhattan(me, b.pos(t)) <= radius
        )
        self.messages += len(nbrs)                       # REQUEST broadcast
        if not nbrs:
            return False
        res = plan_route(self.grid, self.rt, a.id, me, ts, a.planning_legs(),
                         soft=nbrs, penalty=penalty)
        if res is None:
            return False
        path = pre[:-1] + res[0]
        conflicted = conflicting_agents(self.grid, self.rt, a.id, path, t)
        m = self.mark()
        self.commit(a, path, t, level)
        coalition = coalition | {a.id} | conflicted
        for bid in sorted(conflicted, key=lambda i: manhattan(me, self.agents[i].pos(t))):
            b = self.agents[bid]
            self.messages += 2                           # PROPOSE + ACCEPT/REJECT
            if first_invalid(self.grid, self.rt, b.id, b.path, t) is None:
                continue                                 # already out of the way
            cand = self.solo(b, t, b.path)
            if cand is not None and cand.delay <= self.yield_cap:
                self.commit(b, cand.path, t, "yield")
                continue
            if depth > 0 and self.negotiate(b, t, depth - 1, coalition, self.tau, radius,
                                            level="yield+L2"):
                continue
            self.rollback(m)
            return False
        return True

    # ---- entry point ------------------------------------------------------------
    def repair_agent(self, a, t, old_path, emergency=False):
        """Make a's plan valid again. Returns False if every local option failed."""
        cand = self.solo(a, t, old_path, allow_wait=not emergency)
        radii = (self.radius, 2 * self.radius, 4 * self.radius)
        if emergency:
            ts = max(t, a.broken_until)
            lb, cur = ts, a.pos(ts)
            for leg in a.planning_legs():
                lb += self.grid.dist(cur, leg.cell) + (leg.kind != "home")
                cur = leg.cell
            if cand is not None and len(cand.path) - 1 - lb <= self.emergency_tol:
                self.commit(a, cand.path, t, cand.level)
                return True
            for r in radii:
                if self.negotiate(a, t, self.depth, frozenset(), self.emergency_penalty, r,
                                  level="L2 priority"):
                    return True
        else:
            if cand is not None and cand.delay <= self.tau:
                self.commit(a, cand.path, t, cand.level)
                return True
            for r in radii:
                if self.negotiate(a, t, self.depth, frozenset(), self.tau, r):
                    return True
        if cand is not None:
            self.commit(a, cand.path, t, cand.level)
            return True
        # last local resort: everybody may be asked, deeper chains
        return self.negotiate(a, t, self.depth + 1, frozenset(), self.tau, 10 ** 6)
