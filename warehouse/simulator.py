"""Discrete-time execution of the plans with disruptions and repair."""
import copy
import random
import time

from .agent import Leg
from .baseline import full_replan
from .grid import INF
from .metrics import RepairRecord, RunResult
from .planner import PlanningFailure, first_invalid, prioritized_plan
from .repair import RepairEngine
from .reservation import ReservationTable


PERMANENT = 10 ** 9  # broken_until of a robot that never recovers


class Simulator:
    """strategy='repair'  -> local negotiation-based repair (RepairEngine)
    strategy='replan'  -> baseline: whole prioritized MAPF re-run per disruption

    fallback=True lets local repair call the global planner as a last resort
    when every local option failed. With fallback=False the system is purely
    local: if no conflict-free repair exists, the run halts and the tasks not
    delivered by then count as failed.
    """

    def __init__(self, grid, agents, events, strategy="repair", seed=0, density=0.0,
                 record=False, t_max=1500, check=False, max_defer=30, fallback=True, **engine_kw):
        self.grid, self.agents = grid, agents
        self.by_id = {a.id: a for a in agents}
        self.events = copy.deepcopy(events)
        self.strategy = strategy
        self.rng = random.Random(seed)
        self.rt = ReservationTable()
        self.engine = RepairEngine(grid, self.rt, agents, **engine_kw)
        self.homes = set(grid.homes)
        self.record, self.frames = record, []
        self.t_max, self.check, self.max_defer = t_max, check, max_defer
        self.fallback = fallback
        self.result = RunResult(strategy, len(agents), density, seed)
        self._n_emerg = 0

    # ------------------------------------------------------------------ run
    def run(self):
        prioritized_plan(self.grid, self.rt, self.agents, 0, rng=self.rng, retries=30)
        for a in self.agents:
            a.original_path = list(a.path)
        res = self.result
        res.initial_sum_home = sum(len(a.path) - 1 for a in self.agents)
        res.tasks_total = sum(len(a.tasks) for a in self.agents)
        queue = list(self.events)
        deferred = {}
        t = 0
        self._frame(0, [], set())
        while True:
            fired, changed = [], set()
            due = [e for e in queue if e.t <= t]
            queue = [e for e in queue if e.t > t]
            for ev in due:
                out = self.fire(ev, t)
                if out == "defer":
                    deferred[id(ev)] = deferred.get(id(ev), 0) + 1
                    if deferred[id(ev)] <= self.max_defer:
                        ev.t = t + 1
                        queue.append(ev)
                elif out is not None:
                    res.records.append(out)
                    fired.append(f"{ev.describe()}: {out.n_changed} plan(s) changed")
                    changed |= set(out.changed)
                if res.halted:
                    break
            if res.halted:
                fired.append(f"FAILED: {res.halt_reason}")
                last = res.records[-1] if res.records else None
                stuck = set(last.disrupted) if last is not None and last.failed else set()
                self._frame(t, fired, stuck, failed=True)
                break
            t += 1
            if t > self.t_max:
                res.timeout = True
                break
            self.grid.expire(t)
            self._advance(t)
            res.collisions += self._validate(t)
            self._frame(t, fired, changed)
            if all(a.done for a in self.agents):
                break
        res.makespan = max(a.last_delivery for a in self.agents)
        res.sum_of_costs = sum(a.last_delivery for a in self.agents)
        res.sum_home = sum(max(a.home_time or 0, len(a.path) - 1) for a in self.agents)
        res.tasks_total = sum(len(a.tasks) for a in self.agents)
        return res

    # --------------------------------------------------------------- events
    def fire(self, ev, t):
        rng = random.Random(ev.seed * 7919 + t)
        before = {a.id: a.future(t) for a in self.agents}
        t0 = time.perf_counter()
        special = None  # (agent, old_path, is_emergency) that must replan itself
        if ev.kind == "blockage":
            cell = ev.cell if ev.cell is not None else self._pick_block_cell(rng, t)
            if cell is None or any(a.pos(t) == cell for a in self.agents):
                return "defer"
            ev.cell = cell
            if ev.permanent:
                self.grid.add_block(cell, t, INF)
                victims = self._permanent_victims(t)
            else:
                self.grid.add_block(cell, t, t + ev.duration)
                victims = self._owners(cell, range(t + 1, t + ev.duration), None)
        elif ev.kind == "breakdown":
            a = self.by_id[ev.agent] if ev.agent is not None else self._pick_agent(rng, t)
            if a is None or a.done or a.is_broken(t):
                return None  # nothing to break (robot idle/parked or already down)
            if ev.permanent and a.pos(t) in self.homes:
                return "defer"  # a dead robot on a home cell could lock another robot out
            ev.agent = a.id
            if ev.permanent:
                self._remove_robot(a, t, "dead")
                return self._handle(ev, t, t0, before, self._permanent_victims(t), None)
            old = list(a.path)
            pos = a.pos(t)
            a.broken_until = t + ev.duration
            a.path = a.prefix(t) + [pos] * ev.duration
            self.rt.set_path(a.id, a.path, t, park=False)
            victims = self._owners(pos, range(t + 1, t + ev.duration + 1), a.id)
            special = (a, old, False)
        else:  # emergency
            a = self.by_id[ev.agent] if ev.agent is not None else self._pick_agent(rng, t)
            if a is None or a.is_broken(t):
                return "defer"
            ev.agent = a.id
            ev.pickup = ev.pickup or rng.choice(self.grid.pickups)
            ev.drop = ev.drop or rng.choice(self.grid.stations)
            old = list(a.path)
            tid = ("E", self._n_emerg)
            self._n_emerg += 1
            new = [Leg(ev.pickup, "pickup", tid), Leg(ev.drop, "drop", tid)]
            if not a.legs:
                a.legs = new + [Leg(a.home, "home")]
                a.home_time = None
            else:
                i = 1 if a.legs[0].kind == "drop" else 0  # finish the current delivery first
                a.legs[i:i] = new
            a.tasks.append((tid, ev.pickup, ev.drop))
            a.emergency = True
            victims = []
            special = (a, old, True)
        return self._handle(ev, t, t0, before, victims, special)

    def _handle(self, ev, t, t0, before, victims, special):
        disrupted = list(victims) + ([special[0].id] if special else [])
        rec = RepairRecord(t, ev.kind, disrupted, [])
        if disrupted:
            if self.strategy == "repair":
                self._repair(t, victims, special, rec)
            else:
                self._replan(t, victims, special, rec)
        rec.cpu_ms = (time.perf_counter() - t0) * 1000
        rec.changed = [a.id for a in self.agents if a.future(t) != before[a.id]]
        for aid in rec.changed:
            self.by_id[aid].times_changed += 1
        if self.check and not rec.failed:
            self._check_consistent(t)
        return rec

    # ------------------------------------------------- permanent disruptions
    def _remove_robot(self, a, t, why):
        """Robot `a` leaves the fleet for good: it stays where it is forever (an
        obstacle for everyone) and its undelivered tasks are lost."""
        self._lose(why, sum(1 for leg in a.legs if leg.kind == "drop"))
        self.result.robots_lost += 1
        a.lost = why
        a.legs = []
        a.carrying.clear()
        a.broken_until = PERMANENT
        a.path = a.prefix(t)
        self.rt.set_path(a.id, a.path, t, park=True)

    def _drop_unreachable(self, t):
        """After a permanent change, drop the tasks no robot can ever finish.

        A pickup or drop cell is lost if it is permanently blocked (blockage or
        a dead robot) or walled off from the robot. A robot that can no longer
        reach its own home is walled in and leaves the fleet.
        Returns the robots whose task lists changed.
        """
        walls = self.grid.permanent_cells() | {a.pos(t) for a in self.agents if a.lost}
        out = []
        for a in self.agents:
            if a.lost or not a.legs:
                continue
            ts = max(t, a.broken_until)
            reach = self.grid.reachable(a.pos(ts), walls)
            if a.home not in reach:
                self._remove_robot(a, t, "trapped")
                out.append(a.id)
                continue
            bad = {leg.task: leg.cell in walls for leg in a.legs
                   if leg.kind != "home" and leg.cell not in reach}
            if bad:
                a.legs = [leg for leg in a.legs if leg.task not in bad]
                a.carrying -= set(bad)
                for on_wall in bad.values():
                    self._lose("cell blocked" if on_wall else "walled off", 1)
                out.append(a.id)
        return out

    def _lose(self, why, n):
        if n:
            self.result.tasks_lost += n
            self.result.lost_by[why] = self.result.lost_by.get(why, 0) + n

    def _permanent_victims(self, t):
        """Robots whose plan a permanent change made invalid, by first clash."""
        dropped = set(self._drop_unreachable(t))
        first = {}
        for a in self.agents:
            if a.lost:
                continue
            ts = max(t, a.broken_until)
            k = first_invalid(self.grid, self.rt, a.id, a.path, ts)
            if k is not None:
                first[a.id] = k
            elif a.id in dropped:
                first[a.id] = len(a.path)
        return sorted(first, key=first.get)

    def _replan(self, t, victims, special, rec):
        """Baseline. If the global planner finds no joint plan, restore the old plans
        and handle this event with local repair so the run can go on (rec.fallback)."""
        saved = {a.id: (a.path, self.rt.record(a.id)) for a in self.agents}
        try:
            full_replan(self.grid, self.rt, self.agents, t, self.rng,
                        first=special[0] if special and special[2] else None, retries=30)
        except PlanningFailure:
            for a in self.agents:
                a.path, rec_ = saved[a.id]
                if rec_ is None:
                    self.rt.clear(a.id)
                else:
                    self.rt.set_path(a.id, rec_[0], rec_[1], park=rec_[2])
            self._repair(t, victims, special, rec)
            rec.fallback = True

    def _repair(self, t, victims, special, rec):
        eng = self.engine
        eng.begin()
        ok = True
        for vid in victims:
            b = self.by_id[vid]
            if first_invalid(self.grid, self.rt, b.id, b.path, t) is None:
                continue  # already fixed while negotiating for another victim
            if not eng.repair_agent(b, t, b.path):
                ok = False
                break
        if ok and special:
            a, old, emergency = special
            ok = eng.repair_agent(a, t, old, emergency=emergency)
        rec.levels = dict(eng.levels)
        rec.messages = eng.messages
        if ok:
            return
        if self.fallback:  # every local option failed: fall back to a global replan
            rec.fallback = True
            try:
                full_replan(self.grid, self.rt, self.agents, t, self.rng,
                            first=special[0] if special and special[2] else None)
                return
            except PlanningFailure:
                reason = "neither local repair nor a global replan found a conflict-free plan"
        else:
            reason = "local repair found no conflict-free plan (global fallback disabled)"
        rec.failed = True
        res = self.result
        res.halted, res.halt_t, res.halt_reason = True, t, f"{reason} at t={t}"

    def _owners(self, cell, times, aid):
        """Agents (other than aid) holding a reservation on cell at any of `times`,
        ordered by the first clash."""
        first = {}
        for t in times:
            for o in self.rt.vertex_owners(cell, t, aid) or ():
                first.setdefault(o, t)
        return sorted(first, key=first.get)

    def _pick_agent(self, rng, t):
        cands = [a for a in self.agents if a.legs and not a.is_broken(t) and len(a.path) > t + 5]
        return rng.choice(cands) if cands else None

    def _pick_block_cell(self, rng, t):
        """A cell a moving robot is about to enter (so the blockage really disrupts)."""
        cands = [a for a in self.agents if not a.is_broken(t) and len(a.path) > t + 9]
        rng.shuffle(cands)
        occupied = {a.pos(t) for a in self.agents}
        for a in cands:
            cell = a.path[t + rng.randint(3, 8)]
            if cell not in self.homes and cell not in occupied and not self.grid.is_blocked(cell, t):
                return cell
        return None

    # ------------------------------------------------------------ execution
    def _advance(self, t):
        """Robots are at path[t]; complete pickups/deliveries (1-step dwell)."""
        for a in self.agents:
            if not a.legs:
                continue
            leg, p = a.legs[0], a.pos(t)
            if leg.kind == "home":
                if p == a.home and t >= len(a.path) - 1:
                    a.legs.pop(0)
                    a.home_time = t
                    a.emergency = False
            elif p == leg.cell and a.pos(t - 1) == leg.cell:
                a.legs.pop(0)
                if leg.kind == "pickup":
                    a.carrying.add(leg.task)
                else:
                    a.carrying.discard(leg.task)
                    a.last_delivery = t
                    self.result.tasks_done += 1

    def _validate(self, t):
        bad = 0
        at, prev = {}, {}
        for a in self.agents:
            p = a.pos(t)
            if p in at:
                bad += 1
            at[p] = a
            prev[a.pos(t - 1)] = a
            if self.grid.is_blocked(p, t):
                bad += 1
        for a in self.agents:
            u, v = a.pos(t - 1), a.pos(t)
            b = prev.get(v)
            if u != v and b is not None and b is not a and b.pos(t) == u:
                bad += 1
        return bad

    def _check_consistent(self, t):
        for a in self.agents:
            rec = self.rt.record(a.id)
            park = rec[2] if rec else True
            k = first_invalid(self.grid, self.rt, a.id, a.path, t, park=park)
            assert k is None, f"agent {a.id} plan invalid at {k} after repair at t={t}"

    def _frame(self, t, events, changed, failed=False):
        if not self.record:
            return
        self.frames.append({
            "failed": failed,
            "t": t,
            "pos": {a.id: a.pos(t) for a in self.agents},
            "plan": {a.id: a.path[t:t + 12] for a in self.agents},
            "blocked": self.grid.blocked_cells(t),
            "broken": {a.id for a in self.agents if a.is_broken(t)},
            "lost": {a.id for a in self.agents if a.lost},
            "permanent": self.grid.permanent_cells(),
            "tasks": (self.result.tasks_done, self.result.tasks_total, self.result.tasks_lost),
            "carrying": {a.id for a in self.agents if a.carrying},
            "emergency": {a.id for a in self.agents if a.emergency},
            "events": events,
            "changed": set(changed),
        })
