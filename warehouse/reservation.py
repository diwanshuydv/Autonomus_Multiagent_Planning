"""Shared space-time reservation table.

Every committed plan is stored here. It is multi-owner on purpose: after a
disruption an old (now invalid) plan may overlap with a new constraint, and
the overlap is exactly what identifies the agents that must repair.
"""


class ReservationTable:
    def __init__(self):
        self.v = {}     # cell -> {t: set(aid)}             vertex reservations
        self.e = {}     # (u, v, t) -> set(aid)             agent moves u->v between t and t+1
        self.park = {}  # cell -> {aid: t_from}             agent stays there forever from t_from
        self.rec = {}   # aid -> (path list, t_from, parked)
        self.max_t = 0  # upper bound on last reserved time step (never decreases)

    # ---- writes -----------------------------------------------------------
    def set_path(self, aid, path, t_from, park=True):
        """Replace aid's reservations with path[t_from:] (path is indexed by absolute time)."""
        self.clear(aid)
        path = list(path)
        for t in range(t_from, len(path)):
            self.v.setdefault(path[t], {}).setdefault(t, set()).add(aid)
            if t + 1 < len(path) and path[t] != path[t + 1]:
                self.e.setdefault((path[t], path[t + 1], t), set()).add(aid)
        if park:
            self.park.setdefault(path[-1], {})[aid] = len(path) - 1
        self.rec[aid] = (path, t_from, park)
        self.max_t = max(self.max_t, len(path))

    def clear(self, aid):
        rec = self.rec.pop(aid, None)
        if rec is None:
            return
        path, t_from, park = rec
        for t in range(t_from, len(path)):
            vt = self.v[path[t]]
            s = vt[t]
            s.discard(aid)
            if not s:
                del vt[t]
                if not vt:
                    del self.v[path[t]]
            if t + 1 < len(path) and path[t] != path[t + 1]:
                key = (path[t], path[t + 1], t)
                s = self.e[key]
                s.discard(aid)
                if not s:
                    del self.e[key]
        if park:
            pk = self.park[path[-1]]
            del pk[aid]
            if not pk:
                del self.park[path[-1]]

    def record(self, aid):
        return self.rec.get(aid)

    # ---- queries (return None when free, else the set of other owners) -----
    def vertex_owners(self, cell, t, aid):
        res = None
        vt = self.v.get(cell)
        if vt:
            s = vt.get(t)
            if s and (len(s) > 1 or aid not in s):
                res = s - {aid}
        pk = self.park.get(cell)
        if pk:
            for a, tf in pk.items():
                if a != aid and t >= tf:
                    if res is None:
                        res = set()
                    res.add(a)
        return res

    def swap_owners(self, u, v, t, aid):
        """Other agents moving v->u between t and t+1 (a swap with our u->v)."""
        s = self.e.get((v, u, t))
        if s and (len(s) > 1 or aid not in s):
            return s - {aid}
        return None

    def future_owners(self, cell, t, aid):
        """Others that occupy `cell` at any time >= t (blocks parking there from t)."""
        res = set()
        vt = self.v.get(cell)
        if vt:
            for t2, s in vt.items():
                if t2 >= t:
                    res |= s
        pk = self.park.get(cell)
        if pk:
            res |= set(pk)
        res.discard(aid)
        return res or None
