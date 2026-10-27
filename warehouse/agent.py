"""Robot agent state."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Leg:
    cell: tuple
    kind: str          # 'pickup' | 'drop' | 'home'
    task: object = None


class Agent:
    def __init__(self, aid, home, tasks):
        self.id = aid
        self.home = home
        self.tasks = list(tasks)                  # [(task_id, pickup, drop)]
        self.legs = []
        for tid, pickup, drop in self.tasks:
            self.legs += [Leg(pickup, "pickup", tid), Leg(drop, "drop", tid)]
        self.legs.append(Leg(home, "home"))
        self.path = [home]                        # path[t] = cell at absolute time t
        self.original_path = None
        self.broken_until = -1                    # frozen in place for t <= broken_until
        self.emergency = False
        self.carrying = set()
        self.last_delivery = 0                    # time step of last completed delivery
        self.home_time = None                     # time step it parked for good
        self.times_changed = 0
        self.lost = None                          # why the robot left the fleet: 'dead' | 'trapped'

    @property
    def done(self):
        return not self.legs

    def pos(self, t):
        return self.path[t] if t < len(self.path) else self.path[-1]

    def prefix(self, t):
        """Path up to and including time t (padded if the agent is parked)."""
        p = self.path[: t + 1]
        return p + [p[-1]] * (t + 1 - len(p))

    def future(self, t):
        return tuple(self.path[t:]) if len(self.path) > t else (self.path[-1],)

    def planning_legs(self):
        """Legs to plan for. A parked agent asked to move must come back home."""
        return list(self.legs) if self.legs else [Leg(self.home, "home")]

    def is_broken(self, t):
        return t <= self.broken_until

    def __repr__(self):
        return f"Agent({self.id})"
