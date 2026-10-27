"""Result records."""
from dataclasses import dataclass, field


@dataclass
class RepairRecord:
    t: int
    kind: str                     # blockage | breakdown | emergency
    disrupted: list               # agents whose plan was made invalid (or that must replan)
    changed: list                 # agents whose future plan differs after repair
    levels: dict = field(default_factory=dict)   # aid -> L0 wait | L1 detour | L2 negotiate | yield | replan
    messages: int = 0
    cpu_ms: float = 0.0
    fallback: bool = False
    failed: bool = False          # no conflict-free repair was found (the run halts)

    @property
    def n_changed(self):
        return len(self.changed)

    @property
    def n_collateral(self):
        return len(set(self.changed) - set(self.disrupted))


@dataclass
class RunResult:
    strategy: str
    n_agents: int
    density: float
    seed: int
    makespan: int = 0             # time of the last delivery over all agents
    sum_of_costs: int = 0         # sum over agents of their last-delivery time step
    sum_home: int = 0             # sum over agents of the time they parked at home
    tasks_done: int = 0
    tasks_total: int = 0
    collisions: int = 0
    timeout: bool = False
    initial_sum_home: int = 0     # sum_home of the initial plan, before any disruption
    halted: bool = False          # the run stopped because no conflict-free repair was found
    halt_t: int = None
    halt_reason: str = ""
    tasks_lost: int = 0           # tasks dropped as undeliverable (dead robot, unreachable cell)
    robots_lost: int = 0          # robots that left the fleet (dead or walled in)
    lost_by: dict = field(default_factory=dict)  # reason -> tasks lost
    records: list = field(default_factory=list)

    def summary(self):
        affecting = [r for r in self.records if r.disrupted]
        ch = [r.n_changed for r in affecting]
        return {
            "strategy": self.strategy,
            "n_agents": self.n_agents,
            "density": self.density,
            "seed": self.seed,
            "makespan": self.makespan,
            "sum_of_costs": self.sum_of_costs,
            "sum_home": self.sum_home,
            "initial_sum_home": self.initial_sum_home,
            "tasks_done": self.tasks_done,
            "tasks_total": self.tasks_total,
            "events": len(self.records),
            "disruptive_events": len(affecting),
            "mean_changed": sum(ch) / len(ch) if ch else 0.0,
            "max_changed": max(ch) if ch else 0,
            "mean_disrupted": (sum(len(r.disrupted) for r in affecting) / len(affecting)) if affecting else 0.0,
            "messages": sum(r.messages for r in self.records),
            "repair_cpu_ms": sum(r.cpu_ms for r in self.records),
            "fallbacks": sum(r.fallback for r in self.records),
            "collisions": self.collisions,
            "timeout": self.timeout,
            "halted": self.halted,
            "tasks_lost": self.tasks_lost,
            "robots_lost": self.robots_lost,
            "completion": self.tasks_done / self.tasks_total if self.tasks_total else 1.0,
        }
