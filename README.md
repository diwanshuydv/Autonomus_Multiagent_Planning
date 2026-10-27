# Multi-agent warehouse pathfinding with local plan repair

Robots on a grid warehouse pick up objects and deliver them to stations. Their plans come from prioritized Space-Time A*. While the robots execute those plans, three kinds of disruption can happen: a cell gets blocked, a robot breaks down, or a robot receives an emergency task. When that happens, only the affected robots and the neighbours they negotiate with repair their plans. The multi-agent planner is never run again from scratch.

The design and the results are in [REPORT.pdf](REPORT.pdf) (source: [REPORT.md](REPORT.md)). Section 6 of the report lists the settings in which the robots fail to complete their tasks.

## Setup

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Run

```bash
.venv/bin/pytest -q                                      # tests (~1 min)
.venv/bin/python experiments/run_experiments.py --quick  # small version of every experiment (~5 min)
.venv/bin/python experiments/run_experiments.py          # all experiments A-I, writes results/
.venv/bin/python experiments/run_experiments.py --only E,F   # a subset
.venv/bin/python experiments/run_experiments.py --plots-only   # re-plot existing CSVs
.venv/bin/python visualize.py --preset all                # every GIF -> results/gifs/
.venv/bin/python visualize.py --preset compare            # repair vs full replan side by side
.venv/bin/python visualize.py --preset failure_deadlock   # a failure: deadlock at 20% obstacle density
.venv/bin/python visualize.py --preset failure_permanent  # a failure: permanent breakdowns + blocked stations
.venv/bin/python visualize.py --agents 30 --density 0.03 --breakdowns 4 --out results/gifs/custom.gif
.venv/bin/python build_pdf.py                             # REPORT.md -> REPORT.pdf (needs Chrome/Chromium)
```

## Layout

| Path | Contents |
|---|---|
| `warehouse/grid.py` | Grid map with shelves and time-bounded dynamic blockages; warehouse layout generator; BFS heuristic |
| `warehouse/reservation.py` | Shared space-time reservation table (vertex, edge and parking reservations; multi-owner) |
| `warehouse/planner.py` | Space-Time A* (with soft constraints for negotiation), multi-leg route planning, prioritized MAPF |
| `warehouse/repair.py` | Local repair: L0 wait insertion, L1 single-agent detour, L2 neighbour negotiation with rollback |
| `warehouse/baseline.py` | Baseline: full prioritized re-plan after every disruption |
| `warehouse/simulator.py` | Discrete-time execution, disruption injection (temporary and permanent), collision validator, failure detection, metrics |
| `warehouse/disruptions.py` | Event types, scenario and disruption generators |
| `experiments/run_experiments.py` | Experiments A (single disruption), B (number of agents), C (obstacle density), D (delay tolerance), E (communication radius), F (simultaneous breakdowns), G (bigger map), H (tasks per robot), I (failure settings) |
| `visualize.py` | GIF animations (presets: demo, blockages, breakdowns, emergencies, dense, bigmap, compare, failure_deadlock, failure_permanent) and snapshots |
| `build_pdf.py` | Builds `REPORT.pdf` from `REPORT.md` |
| `tests/` | Unit and end-to-end tests |
| `results/` | CSVs, `summary.md`, `figures/`, `gifs/` |
