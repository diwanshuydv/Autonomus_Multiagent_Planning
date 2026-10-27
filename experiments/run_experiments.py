"""Run the experiments, write CSVs to results/ and plots to results/figures/.

  A  single disruption: agents whose plans change for ONE disruption
     (blockage / breakdown / emergency), local repair vs full replan
  B  number of agents (fixed disruption mix)
  C  dynamic-obstacle density (fixed number of agents)
  D  delay tolerance tau of the repair (single disruption)
  E  communication radius of the negotiation (single disruption)
  F  k robots breaking down at the same moment
  G  bigger warehouse, up to 100 robots
  H  number of tasks per robot
  I  failure settings: where the robots do NOT complete their tasks
     (extreme density / fleet size with purely local repair, long blockages,
     permanent breakdowns, permanent blockages, blocked delivery stations)

Usage:  python experiments/run_experiments.py [--only A,B,...] [--quick] [--workers 8]
        python experiments/run_experiments.py --plots-only
"""
import argparse
import csv
import math
import os
import random
import sys
import time
from collections import Counter, defaultdict
from multiprocessing import Pool

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from warehouse.disruptions import Event, make_agents, make_events, single_event  # noqa: E402
from warehouse.grid import make_warehouse  # noqa: E402
from warehouse.simulator import Simulator  # noqa: E402

RESULTS = os.path.join(ROOT, "results")
FIGS = os.path.join(RESULTS, "figures")
STRATEGIES = ("repair", "replan")
KINDS = ("blockage", "breakdown", "emergency")
BIG_MAP = dict(n_block_rows=6, n_block_cols=10)   # 26 x 55, 110 homes
CSV = {
    "A": "single_disruption.csv", "B": "scaling_agents.csv", "C": "scaling_density.csv",
    "D": "tau_ablation.csv", "E": "radius_ablation.csv", "F": "simultaneous_breakdowns.csv",
    "G": "big_map.csv", "H": "tasks_per_agent.csv", "I": "failures.csv",
}


def build(n_agents, seed, tasks=3, big=False):
    rng = random.Random(seed * 1009 + n_agents)
    grid = make_warehouse(**BIG_MAP) if big else make_warehouse()
    agents = make_agents(grid, n_agents, tasks, rng)
    return grid, agents, rng


def undisturbed(n_agents, seed, tasks=3, big=False):
    grid, agents, _ = build(n_agents, seed, tasks, big)
    return Simulator(grid, agents, [], seed=seed).run()


# ---------------------------------------------------------------- jobs
def job_single(spec):
    """A: one disruption, both strategies."""
    n, kind, seed = spec
    base = undisturbed(n, seed)
    rows = []
    for strat in STRATEGIES:
        grid, agents, _ = build(n, seed)
        ev = single_event(kind, random.Random(seed * 31 + len(kind)))
        r = Simulator(grid, agents, [ev], strategy=strat, seed=seed).run()
        rec = r.records[0] if r.records else None
        rows.append({
            "n_agents": n, "kind": kind, "seed": seed, "strategy": strat,
            "fired": rec is not None,
            "t": rec.t if rec else "",
            "n_disrupted": len(rec.disrupted) if rec else 0,
            "n_changed": rec.n_changed if rec else 0,
            "n_collateral": rec.n_collateral if rec else 0,
            "levels": ";".join(sorted(set(rec.levels.values()))) if rec else "",
            "messages": rec.messages if rec else 0,
            "cpu_ms": round(rec.cpu_ms, 3) if rec else 0,
            "fallback": rec.fallback if rec else False,
            "sum_of_costs": r.sum_of_costs, "makespan": r.makespan,
            "base_sum_of_costs": base.sum_of_costs, "base_makespan": base.makespan,
            "extra_steps": r.sum_of_costs - base.sum_of_costs,
            "collisions": r.collisions, "timeout": r.timeout,
        })
    return rows


def job_param(spec):
    """D / E: one disruption, local repair only, one engine parameter varied."""
    n, kind, seed, param, value = spec
    base = undisturbed(n, seed)
    grid, agents, _ = build(n, seed)
    ev = single_event(kind, random.Random(seed * 31 + len(kind)))
    r = Simulator(grid, agents, [ev], strategy="repair", seed=seed, **{param: value}).run()
    rec = r.records[0]
    return [{"n_agents": n, "kind": kind, "seed": seed, param: value,
             "n_changed": rec.n_changed, "n_collateral": rec.n_collateral,
             "messages": rec.messages, "cpu_ms": round(rec.cpu_ms, 3), "fallback": rec.fallback,
             "extra_steps": r.sum_of_costs - base.sum_of_costs, "collisions": r.collisions}]


def job_simultaneous(spec):
    """F: k distinct robots break down at the same time step."""
    n, k, seed = spec
    base = undisturbed(n, seed)
    rng = random.Random(seed * 13 + k)
    t = rng.randint(15, 30)
    victims = rng.sample(range(n), k)
    durs = [rng.randint(10, 20) for _ in range(k)]
    rows = []
    for strat in STRATEGIES:
        grid, agents, _ = build(n, seed)
        events = [Event(t, "breakdown", d, agent=a) for a, d in zip(victims, durs)]
        r = Simulator(grid, agents, events, strategy=strat, seed=seed).run()
        changed = set().union(*(rec.changed for rec in r.records)) if r.records else set()
        rows.append({
            "n_agents": n, "k": k, "seed": seed, "strategy": strat,
            "breakdowns_fired": len(r.records),
            "n_changed": len(changed),
            "messages": sum(rec.messages for rec in r.records),
            "cpu_ms": round(sum(rec.cpu_ms for rec in r.records), 3),
            "fallbacks": sum(rec.fallback for rec in r.records),
            "extra_steps": r.sum_of_costs - base.sum_of_costs,
            "collisions": r.collisions, "timeout": r.timeout,
        })
    return rows


def job_sweep(spec):
    """B / C / G / H: full runs with random disruptions, both strategies."""
    exp, n, density, seed, tasks, big = spec
    base = undisturbed(n, seed, tasks, big)
    rows = []
    for strat in STRATEGIES:
        grid, agents, _ = build(n, seed, tasks, big)
        events = make_events(grid, n, random.Random(seed * 7 + 3), density=density)
        r = Simulator(grid, agents, events, strategy=strat, seed=seed, density=density).run()
        row = {"exp": exp, "tasks_per_agent": tasks, "map": "big" if big else "default", **r.summary()}
        row["base_sum_of_costs"] = base.sum_of_costs
        row["base_makespan"] = base.makespan
        row["extra_steps"] = r.sum_of_costs - base.sum_of_costs
        row["cpu_ms_per_event"] = row["repair_cpu_ms"] / (row["disruptive_events"] or 1)
        rows.append(row)
    return rows


def job_failure(spec):
    """I: stress settings, local repair only; did the robots finish every task?"""
    setting, n, x, seed, fallback = spec
    grid, agents, _ = build(n, seed)
    rng = random.Random(seed * 7 + 3)
    free = [c for c in grid.cells if c not in set(grid.homes)]
    if setting == "density":            # x = density, default blockage durations
        events = make_events(grid, n, rng, density=x)
    elif setting == "duration":         # x = mean blockage duration, density 10%
        events = make_events(grid, n, rng, density=0.10, block_dur=(x // 2, 3 * x // 2))
    elif setting == "perm_breakdown":   # x robots break down for good between t = 10 and 80
        events = [Event(rng.randint(10, 80), "breakdown", permanent=True, seed=rng.randrange(1 << 30))
                  for _ in range(x)]
    elif setting == "perm_blockage":    # x random free cells blocked for good
        events = [Event(rng.randint(10, 80), "blockage", permanent=True, cell=c)
                  for c in rng.sample(free, x)]
    else:                               # "station": x delivery stations blocked for good at t = 10
        events = [Event(10, "blockage", permanent=True, cell=c) for c in rng.sample(grid.stations, x)]
    events.sort(key=lambda e: e.t)
    sim = Simulator(grid, agents, events, strategy="repair", seed=seed, fallback=fallback,
                    density=x if setting == "density" else 0.0)
    r = sim.run()
    return [{
        "setting": setting, "n_agents": n, "x": x, "seed": seed, "fallback": fallback,
        "success": int(r.tasks_done == r.tasks_total and not r.halted and not r.timeout),
        "halted": int(r.halted), "halt_t": r.halt_t if r.halted else "",
        "timeout": int(r.timeout), "tasks_done": r.tasks_done, "tasks_total": r.tasks_total,
        "completion": r.tasks_done / r.tasks_total, "tasks_lost": r.tasks_lost,
        "robots_lost": r.robots_lost,
        **{f"lost_{k.replace(' ', '_')}": r.lost_by.get(k, 0)
           for k in ("dead", "trapped", "cell blocked", "walled off")},
        "fallbacks": sum(rec.fallback for rec in r.records),
        "disruptive_events": sum(1 for rec in r.records if rec.disrupted),
        "collisions": r.collisions,
    }]


def run_pool(fn, specs, workers, label):
    out = []
    t0 = time.time()
    # longest jobs first keeps all workers busy until the end
    specs = sorted(specs, key=lambda s: -s[1] if isinstance(s[1], int) else 0)
    if fn is job_failure:
        specs = sorted(specs, key=lambda s: -s[1])
    with Pool(workers) as pool:
        for i, rows in enumerate(pool.imap_unordered(fn, specs), 1):
            out.extend(rows)
            if i % max(1, len(specs) // 10) == 0 or i == len(specs):
                print(f"  {label}: {i}/{len(specs)}  ({time.time() - t0:.0f}s)", flush=True)
    return out


def write_csv(name, rows, key):
    rows = sorted(rows, key=key)
    with open(os.path.join(RESULTS, name), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def plan(quick):
    """Experiment specs. Full sizes are chosen for tight 95% intervals."""
    s = (lambda full, q: range(q) if quick else range(full))
    return {
        "A": (job_single, [(n, k, sd) for n in ([10, 30] if quick else [10, 20, 30, 40, 50])
                           for k in KINDS for sd in s(30, 3)],
              lambda r: (r["kind"], r["n_agents"], r["seed"], r["strategy"])),
        "B": (job_sweep, [("agents", n, 0.01, sd, 3, False)
                          for n in ([10, 30] if quick else [5, 10, 20, 30, 40, 50]) for sd in s(30, 2)],
              lambda r: (r["n_agents"], r["seed"], r["strategy"])),
        "C": (job_sweep, [("density", 20, d, sd, 3, False)
                          for d in ([0.0, 0.02] if quick else [0.0, 0.005, 0.01, 0.02, 0.03, 0.05, 0.075, 0.10])
                          for sd in s(30, 2)],
              lambda r: (r["density"], r["seed"], r["strategy"])),
        "D": (job_param, [(40, k, sd, "tau", tau) for k in ("blockage", "breakdown") for sd in s(30, 3)
                          for tau in (0, 2, 4, 8, 16, 32, 1000)],
              lambda r: (r["kind"], r["tau"], r["seed"])),
        "E": (job_param, [(40, k, sd, "radius", rad) for k in KINDS for sd in s(30, 3)
                          for rad in (1, 2, 4, 8, 16, 1000)],
              lambda r: (r["kind"], r["radius"], r["seed"])),
        "F": (job_simultaneous, [(30, k, sd) for k in (1, 2, 4, 8, 12) for sd in s(30, 3)],
              lambda r: (r["k"], r["seed"], r["strategy"])),
        "G": (job_sweep, [("bigmap", n, 0.01, sd, 3, True)
                          for n in ([25, 50] if quick else [25, 50, 75, 100]) for sd in s(10, 1)],
              lambda r: (r["n_agents"], r["seed"], r["strategy"])),
        "H": (job_sweep, [("tasks", 20, 0.01, sd, tk, False)
                          for tk in ([1, 3] if quick else [1, 3, 5, 8]) for sd in s(30, 2)],
              lambda r: (r["tasks_per_agent"], r["seed"], r["strategy"])),
        "I": (job_failure, failure_specs(quick),
              lambda r: (r["setting"], r["n_agents"], r["x"], r["fallback"], r["seed"])),
    }


def failure_specs(quick):
    seeds = range(3) if quick else range(20)
    densities = [0.0, 0.05, 0.15, 0.30] if quick else [0.0, 0.02, 0.05, 0.10, 0.15, 0.20, 0.30]
    fleets = [20, 50] if quick else [10, 20, 30, 40, 50, 60, 70]
    specs = [("density", n, d, sd, fb) for n in fleets for d in densities for sd in seeds
             for fb in (False, True)]
    specs += [("duration", 30, dur, sd, fb) for dur in ([15, 90] if quick else [15, 45, 90, 180])
              for sd in seeds for fb in (False, True)]
    seeds = range(3) if quick else range(30)
    specs += [("perm_breakdown", 30, k, sd, True) for k in ([0, 4] if quick else [0, 1, 2, 4, 8, 12])
              for sd in seeds]
    specs += [("perm_blockage", 30, m, sd, True) for m in ([0, 20] if quick else [0, 5, 10, 20, 40, 80])
              for sd in seeds]
    specs += [("station", 30, k, sd, True) for k in ([1, 14] if quick else [1, 2, 4, 7, 14])
              for sd in seeds]
    return specs


# ---------------------------------------------------------------- stats
def mean_ci(xs):
    xs = [float(x) for x in xs]
    m = sum(xs) / len(xs)
    if len(xs) < 2:
        return m, 0.0
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))
    return m, 1.96 * sd / math.sqrt(len(xs))


def group(rows, keys, value):
    g = defaultdict(list)
    for r in rows:
        g[tuple(r[k] for k in keys)].append(r[value])
    return {k: mean_ci(v) for k, v in g.items()}


def load_csv(name):
    path = os.path.join(RESULTS, name)
    if not os.path.exists(path):
        return []
    num = lambda v: (float(v) if "." in v or "e" in v else int(v))  # noqa: E731
    rows = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            row = {}
            for k, v in r.items():
                if v in ("True", "False"):
                    row[k] = v == "True"
                else:
                    try:
                        row[k] = num(v)
                    except ValueError:
                        row[k] = v
            rows.append(row)
    return rows


# ---------------------------------------------------------------- plots
# validated reference palette (first three slots are safe all-pairs)
COLORS = {"repair": "#2a78d6", "replan": "#eb6834", "undisturbed": "#1baf7a",
          "blockage": "#2a78d6", "breakdown": "#eb6834", "emergency": "#1baf7a"}
LABELS = {"repair": "Local repair (negotiation)", "replan": "Full replan (baseline)",
          "undisturbed": "No disruptions", "blockage": "Blockage", "breakdown": "Breakdown",
          "emergency": "Emergency"}
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(FIGS, exist_ok=True)
    plt.rcParams.update({"font.size": 10, "figure.facecolor": "#fcfcfb",
                         "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
    return plt


def _style(ax, title, xlabel, ylabel):
    ax.set_title(title, loc="left", fontsize=11, color=INK)
    ax.set_xlabel(xlabel, color=MUTED)
    ax.set_ylabel(ylabel, color=MUTED)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED)


def _lines(ax, series, xs, err=True, ticklabels=None):
    """series: {name: {x: (mean, ci)}}"""
    for name, pts in series.items():
        ys = [pts[x][0] for x in xs]
        es = [pts[x][1] for x in xs] if err else None
        ax.errorbar(xs, ys, yerr=es, color=COLORS[name], label=LABELS[name], linewidth=2,
                    marker="o", markersize=6, capsize=3, markeredgecolor="white",
                    markeredgewidth=1.5)
    ax.set_xticks(xs)
    if ticklabels:
        ax.set_xticklabels(ticklabels)


def _save(plt, fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, name), dpi=150)
    plt.close(fig)


def plot_single(rows):
    plt = _plt()
    fired = [r for r in rows if r["fired"]]
    ns = sorted({r["n_agents"] for r in fired})
    for metric, ylabel, fname, title in (
        ("n_changed", "agents whose plan changed", "single_changed.png",
         "Plans altered by one disruption"),
        ("extra_steps", "extra total time steps vs. no disruption", "single_extra_steps.png",
         "Time cost of one disruption"),
    ):
        fig, axes = plt.subplots(1, 3, figsize=(13, 3.8), sharey=True)
        for ax, kind in zip(axes, KINDS):
            g = group([r for r in fired if r["kind"] == kind], ["strategy", "n_agents"], metric)
            _lines(ax, {s: {n: g[(s, n)] for n in ns} for s in STRATEGIES}, ns)
            _style(ax, kind.capitalize(), "number of agents", ylabel if ax is axes[0] else "")
        axes[0].legend(frameon=False, loc="upper left")
        fig.suptitle(title, x=0.01, ha="left", fontsize=12, color=INK)
        _save(plt, fig, fname)


def plot_sweep(rows, xkey, xlabel, prefix, xfmt=lambda x: x):
    plt = _plt()
    xs = sorted({r[xkey] for r in rows})
    disp = [xfmt(x) for x in xs]
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    g = group(rows, ["strategy", xkey], "sum_of_costs")
    gb = group([r for r in rows if r["strategy"] == "repair"], [xkey], "base_sum_of_costs")
    series = {"undisturbed": {xfmt(x): gb[(x,)] for x in xs}}
    series.update({s: {xfmt(x): g[(s, x)] for x in xs} for s in STRATEGIES})
    _lines(axes[0], series, disp)
    _style(axes[0], "Total time (sum of costs)", xlabel, "sum of task-completion times (steps)")
    axes[0].legend(frameon=False, loc="upper left")
    g = group(rows, ["strategy", xkey], "extra_steps")
    _lines(axes[1], {s: {xfmt(x): g[(s, x)] for x in xs} for s in STRATEGIES}, disp)
    _style(axes[1], "Extra time caused by disruptions", xlabel, "sum of costs minus undisturbed (steps)")
    _save(plt, fig, f"{prefix}_time.png")

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    for ax, (metric, title, ylabel, log) in zip(axes, (
        ("mean_changed", "Plans altered per disruption", "agents changed per disruptive event", False),
        ("cpu_ms_per_event", "Repair computation time", "CPU ms per disruptive event (log scale)", True),
    )):
        g = group(rows, ["strategy", xkey], metric)
        _lines(ax, {s: {xfmt(x): g[(s, x)] for x in xs} for s in STRATEGIES}, disp, err=not log)
        if log:
            ax.set_yscale("log")
        _style(ax, title, xlabel, ylabel)
    axes[0].legend(frameon=False, loc="upper left")
    _save(plt, fig, f"{prefix}_changes.png")


def plot_radius(rows):
    plt = _plt()
    radii = sorted({r["radius"] for r in rows})
    ticks = [("inf" if r >= 1000 else str(r)) for r in radii]
    pos = list(range(len(radii)))
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.8))
    for ax, (metric, title, ylabel) in zip(axes, (
        ("n_changed", "Plans altered", "agents whose plan changed"),
        ("extra_steps", "Time cost", "extra total time steps"),
        ("messages", "Communication", "messages exchanged"),
    )):
        series = {}
        for kind in KINDS:
            g = group([r for r in rows if r["kind"] == kind], ["radius"], metric)
            series[kind] = {p: g[(rad,)] for p, rad in zip(pos, radii)}
        _lines(ax, series, pos, ticklabels=ticks)
        _style(ax, title, "communication radius R (cells)", ylabel)
    axes[0].legend(frameon=False, loc="upper left")
    fig.suptitle("Negotiation radius (40 agents, one disruption, local repair)", x=0.01, ha="left",
                 fontsize=12, color=INK)
    _save(plt, fig, "radius_ablation.png")


def plot_simultaneous(rows):
    plt = _plt()
    ks = sorted({r["k"] for r in rows})
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    for ax, (metric, title, ylabel) in zip(axes, (
        ("n_changed", "Plans altered", "agents whose plan changed (all k events)"),
        ("extra_steps", "Time cost", "extra total time steps"),
    )):
        g = group(rows, ["strategy", "k"], metric)
        _lines(ax, {s: {k: g[(s, k)] for k in ks} for s in STRATEGIES}, ks)
        _style(ax, title, "robots breaking down at the same moment (30 agents)", ylabel)
    axes[0].legend(frameon=False, loc="center right")
    _save(plt, fig, "simultaneous_breakdowns.png")


def plot_failures(rows):
    plt = _plt()
    dens = [r for r in rows if r["setting"] == "density"]
    if dens:
        ns = sorted({r["n_agents"] for r in dens})
        ds = sorted({r["x"] for r in dens})
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))
        for ax, fb, title in zip(axes, (False, True), ("Purely local repair (no global fallback)",
                                                        "Local repair + global fallback")):
            g = group([r for r in dens if r["fallback"] == fb], ["n_agents", "x"], "success")
            m = [[g[(n, d)][0] * 100 for d in ds] for n in ns]
            im = ax.imshow(m, cmap="RdYlGn", vmin=0, vmax=100, aspect="auto", origin="lower")
            for i in range(len(ns)):
                for j in range(len(ds)):
                    ax.text(j, i, f"{m[i][j]:.0f}", ha="center", va="center", fontsize=8,
                            color=INK if 25 < m[i][j] < 85 else "white")
            ax.set_xticks(range(len(ds)))
            ax.set_xticklabels([f"{d * 100:g}%" for d in ds])
            ax.set_yticks(range(len(ns)))
            ax.set_yticklabels(ns)
            ax.set_title(title, loc="left", fontsize=11, color=INK)
            ax.set_xlabel("dynamic-obstacle density", color=MUTED)
            ax.set_ylabel("number of agents", color=MUTED)
        fig.colorbar(im, ax=axes, label="% of runs where every task was delivered", shrink=0.9)
        fig.suptitle("Where the system fails: success rate over fleet size and obstacle density",
                     x=0.01, ha="left", fontsize=12, color=INK)
        fig.savefig(os.path.join(FIGS, "failure_heatmap.png"), dpi=150, bbox_inches="tight")
        plt.close(fig)
    fig, axes = plt.subplots(1, 3, figsize=(14, 3.8), sharey=True)
    for ax, (setting, xlabel, title) in zip(axes, (
        ("perm_breakdown", "robots that break down for good (30 agents)", "Permanent breakdowns"),
        ("perm_blockage", "cells blocked for good (30 agents)", "Permanent blockages"),
        ("station", "delivery stations blocked for good (of 14)", "Blocked delivery stations"),
    )):
        sub = [r for r in rows if r["setting"] == setting]
        if not sub:
            continue
        xs = sorted({r["x"] for r in sub})
        g = group(sub, ["x"], "completion")
        ax.errorbar(xs, [g[(x,)][0] * 100 for x in xs], yerr=[g[(x,)][1] * 100 for x in xs],
                    color=COLORS["breakdown"], lw=2, marker="o", markersize=6, capsize=3,
                    markeredgecolor="white", markeredgewidth=1.5)
        ax.set_xticks(xs)
        ax.set_ylim(0, 105)
        _style(ax, title, xlabel, "% of tasks delivered" if ax is axes[0] else "")
    fig.suptitle("Permanent disruptions: tasks that can never be completed", x=0.01, ha="left",
                 fontsize=12, color=INK)
    _save(plt, fig, "failure_permanent.png")


# ---------------------------------------------------------------- summary tables
def fmt(v, d=2):
    return f"{v[0]:.{d}f} +- {v[1]:.{d}f}"


def md_single(rows):
    fired = [r for r in rows if r["fired"]]
    out = ["## Exp A - one disruption: agents whose plans change (mean +- 95% CI)", "",
           "| kind | agents | repair: changed | repair: collateral | replan: changed | "
           "repair extra steps | replan extra steps | repair msgs |",
           "|---|---|---|---|---|---|---|---|"]
    keys = ["kind", "n_agents", "strategy"]
    ch, co, ex, ms = (group(fired, keys, m) for m in ("n_changed", "n_collateral", "extra_steps", "messages"))
    for kind in KINDS:
        for n in sorted({r["n_agents"] for r in fired}):
            k = (kind, n)
            out.append(f"| {kind} | {n} | {fmt(ch[k + ('repair',)])} | {fmt(co[k + ('repair',)])} | "
                       f"{fmt(ch[k + ('replan',)])} | {fmt(ex[k + ('repair',)], 1)} | "
                       f"{fmt(ex[k + ('replan',)], 1)} | {ms[k + ('repair',)][0]:.1f} |")
    lv = Counter(level for r in fired if r["strategy"] == "repair"
                 for level in r["levels"].split(";") if level)
    n_runs = sum(1 for r in fired if r["strategy"] == "repair")
    out += ["", f"Repair levels used (number of the {n_runs} single-disruption repairs using each level): "
            + ", ".join(f"{k}: {v}" for k, v in sorted(lv.items())), ""]
    return out


def md_sweep(rows, xkey, title, xfmt=lambda x: x):
    out = [f"## {title}", "",
           f"| {xkey} | strategy | sum of costs | extra time | makespan | undisturbed SoC | "
           "disruptive events | agents changed / event | CPU ms / event | fallbacks | collisions |",
           "|---|---|---|---|---|---|---|---|---|---|---|"]
    keys = ["sum_of_costs", "extra_steps", "makespan", "base_sum_of_costs", "disruptive_events",
            "mean_changed", "cpu_ms_per_event", "fallbacks", "collisions"]
    gs = {k: group(rows, [xkey, "strategy"], k) for k in keys}
    for x in sorted({r[xkey] for r in rows}):
        for s in STRATEGIES:
            v = [gs[k][(x, s)] for k in keys]
            out.append(f"| {xfmt(x)} | {s} | {v[0][0]:.0f} | {fmt(v[1], 0)} | {v[2][0]:.1f} | {v[3][0]:.0f} | "
                       f"{v[4][0]:.1f} | {fmt(v[5])} | {v[6][0]:.1f} | {v[7][0]:.2f} | {v[8][0]:.0f} |")
    return out + [""]


def md_param(rows, param, title, kinds, show=lambda v: v):
    out = [f"## {title}", "",
           f"| kind | {param} | agents changed | collateral | extra steps | messages | CPU ms | fallbacks |",
           "|---|---|---|---|---|---|---|---|"]
    for kind in kinds:
        sub = [r for r in rows if r["kind"] == kind]
        g = {m: group(sub, [param], m) for m in
             ("n_changed", "n_collateral", "extra_steps", "messages", "cpu_ms")}
        for (v,) in sorted(g["n_changed"]):
            out.append(f"| {kind} | {show(v)} | {fmt(g['n_changed'][(v,)])} | {g['n_collateral'][(v,)][0]:.2f} | "
                       f"{fmt(g['extra_steps'][(v,)], 1)} | {g['messages'][(v,)][0]:.1f} | "
                       f"{g['cpu_ms'][(v,)][0]:.1f} | {sum(r['fallback'] for r in sub if r[param] == v)} |")
    return out + [""]


def md_simultaneous(rows):
    out = ["## Exp F - k robots break down at the same moment (30 agents)", "",
           "| k | strategy | agents changed (union) | extra steps | messages | CPU ms (all k) | fallbacks |",
           "|---|---|---|---|---|---|---|"]
    g = {m: group(rows, ["k", "strategy"], m) for m in ("n_changed", "extra_steps", "messages", "cpu_ms", "fallbacks")}
    for k in sorted({r["k"] for r in rows}):
        for s in STRATEGIES:
            key = (k, s)
            out.append(f"| {k} | {s} | {fmt(g['n_changed'][key])} | {fmt(g['extra_steps'][key], 1)} | "
                       f"{g['messages'][key][0]:.1f} | {g['cpu_ms'][key][0]:.0f} | {g['fallbacks'][key][0]:.2f} |")
    return out + [""]


def md_failures(rows):
    out = ["## Exp I - failure settings (local repair)", "",
           "### I.1 Success rate (% of runs in which every task was delivered) over fleet size and density", ""]
    dens = [r for r in rows if r["setting"] == "density"]
    ds = sorted({r["x"] for r in dens})
    for fb in (False, True):
        sub = [r for r in dens if r["fallback"] == fb]
        if not sub:
            continue
        g = group(sub, ["n_agents", "x"], "success")
        c = group(sub, ["n_agents", "x"], "completion")
        out += [f"**{'with' if fb else 'without'} global fallback** (cell = success % / tasks delivered %)", "",
                "| agents | " + " | ".join(f"rho={d * 100:g}%" for d in ds) + " |",
                "|---|" + "---|" * len(ds)]
        for n in sorted({r["n_agents"] for r in sub}):
            out.append(f"| {n} | " + " | ".join(f"{g[(n, d)][0] * 100:.0f} / {c[(n, d)][0] * 100:.0f}"
                                                 for d in ds) + " |")
        out.append("")
    for setting, title in (("duration", "I.2 Long blockages (30 agents, density 10%; x = mean duration)"),
                           ("perm_breakdown", "I.3 Permanent breakdowns (30 agents; x = robots lost)"),
                           ("perm_blockage", "I.4 Permanent blockages (30 agents; x = cells blocked)"),
                           ("station", "I.5 Blocked delivery stations (30 agents; x = stations blocked of 14)")):
        sub = [r for r in rows if r["setting"] == setting]
        if not sub:
            continue
        out += [f"### {title}", "",
                "| x | fallback | success % | tasks delivered % | halted % | tasks lost | robots lost | "
                "lost: dead robot | lost: trapped | lost: cell blocked | lost: walled off | collisions |",
                "|---|---|---|---|---|---|---|---|---|---|---|---|"]
        keys = ["success", "completion", "halted", "tasks_lost", "robots_lost", "lost_dead",
                "lost_trapped", "lost_cell_blocked", "lost_walled_off", "collisions"]
        gs = {k: group(sub, ["x", "fallback"], k) for k in keys}
        for (x, fb) in sorted(gs["success"]):
            v = [gs[k][(x, fb)][0] for k in keys]
            out.append(f"| {x} | {fb} | {v[0] * 100:.0f} | {v[1] * 100:.1f} | {v[2] * 100:.0f} | {v[3]:.2f} | "
                       f"{v[4]:.2f} | {v[5]:.2f} | {v[6]:.2f} | {v[7]:.2f} | {v[8]:.2f} | {v[9]:.0f} |")
        out.append("")
    return out


def report():
    data = {k: load_csv(v) for k, v in CSV.items()}
    lines = ["# Results summary", "", "All values are means over seeds, with 95% confidence intervals (+-).", ""]
    if data["A"]:
        plot_single(data["A"])
        lines += md_single(data["A"])
    if data["B"]:
        plot_sweep(data["B"], "n_agents", "number of agents", "agents")
        lines += md_sweep(data["B"], "n_agents", "Exp B - number of agents (density 1%)")
    if data["C"]:
        pct = lambda d: round(d * 100, 2)  # noqa: E731
        plot_sweep(data["C"], "density", "dynamic-obstacle density (% of free cells blocked)", "density", pct)
        lines += md_sweep(data["C"], "density", "Exp C - dynamic-obstacle density (20 agents)",
                          lambda d: f"{d * 100:g}%")
    if data["D"]:
        lines += md_param(data["D"], "tau", "Exp D - delay tolerance tau (40 agents, one disruption, local repair)",
                          ("blockage", "breakdown"), lambda v: "inf" if v >= 1000 else v)
    if data["E"]:
        plot_radius(data["E"])
        lines += md_param(data["E"], "radius", "Exp E - communication radius R (40 agents, one disruption, local repair)",
                          KINDS, lambda v: "inf" if v >= 1000 else v)
    if data["F"]:
        plot_simultaneous(data["F"])
        lines += md_simultaneous(data["F"])
    if data["G"]:
        plot_sweep(data["G"], "n_agents", "number of agents (26 x 55 map)", "bigmap")
        lines += md_sweep(data["G"], "n_agents", "Exp G - bigger warehouse (26 x 55, density 1%)")
    if data["H"]:
        plot_sweep(data["H"], "tasks_per_agent", "tasks per robot (20 agents)", "tasks")
        lines += md_sweep(data["H"], "tasks_per_agent", "Exp H - tasks per robot (20 agents, density 1%)")
    if data["I"]:
        plot_failures(data["I"])
        lines += md_failures(data["I"])
    with open(os.path.join(RESULTS, "summary.md"), "w") as fh:
        fh.write("\n".join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="ABCDEFGHI", help="experiments to run, e.g. A,E,F")
    ap.add_argument("--quick", action="store_true", help="small sizes for a smoke test")
    ap.add_argument("--workers", type=int, default=os.cpu_count())
    ap.add_argument("--plots-only", action="store_true", help="re-plot from existing CSVs")
    args = ap.parse_args()
    os.makedirs(RESULTS, exist_ok=True)
    if not args.plots_only:
        specs = plan(args.quick)
        for exp in [e for e in args.only.upper() if e in specs]:
            fn, sp, key = specs[exp]
            rows = run_pool(fn, sp, args.workers, f"Exp {exp}")
            write_csv(CSV[exp], rows, key)
    report()
    print(f"wrote {RESULTS}/*.csv, summary.md and figures/")


if __name__ == "__main__":
    main()
