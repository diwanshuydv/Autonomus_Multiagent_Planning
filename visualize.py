"""Animate warehouse scenarios as GIFs (and save snapshots of repair moments).

Usage:
  python visualize.py --preset all              # every preset -> results/gifs/
  python visualize.py --preset compare          # repair vs full replan side by side
  python visualize.py --agents 30 --density 0.03 --breakdowns 4 --out results/gifs/custom.gif
"""
import argparse
import os
import random

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch, Rectangle  # noqa: E402

from warehouse.disruptions import Event, make_agents, make_events  # noqa: E402
from warehouse.grid import make_warehouse  # noqa: E402
from warehouse.simulator import Simulator  # noqa: E402

SURFACE, INK, MUTED = "#fcfcfb", "#0b0b0b", "#52514e"
SHELF, STATION, HOME = "#b9b8b2", "#1baf7a", "#ecebe7"
ROBOT, CHANGED, BLOCKED, BROKEN, EMERG = "#2a78d6", "#eb6834", "#e34948", "#0b0b0b", "#4a3aa7"
PERM_BLOCK, LOST = "#7a1f1e", "#8d8c87"
HIGHLIGHT_FRAMES = 10
BIG_MAP = dict(n_block_rows=6, n_block_cols=10)
GIF_DIR = "results/gifs"

PRESETS = {
    "demo": dict(agents=15, density=0.02, breakdowns=3, emergencies=2, snapshots=True),
    "blockages": dict(agents=20, density=0.05, breakdowns=0, emergencies=0),
    "breakdowns": dict(agents=25, density=0.0, breakdowns=6, emergencies=0),
    "emergencies": dict(agents=20, density=0.0, breakdowns=0, emergencies=4),
    "dense": dict(agents=50, density=0.02, breakdowns=3, emergencies=2),
    "bigmap": dict(agents=60, density=0.01, breakdowns=4, emergencies=2, big=True),
    "compare": dict(agents=25, density=0.02, breakdowns=3, emergencies=2, compare=True),
    # failure settings (report section 6): the robots do not finish every task
    "failure_deadlock": dict(agents=40, density=0.20, breakdowns=0, emergencies=0, fallback=False,
                             seed=1),
    "failure_permanent": dict(agents=25, density=0.01, breakdowns=0, emergencies=0,
                              perm_breakdowns=3, blocked_stations=3),
}


def simulate(cfg, strategy):
    rng = random.Random(cfg["seed"])
    grid = make_warehouse(**BIG_MAP) if cfg.get("big") else make_warehouse()
    agents = make_agents(grid, cfg["agents"], 3, rng)
    ev_rng = random.Random(cfg["seed"] + 1)
    events = make_events(grid, cfg["agents"], ev_rng, density=cfg["density"],
                         n_breakdowns=cfg["breakdowns"], n_emergencies=cfg["emergencies"])
    events += [Event(ev_rng.randint(15, 60), "breakdown", permanent=True, seed=ev_rng.randrange(1 << 30))
               for _ in range(cfg.get("perm_breakdowns", 0))]
    events += [Event(10, "blockage", permanent=True, cell=c)
               for c in ev_rng.sample(grid.stations, cfg.get("blocked_stations", 0))]
    events.sort(key=lambda e: e.t)
    sim = Simulator(grid, agents, events, strategy=strategy, seed=cfg["seed"], record=True,
                    fallback=cfg.get("fallback", True))
    res = sim.run()
    print(f"  {strategy}: {len(sim.frames)} steps, sum of costs {res.sum_of_costs}, "
          f"{len(res.records)} events, collisions {res.collisions}, tasks {res.tasks_done}/{res.tasks_total}"
          + (f", HALTED: {res.halt_reason}" if res.halted else ""))
    return grid, sim.frames, res


def draw_static(ax, grid):
    ax.set_facecolor(SURFACE)
    homes = set(grid.homes)
    for r in range(grid.H):
        for c in range(grid.W):
            color = SHELF if (r, c) in grid.shelves else HOME if (r, c) in homes else None
            if color:
                ax.add_patch(Rectangle((c - 0.5, r - 0.5), 1, 1, color=color, lw=0))
    for r, c in grid.stations:
        ax.add_patch(Rectangle((c - 0.45, r - 0.45), 0.9, 0.9, fill=False, ec=STATION, lw=2))
    ax.set_xlim(-0.5, grid.W - 0.5)
    ax.set_ylim(grid.H - 0.5, -0.5)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)


def legend(fig):
    handles = [
        Patch(color=SHELF, label="shelf"),
        Patch(fc="none", ec=STATION, lw=2, label="delivery station"),
        Patch(color=HOME, label="robot home"),
        Patch(color=BLOCKED, label="blocked cell"),
        Patch(color=PERM_BLOCK, label="blocked for good"),
        Line2D([], [], marker="o", ls="", color=ROBOT, markersize=9, label="robot"),
        Line2D([], [], marker="o", ls="", color=EMERG, markersize=9, label="emergency task"),
        Line2D([], [], marker="X", ls="", color=BROKEN, markersize=9, label="broken down"),
        Line2D([], [], marker="X", ls="", color=LOST, markersize=9, label="dead for good"),
        Line2D([], [], marker="o", ls="", mfc="none", mec=CHANGED, mew=2.5, markersize=12,
               label="plan just changed"),
        Line2D([], [], color=CHANGED, lw=1.5, label="changed route (next 12 steps)"),
        Line2D([], [], marker="o", ls="", mfc="none", mec=BLOCKED, mew=3, markersize=12,
               label="repair failed"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=6, frameon=False, fontsize=8,
               labelcolor=MUTED)


class Panel:
    """One map view that replays a list of recorded frames."""

    def __init__(self, ax, grid, frames, label=None):
        self.ax, self.frames, self.label = ax, frames, label
        self.dyn, self.recent, self.changes = [], {}, 0
        draw_static(ax, grid)
        self.title = ax.set_title("", loc="left", fontsize=10, color=INK)
        # marker size follows the cell size so large maps stay legible
        cell_pt = ax.get_window_extent().width / grid.W * 72 / ax.figure.dpi
        self.ms = max(5.0, min(13.0, cell_pt * 0.8))

    def render(self, i, focus=None):
        ax = self.ax
        for artist in self.dyn:
            artist.remove()
        self.dyn.clear()
        f = self.frames[min(i, len(self.frames) - 1)]
        if i < len(self.frames) and not f.get("failed"):
            for aid in f["changed"]:
                self.recent[aid] = i
            self.changes += len(f["changed"])
        perm = f.get("permanent", set())
        for r, c in f["blocked"]:
            color = PERM_BLOCK if (r, c) in perm else BLOCKED
            self.dyn.append(ax.add_patch(Rectangle((c - 0.5, r - 0.5), 1, 1, color=color, alpha=0.85, lw=0)))
        hot = {a for a, k in self.recent.items() if i - k < HIGHLIGHT_FRAMES}
        for aid in hot:
            plan = f["plan"][aid]
            if len(plan) > 1:
                self.dyn.extend(ax.plot([c for _, c in plan], [r for r, _ in plan], color=CHANGED,
                                        lw=1.5, alpha=0.9))
        ms = self.ms
        for aid, (r, c) in f["pos"].items():
            if aid in f.get("lost", ()):
                self.dyn.extend(ax.plot(c, r, marker="X", color=LOST, markersize=ms, mec=SURFACE, mew=1.2))
            elif aid in f["broken"]:
                self.dyn.extend(ax.plot(c, r, marker="X", color=BROKEN, markersize=ms, mec=SURFACE, mew=1.2))
            else:
                color = EMERG if aid in f["emergency"] else ROBOT
                self.dyn.extend(ax.plot(c, r, marker="o", color=color, markersize=ms, mec=SURFACE, mew=1.2))
            if aid in hot:
                self.dyn.extend(ax.plot(c, r, marker="o", mfc="none", mec=CHANGED, mew=2.2, markersize=ms * 1.4))
            if ms >= 10:
                self.dyn.append(ax.text(c, r, str(aid), ha="center", va="center", fontsize=6.5,
                                        color="white", fontweight="bold"))
            if aid in f["carrying"]:
                self.dyn.extend(ax.plot(c + 0.33, r - 0.33, marker="s", color="#eda100",
                                        markersize=ms * 0.4, mec=SURFACE, mew=0.6))
        text = f"t = {f['t']}"
        if "tasks" in f and not self.label:
            done, total, lost = f["tasks"]
            text += f"   |   delivered {done}/{total}" + (f", lost {lost}" if lost else "")
        if f.get("failed"):
            # robots the repair could not fix, ringed in red
            for aid in f["changed"]:
                r, c = f["pos"][aid]
                self.dyn.extend(ax.plot(c, r, marker="o", mfc="none", mec=BLOCKED, mew=3, markersize=ms * 1.7))
            self.title.set_text(text + "   |   " + f["events"][-1])
            self.title.set_color(BLOCKED)
            return self.dyn
        if self.label:
            text = f"{self.label}   |   plan changes so far: {self.changes}   |   " + text
        if i >= len(self.frames):
            text += "   (all tasks done)"
        evs = sorted(f["events"], key=lambda e: focus is None or focus not in e) if i < len(self.frames) else []
        if evs and not self.label:
            text += "   |   " + "; ".join(evs[:2])
        self.title.set_text(text)
        return self.dyn


def save_snapshots(fig, panel, out_dir):
    """Stills of the most illustrative repair (most plans changed) per disruption type."""
    os.makedirs(out_dir, exist_ok=True)
    for kind in ("broke down", "emergency", "blocked"):
        best = None
        for i, f in enumerate(panel.frames):
            for e in f["events"]:
                if kind in e:
                    n = int(e.rsplit(": ", 1)[1].split()[0])
                    if best is None or n > best[0]:
                        best = (n, i)
        if best is None:
            continue
        panel.recent.clear()  # highlight only the plans altered by this event
        panel.render(best[1], focus=kind)
        fig.savefig(os.path.join(out_dir, f"snapshot_{kind.split()[0]}.png"), dpi=130,
                    bbox_inches="tight", facecolor=SURFACE)


def make_gif(name, cfg, out, fps):
    print(f"{name}: {cfg}")
    big, compare = cfg.get("big"), cfg.get("compare")
    if compare:
        runs = [simulate(cfg, s) for s in ("repair", "replan")]
        labels = ("Local repair (negotiation)", "Full replan (baseline)")
        fig, axes = plt.subplots(2, 1, figsize=(11, 11.5))
    else:
        runs = [simulate(cfg, cfg.get("strategy", "repair"))]
        labels = (None,)
        fig, ax = plt.subplots(figsize=(14, 7.6) if big else (11, 6.6))
        axes = [ax]
    fig.patch.set_facecolor(SURFACE)
    legend(fig)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.97 if compare else 0.95,
                        bottom=0.05 if compare else 0.09, hspace=0.12)
    panels = [Panel(ax, g, fr, label) for ax, (g, fr, _), label in zip(axes, runs, labels)]
    n = max(len(p.frames) for p in panels)
    if any(p.frames[-1].get("failed") for p in panels):
        n += 3 * fps  # hold the failure frame for three seconds

    def step(i):
        out_artists = []
        for p in panels:
            out_artists += p.render(i)
        return out_artists

    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    FuncAnimation(fig, step, frames=n, blit=False).save(out, writer=PillowWriter(fps=fps))
    print(f"  wrote {out} ({os.path.getsize(out) / 1e6:.1f} MB)")
    if name.startswith("failure"):
        os.makedirs("results/figures", exist_ok=True)
        panels[0].recent.clear()  # show only the robots the repair could not fix
        panels[0].render(len(panels[0].frames) - 1)
        fig.savefig(f"results/figures/snapshot_{name}.png", dpi=130, bbox_inches="tight", facecolor=SURFACE)
        print(f"  wrote results/figures/snapshot_{name}.png")
    if cfg.get("snapshots"):
        save_snapshots(fig, panels[0], "results/figures")
        print("  wrote snapshots to results/figures")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--preset", choices=list(PRESETS) + ["all"])
    ap.add_argument("--agents", type=int, default=15)
    ap.add_argument("--density", type=float, default=0.02)
    ap.add_argument("--breakdowns", type=int, default=3)
    ap.add_argument("--emergencies", type=int, default=2)
    ap.add_argument("--big", action="store_true", help="use the 26 x 55 warehouse")
    ap.add_argument("--compare", action="store_true", help="repair and full replan side by side")
    ap.add_argument("--strategy", default="repair", choices=["repair", "replan"])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--fps", type=int, default=8)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    if args.preset:
        names = list(PRESETS) if args.preset == "all" else [args.preset]
        for name in names:
            cfg = {"seed": args.seed, **PRESETS[name]}
            make_gif(name, cfg, os.path.join(GIF_DIR, f"{name}.gif"), args.fps)
    else:
        cfg = {k: getattr(args, k) for k in ("agents", "density", "breakdowns", "emergencies", "big",
                                             "compare", "strategy", "seed")}
        make_gif("custom", cfg, args.out or os.path.join(GIF_DIR, "custom.gif"), args.fps)


if __name__ == "__main__":
    main()
