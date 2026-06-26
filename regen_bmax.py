#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fairness-cap sensitivity sweep at balanced, saturated demand.

Holds demand at the balanced saturation point (lambda=0.30 veh/s per direction)
and sweeps the fairness cap B_max (Cfg.max_batch). It shows that the proposed
coordinator's throughput climbs toward the idealized fixed-time signal as B_max
grows, while the guaranteed opposing-direction wait bound
    W_opp = B_max + g_same + D_max + 2 g_opp
grows linearly. The headline operating point B_max=12 s is therefore a chosen
point on a throughput--fairness frontier, not a capacity ceiling.

Caches to out_baseline/bmax_cache.csv so it can be run incrementally.
Usage:
    python3 regen_bmax.py                  # full sweep + plot
    PLOT=1 python3 regen_bmax.py           # plot from cache
Set FIGOUT for the output directory (default: current dir).
"""
import os, csv, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from cloud_overtaking_sim import Cfg, simulate

Cfg.dt = 0.1; Cfg.T_warm = 60.0; Cfg.T_sim = 600.0      # production config
LAM = 0.30                                              # balanced saturation
SEEDS = [1, 2, 3]
BMAX = [6.0, 9.0, 12.0, 16.0, 20.0, 26.0, 34.0, 45.0, 60.0]
D_MAX = 5.8                                             # max crossing duration [s]
WOPP = lambda b: b + Cfg.g_same + D_MAX + 2.0 * Cfg.g_opp

OUTDIR = "out_baseline"; os.makedirs(OUTDIR, exist_ok=True)
CACHE = os.path.join(OUTDIR, "bmax_cache.csv")
FIGOUT = os.environ.get("FIGOUT", ".")


def load():
    d = {}
    if os.path.exists(CACHE):
        for r in csv.DictReader(open(CACHE)):
            d[(float(r["bmax"]), r["pol"], int(r["seed"]))] = float(r["Q"])
    return d


def append(key, Q):
    new = not os.path.exists(CACHE)
    with open(CACHE, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["bmax", "pol", "seed", "Q"])
        w.writerow([key[0], key[1], key[2], f"{Q:.3f}"])


def compute():
    cache = load()
    base = Cfg.max_batch
    # cloud sweep over B_max
    for b in BMAX:
        Cfg.max_batch = b
        for s in SEEDS:
            k = (b, "cloud", s)
            if k in cache:
                continue
            r = simulate(lam=LAM, policy="cloud", rng=np.random.default_rng(s))
            append(k, r["Q"]); cache[k] = r["Q"]
            print(f"  B_max={b:4.0f} cloud seed={s}: Q={r['Q']:.0f}", flush=True)
    Cfg.max_batch = base
    # idealized signal reference (B_max-independent)
    for s in SEEDS:
        k = (0.0, "signal", s)
        if k in cache:
            continue
        r = simulate(lam=LAM, policy="signal", rng=np.random.default_rng(s))
        append(k, r["Q"]); cache[k] = r["Q"]
        print(f"  signal seed={s}: Q={r['Q']:.0f}", flush=True)


def plot():
    cache = load()
    x, m, e = [], [], []
    for b in BMAX:
        vals = [cache[(b, "cloud", s)] for s in SEEDS if (b, "cloud", s) in cache]
        if len(vals) == len(SEEDS):
            x.append(b); m.append(np.mean(vals)); e.append(np.std(vals))
    sig = [cache[(0.0, "signal", s)] for s in SEEDS if (0.0, "signal", s) in cache]
    sig_mean = np.mean(sig) if sig else None

    fig, ax = plt.subplots(figsize=(5.6, 3.9))
    ax.errorbar(x, m, yerr=e, fmt="o-", color="#1f5fbf", capsize=3, lw=2.2, ms=7,
                markeredgecolor="white", markeredgewidth=0.6, elinewidth=1.2,
                label="Cloud-assisted (proposed)")
    if sig_mean is not None:
        ax.axhline(sig_mean, ls="--", lw=2.0, color="#e0a000",
                   label="Idealized fixed-time signal")
    ax.axvline(12.0, ls=":", lw=1.6, color="0.4")
    ax.annotate("operating point\n$B_{\\max}=12$ s", xy=(12, min(m)),
                xytext=(15, min(m) + 0.18 * (max(m) - min(m))),
                fontsize=9.5, color="0.3")
    ax.set_xlabel("fairness cap $B_{\\max}$ [s]", fontsize=14)
    ax.set_ylabel("throughput $Q$ [veh/h]", fontsize=14)
    ax.tick_params(axis="both", labelsize=12)
    ax.grid(alpha=0.35, lw=0.6)
    ax.legend(fontsize=10.5, framealpha=0.92, loc="lower right")

    # secondary axis: guaranteed opposing-wait bound (linear in B_max)
    ax2 = ax.twinx()
    bb = np.array([min(x), max(x)])
    ax2.plot(bb, WOPP(bb), ls="-.", lw=1.8, color="#c0392b")
    ax2.set_ylabel("opposing-wait bound $W_{\\mathrm{opp}}$ [s]", fontsize=13,
                   color="#c0392b")
    ax2.tick_params(axis="y", labelsize=11, colors="#c0392b")

    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(f"{FIGOUT}/fig_bmax_tradeoff.{ext}", dpi=300)
    plt.close(fig)
    print(f"wrote fig_bmax_tradeoff to {FIGOUT}")
    if sig_mean is not None:
        print(f"signal reference Q={sig_mean:.0f}; cloud Q range "
              f"{min(m):.0f}..{max(m):.0f} over B_max {min(x):.0f}..{max(x):.0f}")


if __name__ == "__main__":
    if os.environ.get("PLOT") == "1":
        plot()
    else:
        compute()
        plot()
