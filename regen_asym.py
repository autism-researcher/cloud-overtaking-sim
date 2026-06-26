#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regenerate the asymmetric-demand figure (total throughput vs heavy-direction
share, total demand fixed) from the fixed, collision-free simulator.

Caches per-(share, policy, seed) results to out_baseline/asym_sweep_cache.csv so
the sweep can be run incrementally, then plots fig_asym_throughput.pdf/.png.

Usage:
    python3 regen_asym.py                  # full sweep + plot
    SHARES=0.5,0.6 python3 regen_asym.py   # compute only these shares (no plot)
    PLOT=1 python3 regen_asym.py           # plot from cache
Set FIGOUT for the output directory (default: current dir).
"""
import os, csv, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from cloud_overtaking_sim import Cfg, simulate

Cfg.dt = 0.1; Cfg.T_warm = 60.0; Cfg.T_sim = 600.0
TOTAL = 0.40                     # total demand (sum of both directions) [veh/s]
SEEDS = [1,2,3,4,5]
SHARES_ALL = [0.50, 0.60, 0.70, 0.80, 0.90, 0.95]
POLS = ["base", "signal", "cloud"]
LAB = {"base": "Decentralized bridge", "signal": "Fixed-time signal",
       "cloud": "Cloud-assisted (proposed)"}
COL = {"base": "#c0392b", "signal": "#e0a000", "cloud": "#1f5fbf"}
MRK = {"base": "s-", "signal": "^-", "cloud": "o-"}

OUTDIR = "out_baseline"; os.makedirs(OUTDIR, exist_ok=True)
CACHE = os.path.join(OUTDIR, "asym_sweep_cache.csv")
FIGOUT = os.environ.get("FIGOUT", ".")


def load_cache():
    d = {}
    if os.path.exists(CACHE):
        for r in csv.DictReader(open(CACHE)):
            d[(float(r["share"]), r["pol"], int(r["seed"]))] = float(r["Q"])
    return d


def append_cache(key, Q):
    new = not os.path.exists(CACHE)
    with open(CACHE, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["share", "pol", "seed", "Q"])
        w.writerow([key[0], key[1], key[2], f"{Q:.3f}"])


def compute(shares):
    cache = load_cache()
    for sh in shares:
        lam = {+1: TOTAL * sh, -1: TOTAL * (1.0 - sh)}
        for pol in POLS:
            for s in SEEDS:
                k = (sh, pol, s)
                if k in cache:
                    continue
                r = simulate(lam=lam, policy=pol, rng=np.random.default_rng(s))
                append_cache(k, r["Q"])
                print(f"  share={sh:.2f} {pol} seed={s}: Qtot={r['Q']:.0f}", flush=True)


def plot():
    cache = load_cache()
    fig, ax = plt.subplots(figsize=(5.6, 3.9))
    for p in POLS:
        x, m, e = [], [], []
        for sh in SHARES_ALL:
            vals = [cache[(sh, p, s)] for s in SEEDS if (sh, p, s) in cache]
            if len(vals) == len(SEEDS):
                x.append(sh * 100); m.append(np.mean(vals)); e.append(np.std(vals))
        ax.errorbar(x, m, yerr=e, fmt=MRK[p], color=COL[p], capsize=3, lw=2.2, ms=7, markeredgecolor='white', markeredgewidth=0.6, elinewidth=1.2, label=LAB[p])
    ax.set_xlabel("heavy-direction share of demand [%] (total fixed at 0.40 veh/s)", fontsize=14)
    ax.set_ylabel("total throughput $Q$ [veh/h]", fontsize=14)
    ax.legend(fontsize=11, framealpha=0.9, loc='best'); ax.grid(alpha=0.35, lw=0.6)
    ax.tick_params(axis='both', labelsize=12)
    fig.tight_layout()
    for e in ("pdf", "png"):
        fig.savefig(f"{FIGOUT}/fig_asym_throughput.{e}", dpi=300)
    plt.close(fig)
    print(f"wrote fig_asym_throughput to {FIGOUT}")


if __name__ == "__main__":
    if os.environ.get("PLOT") == "1":
        plot()
    else:
        sh = os.environ.get("SHARES")
        sh = [float(x) for x in sh.split(",")] if sh else SHARES_ALL
        compute(sh)
        if not os.environ.get("SHARES"):
            plot()
