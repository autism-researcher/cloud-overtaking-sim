#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regenerate the three-way baseline figures (throughput & waiting vs demand)
from the fixed, collision-free simulator.

Caches per-(lambda, policy, seed) results to out_baseline/baseline_sweep_cache.csv
so the sweep can be run incrementally, then plots:
    fig_baseline_throughput.pdf / .png
    fig_baseline_waiting.pdf   / .png

Usage:
    python3 regen_baseline.py                # full sweep + plot
    LAMS=0.05,0.10 python3 regen_baseline.py # compute only these lambdas (no plot)
    PLOT=1 python3 regen_baseline.py         # just plot from the cache
Set FIGOUT to choose where the PDFs are written (default: current dir).
"""
import os, csv, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from cloud_overtaking_sim import Cfg, simulate

Cfg.dt = 0.1; Cfg.T_warm = 60.0; Cfg.T_sim = 600.0
SEEDS = [1, 2, 3]
LAMS_ALL = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50]
POLS = ["base", "signal", "cloud"]
LAB = {"base": "Decentralized bridge", "signal": "Fixed-time signal",
       "cloud": "Cloud-assisted (proposed)"}
COL = {"base": "#c0392b", "signal": "#e0a000", "cloud": "#1f5fbf"}
MRK = {"base": "s-", "signal": "^-", "cloud": "o-"}

OUTDIR = "out_baseline"; os.makedirs(OUTDIR, exist_ok=True)
CACHE = os.path.join(OUTDIR, "baseline_sweep_cache.csv")
FIGOUT = os.environ.get("FIGOUT", ".")


def load_cache():
    d = {}
    if os.path.exists(CACHE):
        for r in csv.DictReader(open(CACHE)):
            d[(float(r["lam"]), r["pol"], int(r["seed"]))] = (float(r["Q"]), float(r["W"]))
    return d


def append_cache(key, Q, W):
    new = not os.path.exists(CACHE)
    with open(CACHE, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["lam", "pol", "seed", "Q", "W"])
        w.writerow([key[0], key[1], key[2], f"{Q:.3f}", f"{W:.3f}"])


def compute(lams):
    cache = load_cache()
    for lam in lams:
        for pol in POLS:
            for s in SEEDS:
                k = (lam, pol, s)
                if k in cache:
                    continue
                r = simulate(lam=lam, policy=pol, rng=np.random.default_rng(s))
                append_cache(k, r["Q"], r["wait_mean"])
                print(f"  computed lam={lam} {pol} seed={s}: Q={r['Q']:.0f} W={r['wait_mean']:.1f}", flush=True)


def agg(metric_idx):
    cache = load_cache()
    out = {p: ([], [], []) for p in POLS}  # lam, mean, std
    for lam in LAMS_ALL:
        for p in POLS:
            vals = [cache[(lam, p, s)][metric_idx] for s in SEEDS if (lam, p, s) in cache]
            if len(vals) == len(SEEDS):
                out[p][0].append(lam); out[p][1].append(np.mean(vals)); out[p][2].append(np.std(vals))
    return out


def plot():
    # throughput
    fig, ax = plt.subplots(figsize=(5.0, 3.4))
    data = agg(0)
    for p in POLS:
        x, m, e = data[p]
        ax.errorbar(x, m, yerr=e, fmt=MRK[p], color=COL[p], capsize=3, lw=1.4, ms=5, label=LAB[p])
    ax.set_xlabel("arrival rate $\\lambda$ [veh/s/dir] (symmetric)")
    ax.set_ylabel("throughput $Q$ [veh/h]")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    for e in ("pdf", "png"):
        fig.savefig(f"{FIGOUT}/fig_baseline_throughput.{e}", dpi=300)
    plt.close(fig)
    # waiting
    fig, ax = plt.subplots(figsize=(5.0, 3.4))
    data = agg(1)
    for p in POLS:
        x, m, e = data[p]
        ax.errorbar(x, m, yerr=e, fmt=MRK[p], color=COL[p], capsize=3, lw=1.4, ms=5, label=LAB[p])
    ax.set_xlabel("arrival rate $\\lambda$ [veh/s/dir] (symmetric)")
    ax.set_ylabel("average waiting time [s]")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    for e in ("pdf", "png"):
        fig.savefig(f"{FIGOUT}/fig_baseline_waiting.{e}", dpi=300)
    plt.close(fig)
    print(f"wrote fig_baseline_throughput / fig_baseline_waiting to {FIGOUT}")


if __name__ == "__main__":
    if os.environ.get("PLOT") == "1":
        plot()
    else:
        lams = os.environ.get("LAMS")
        lams = [float(x) for x in lams.split(",")] if lams else LAMS_ALL
        compute(lams)
        if not os.environ.get("LAMS"):
            plot()
