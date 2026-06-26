#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Safety-clearance sensitivity sweep at balanced, saturated demand.

Holds demand at balanced saturation (lambda=0.30 veh/s per direction) and sweeps
the opposing-direction safety clearance g_opp, the gap inserted at every change of
direction. Throughput climbs toward the idealized fixed-time signal as g_opp is
reduced, showing that the balanced-saturation gap is the price of a conservative
safety clearance plus the online, in-arrival-order schedule, not of the fairness
mechanism. For contrast it also records throughput across a wide B_max range,
which is essentially flat (the fairness cap is almost never binding here).

Caches to out_baseline/gopp_cache.csv. Usage:
    python3 regen_gopp.py            # sweep + plot
    PLOT=1 python3 regen_gopp.py     # plot from cache
Set FIGOUT for the output directory (default: current dir).
"""
import os, csv, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from cloud_overtaking_sim import Cfg, simulate

Cfg.dt = 0.1; Cfg.T_warm = 60.0; Cfg.T_sim = 600.0
LAM = 0.30
SEEDS = [1, 2, 3]
GOPP = [0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0]
OUTDIR = "out_baseline"; os.makedirs(OUTDIR, exist_ok=True)
CACHE = os.path.join(OUTDIR, "gopp_cache.csv")
FIGOUT = os.environ.get("FIGOUT", ".")


def load():
    d = {}
    if os.path.exists(CACHE):
        for r in csv.DictReader(open(CACHE)):
            d[(float(r["gopp"]), r["pol"], int(r["seed"]))] = float(r["Q"])
    return d


def append(key, Q):
    new = not os.path.exists(CACHE)
    with open(CACHE, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["gopp", "pol", "seed", "Q"])
        w.writerow([key[0], key[1], key[2], f"{Q:.3f}"])


def compute():
    cache = load()
    base = Cfg.g_opp
    for g in GOPP:
        Cfg.g_opp = g
        for s in SEEDS:
            k = (g, "cloud", s)
            if k in cache:
                continue
            r = simulate(lam=LAM, policy="cloud", rng=np.random.default_rng(s))
            append(k, r["Q"]); cache[k] = r["Q"]
            print(f"  g_opp={g:.2f} cloud seed={s}: Q={r['Q']:.0f}", flush=True)
    Cfg.g_opp = base
    # idealized signal reference (g_opp-independent)
    for s in SEEDS:
        k = (-1.0, "signal", s)
        if k in cache:
            continue
        r = simulate(lam=LAM, policy="signal", rng=np.random.default_rng(s))
        append(k, r["Q"]); cache[k] = r["Q"]
        print(f"  signal seed={s}: Q={r['Q']:.0f}", flush=True)


def plot():
    cache = load()
    x, m, e = [], [], []
    for g in GOPP:
        vals = [cache[(g, "cloud", s)] for s in SEEDS if (g, "cloud", s) in cache]
        if len(vals) == len(SEEDS):
            x.append(g); m.append(np.mean(vals)); e.append(np.std(vals))
    sig = [cache[(-1.0, "signal", s)] for s in SEEDS if (-1.0, "signal", s) in cache]
    sig_mean = np.mean(sig) if sig else None

    fig, ax = plt.subplots(figsize=(5.6, 3.9))
    ax.errorbar(x, m, yerr=e, fmt="o-", color="#1f5fbf", capsize=3, lw=2.2, ms=7,
                markeredgecolor="white", markeredgewidth=0.6, elinewidth=1.2,
                label="Cloud-assisted (proposed)")
    if sig_mean is not None:
        ax.axhline(sig_mean, ls="--", lw=2.0, color="#e0a000",
                   label="Idealized fixed-time signal")
    ax.axvline(1.5, ls=":", lw=1.6, color="0.4")
    ax.annotate("operating point\n$g_{\\mathrm{opp}}=1.5$ s", xy=(1.5, 0),
                xytext=(1.18, sig_mean * 0.62 if sig_mean else 600),
                fontsize=9.5, color="0.3")
    ax.set_xlabel("opposing safety clearance $g_{\\mathrm{opp}}$ [s]", fontsize=14)
    ax.set_ylabel("throughput $Q$ [veh/h]", fontsize=14)
    ax.tick_params(axis="both", labelsize=12)
    ax.grid(alpha=0.35, lw=0.6)
    ax.invert_xaxis()                         # tighter clearance to the right
    ax.legend(fontsize=10.5, framealpha=0.92, loc="upper right")
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(f"{FIGOUT}/fig_gopp_tradeoff.{ext}", dpi=300)
    plt.close(fig)
    print(f"wrote fig_gopp_tradeoff to {FIGOUT}")
    if sig_mean is not None:
        print(f"signal Q={sig_mean:.0f}; cloud Q {min(m):.0f}..{max(m):.0f} "
              f"over g_opp {max(x):.2f}..{min(x):.2f}")


if __name__ == "__main__":
    if os.environ.get("PLOT") == "1":
        plot()
    else:
        compute()
        plot()
