#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Per-direction fairness experiment: demonstrates the fairness proposition
empirically. For each policy and demand split it reports the mean waiting of the
heavy and light directions and Jain's fairness index on the two,

    J = (W_h + W_l)^2 / (2 (W_h^2 + W_l^2)),

which is 1 when both directions are treated equally and falls toward 0.5 as one
direction is starved relative to the other.

Caches to out_baseline/fairness_cache.csv so it can be run incrementally.
Usage:
    CELLS=bal python3 regen_fairness.py    # compute one cell group, then print
    python3 regen_fairness.py              # compute all, print table
"""
import os, csv, numpy as np
from cloud_overtaking_sim import Cfg, simulate

Cfg.dt = 0.1; Cfg.T_warm = 60.0; Cfg.T_sim = 600.0
SEEDS = [1, 2, 3, 4, 5]
# (label, lambda_heavy, lambda_light, split text)
CELLS = {
    "bal":  ("Balanced (50/50)",   0.30, 0.30),
    "a83":  ("Asymmetric (83/17)", 0.40, 0.08),
    "a95":  ("Asymmetric (95/5)",  0.40, 0.02),
}
POLS = ["base", "signal", "cloud"]
LAB = {"base": "Bridge", "signal": "Signal", "cloud": "Cloud"}
OUTDIR = "out_baseline"; os.makedirs(OUTDIR, exist_ok=True)
CACHE = os.path.join(OUTDIR, "fairness_cache.csv")


def load():
    d = {}
    if os.path.exists(CACHE):
        for r in csv.DictReader(open(CACHE)):
            d[(r["cell"], r["pol"], int(r["seed"]))] = (float(r["Wh"]), float(r["Wl"]))
    return d


def append(key, Wh, Wl):
    new = not os.path.exists(CACHE)
    with open(CACHE, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["cell", "pol", "seed", "Wh", "Wl"])
        w.writerow([key[0], key[1], key[2], f"{Wh:.3f}", f"{Wl:.3f}"])


def compute(cell_keys):
    cache = load()
    for ck in cell_keys:
        _, lh, ll = CELLS[ck]
        for pol in POLS:
            for s in SEEDS:
                k = (ck, pol, s)
                if k in cache:
                    continue
                r = simulate(lam={+1: lh, -1: ll}, policy=pol, rng=np.random.default_rng(s))
                append(k, r["wait_pos"], r["wait_neg"])
                print(f"  {ck} {pol} seed={s}: Wh={r['wait_pos']:.1f} Wl={r['wait_neg']:.1f}", flush=True)


def jain(a, b):
    return (a + b) ** 2 / (2.0 * (a * a + b * b)) if (a * a + b * b) > 0 else 1.0


def report():
    cache = load()
    print("\ncell                 policy   W_heavy[s]  W_light[s]   Jain")
    for ck, (label, lh, ll) in CELLS.items():
        for pol in POLS:
            Wh = [cache[(ck, pol, s)][0] for s in SEEDS if (ck, pol, s) in cache]
            Wl = [cache[(ck, pol, s)][1] for s in SEEDS if (ck, pol, s) in cache]
            if len(Wh) == len(SEEDS):
                mh, ml = np.mean(Wh), np.mean(Wl)
                print(f"{label:20s} {LAB[pol]:7s}  {mh:9.1f}  {ml:9.1f}   {jain(mh, ml):.3f}")


if __name__ == "__main__":
    sel = os.environ.get("CELLS")
    keys = sel.split(",") if sel else list(CELLS.keys())
    compute(keys)
    if not sel:
        report()
