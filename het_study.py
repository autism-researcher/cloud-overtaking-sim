#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Main study with HETEROGENEITY ON by default (per-vehicle T_head/a_max/b_comf,
15% heavy vehicles). Realistic demand. Captures Q/W/W95/Tt/stops. Resumable."""
import os, csv, numpy as np
import het_sim as H
H.Cfg.dt=0.1; H.Cfg.T_warm=60.0; H.Cfg.T_sim=600.0
H.Cfg.het_drivers=True; H.Cfg.truck_frac=0.15
LAMS=[0.02,0.04,0.06,0.08,0.10,0.12,0.14]; POLS=["base","signal","actuated","cloud"]; SEEDS=list(range(1,11))
OUT="out_real"; os.makedirs(OUT,exist_ok=True); C=os.path.join(OUT,"het_sweep.csv")
def load():
    d={}
    if os.path.exists(C):
        for r in csv.DictReader(open(C)): d[(float(r["lam"]),r["pol"],int(r["seed"]))]=r
    return d
def app(k,r):
    new=not os.path.exists(C)
    with open(C,"a",newline="") as f:
        w=csv.writer(f)
        if new: w.writerow(["lam","pol","seed","Q","W","W95","Tt","stops"])
        w.writerow([k[0],k[1],k[2],f"{r['Q']:.2f}",f"{r['wait_mean']:.2f}",f"{r['wait_p95']:.2f}",f"{r['travel_mean']:.2f}",f"{r['stops_mean']:.3f}"])
if __name__=="__main__":
    lams=os.environ.get("LAMS"); lams=[float(x) for x in lams.split(",")] if lams else LAMS
    have=load()
    for lam in lams:
        for p in POLS:
            for s in SEEDS:
                k=(lam,p,s)
                if k in have: continue
                r=H.simulate(lam=lam,policy=p,rng=np.random.default_rng(s))
                app(k,r); print(f"  lam={lam:.2f} {p:6s} s{s}: Q={r['Q']:.0f} W={r['wait_mean']:.1f}",flush=True)
