#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Realistic-demand study: base/signal/cloud across a demand range at or below
corridor capacity (~742 veh/h), where the corridor flows instead of gridlocking.
Captures Q, mean/95th waiting, travel time, and stops. Resumable cache."""
import os, csv, numpy as np
from cloud_overtaking_sim import Cfg, simulate
Cfg.dt=0.1; Cfg.T_warm=60.0; Cfg.T_sim=600.0
LAMS_ALL=[0.02,0.04,0.06,0.08,0.10,0.12,0.14]
POLS=["base","signal","cloud"]
SEEDS=[1,2,3,4,5,6,7,8,9,10]
OUT="out_real"; os.makedirs(OUT, exist_ok=True)
CACHE=os.path.join(OUT,"sweep_real.csv")
def load():
    d={}
    if os.path.exists(CACHE):
        for r in csv.DictReader(open(CACHE)):
            d[(float(r["lam"]),r["pol"],int(r["seed"]))]=r
    return d
def app(k,r):
    new=not os.path.exists(CACHE)
    with open(CACHE,"a",newline="") as f:
        w=csv.writer(f)
        if new: w.writerow(["lam","pol","seed","Q","W","W95","Tt","stops"])
        w.writerow([k[0],k[1],k[2],f"{r['Q']:.2f}",f"{r['wait_mean']:.2f}",
                    f"{r['wait_p95']:.2f}",f"{r['travel_mean']:.2f}",f"{r['stops_mean']:.3f}"])
if __name__=="__main__":
    lams=os.environ.get("LAMS")
    lams=[float(x) for x in lams.split(",")] if lams else LAMS_ALL
    have=load()
    for lam in lams:
        for pol in POLS:
            for s in SEEDS:
                k=(lam,pol,s)
                if k in have: continue
                r=simulate(lam=lam,policy=pol,rng=np.random.default_rng(s))
                app(k,r)
                print(f"  lam={lam:.2f} {pol:6s} s{s}: Q={r['Q']:.0f} W={r['wait_mean']:.1f}",flush=True)
