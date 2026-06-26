#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Realistic asymmetric-demand sweep: total demand fixed below capacity, vary
the heavy-direction share. base/signal/cloud."""
import os, csv, numpy as np
from cloud_overtaking_sim import Cfg, simulate
Cfg.dt=0.1; Cfg.T_warm=60.0; Cfg.T_sim=600.0
TOTAL=0.16
SHARES=[0.50,0.60,0.70,0.80,0.90,0.95]
POLS=["base","signal","cloud"]; SEEDS=[1,2,3,4,5]
OUT="out_real"; os.makedirs(OUT,exist_ok=True)
CACHE=os.path.join(OUT,"asym_real.csv")
def load():
    d={}
    if os.path.exists(CACHE):
        for r in csv.DictReader(open(CACHE)): d[(float(r["share"]),r["pol"],int(r["seed"]))]=r
    return d
def app(k,r):
    new=not os.path.exists(CACHE)
    with open(CACHE,"a",newline="") as f:
        w=csv.writer(f)
        if new: w.writerow(["share","pol","seed","Q","Wh","Wl","stops"])
        w.writerow([k[0],k[1],k[2],f"{r['Q']:.2f}",f"{r['wait_pos']:.2f}",f"{r['wait_neg']:.2f}",f"{r['stops_mean']:.3f}"])
have=load()
sel=os.environ.get("SHARES")
shs=[float(x) for x in sel.split(",")] if sel else SHARES
for sh in shs:
    lam={+1:TOTAL*sh,-1:TOTAL*(1-sh)}
    for p in POLS:
        for s in SEEDS:
            k=(sh,p,s)
            if k in have: continue
            r=simulate(lam=lam,policy=p,rng=np.random.default_rng(s))
            app(k,r); print(f"  share={sh:.2f} {p:6s} s{s}: Q={r['Q']:.0f}",flush=True)
