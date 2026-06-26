#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Robustness to vehicle heterogeneity (paper Section V-H, Table).
Uses het_sim.py (the core simulator with optional per-vehicle dynamics) with
heterogeneity enabled: per-vehicle T_head/a_max/b_comf and a 15% heavy-vehicle
share. Reproduces the throughput comparison at the representative demand."""
import numpy as np, het_sim as H
H.Cfg.dt=0.1; H.Cfg.T_warm=60.0; H.Cfg.T_sim=600.0
H.Cfg.het_drivers=True; H.Cfg.truck_frac=0.15
SEEDS=[1,2,3,4]
def run(lam,pol):
    Q=[H.simulate(lam=lam,policy=pol,rng=np.random.default_rng(s))["Q"] for s in SEEDS]
    return np.mean(Q),np.std(Q)
print("Heterogeneous fleet (T_head in [1.0,2.2]s, randomized a_max/b_comf, 15% trucks)")
print("balanced lambda=0.30:")
for p in ["base","signal","cloud"]:
    m,sd=run(0.30,p); print(f"  {p:7s}: Q={m:.0f}+/-{sd:.0f}")
print("asymmetric 83/17 (lam+=0.40, lam-=0.08):")
for p in ["base","signal","cloud"]:
    m,sd=run({+1:0.40,-1:0.08},p); print(f"  {p:7s}: Q={m:.0f}+/-{sd:.0f}")
