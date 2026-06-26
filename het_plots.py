#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, csv, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from collections import defaultdict
FIGOUT=os.environ.get("FIGOUT",".")
COL={"base":"#c0392b","signal":"#e0a000","actuated":"#2ca02c","cloud":"#1f5fbf"}
MRK={"base":"s-","signal":"^-","actuated":"D--","cloud":"o-"}
LAB={"base":"Decentralized bridge","signal":"Fixed-time signal","actuated":"Actuated signal (idealized)","cloud":"Cloud-assisted (proposed)"}
def agg(path,key):
    D=defaultdict(lambda: defaultdict(list))
    for r in csv.DictReader(open(path)):
        D[(float(r[key]),r['pol'])]
    D=defaultdict(lambda: defaultdict(list))
    for r in csv.DictReader(open(path)):
        D[(float(r[key]),r['pol'])]['Q'].append(float(r['Q']))
        if 'W' in r: D[(float(r[key]),r['pol'])]['W'].append(float(r['W']))
    return D
# --- baseline sweep ---
LAMS=[0.02,0.04,0.06,0.08,0.10,0.12,0.14]
D=agg("out_real/het_sweep.csv","lam")
def sweepfig(metric,ylab,fname):
    fig,ax=plt.subplots(figsize=(5.6,3.9))
    for p in ["base","signal","actuated","cloud"]:
        x=LAMS; m=[np.mean(D[(l,p)][metric]) for l in LAMS]; e=[np.std(D[(l,p)][metric]) for l in LAMS]
        ax.errorbar(x,m,yerr=e,fmt=MRK[p],color=COL[p],capsize=3,lw=2.2,ms=7,
                    markeredgecolor="white",markeredgewidth=0.6,elinewidth=1.2,label=LAB[p])
    ax.set_xlabel("arrival rate $\\lambda$ [veh/s/dir] (symmetric)",fontsize=14)
    ax.set_ylabel(ylab,fontsize=14); ax.tick_params(axis="both",labelsize=12)
    ax.grid(alpha=0.35,lw=0.6); ax.legend(fontsize=10.5,framealpha=0.92,loc="best")
    fig.tight_layout()
    for e2 in ("pdf","png"): fig.savefig(f"{FIGOUT}/{fname}.{e2}",dpi=300)
    plt.close(fig)
sweepfig("Q","throughput $Q$ [veh/h]","fig_baseline_throughput")
sweepfig("W","average waiting time [s]","fig_baseline_waiting")
# --- asymmetric ---
A=defaultdict(lambda: defaultdict(list))
for r in csv.DictReader(open("out_real/het_asym.csv")):
    A[(float(r['share']),r['pol'])]['Q'].append(float(r['Q']))
SH=[0.50,0.60,0.70,0.80,0.90,0.95]
fig,ax=plt.subplots(figsize=(5.6,3.9))
for p in ["base","signal","cloud"]:
    x=[s*100 for s in SH]; m=[np.mean(A[(s,p)]['Q']) for s in SH]; e=[np.std(A[(s,p)]['Q']) for s in SH]
    ax.errorbar(x,m,yerr=e,fmt=MRK[p],color=COL[p],capsize=3,lw=2.2,ms=7,
                markeredgecolor="white",markeredgewidth=0.6,elinewidth=1.2,label=LAB[p])
ax.set_xlabel("heavy-direction share of demand [%] (total fixed at 0.16 veh/s, heterogeneous)",fontsize=14)
ax.set_ylabel("total throughput $Q$ [veh/h]",fontsize=14); ax.tick_params(axis="both",labelsize=12)
ax.grid(alpha=0.35,lw=0.6); ax.legend(fontsize=10.5,framealpha=0.92,loc="best")
fig.tight_layout()
for e2 in ("pdf","png"): fig.savefig(f"{FIGOUT}/fig_asym_throughput.{e2}",dpi=300)
plt.close(fig)
print("wrote realistic baseline throughput/waiting + asym figures")
