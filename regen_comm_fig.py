#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Re-plot fig_comm as a single-column, two-panel (stacked) figure."""
import os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
FIGOUT=os.environ.get("FIGOUT",".")
CQ="#1f5fbf"; CW="#c0392b"

lat_x=[0,50,100,200,500]
lat_Q=[678,679,680,680,688]; lat_Qs=[26,25,26,26,36]
lat_W=[2.0,1.8,2.0,2.1,2.1]; lat_Ws=[0.7,0.6,0.6,0.7,0.3]
loss_x=[0,5,10,20]
loss_Q=[680,721,726,659]; loss_Qs=[26,21,37,55]
loss_W=[2.0,6.2,2.7,2.6]; loss_Ws=[0.6,7.3,1.6,3.3]

def panel(ax,x,Q,Qs,W,Ws,xlabel,title):
    axr=ax.twinx()
    ax.errorbar(x,Q,yerr=Qs,fmt="o-",color=CQ,capsize=2.5,lw=1.8,ms=5,label="throughput")
    axr.errorbar(x,W,yerr=Ws,fmt="s--",color=CW,capsize=2.5,lw=1.6,ms=4,label="waiting")
    ax.set_xlabel(xlabel,fontsize=9.5)
    ax.set_ylabel("$Q$ [veh/h]",fontsize=9.5,color=CQ)
    axr.set_ylabel("waiting [s]",fontsize=9.5,color=CW)
    ax.tick_params(axis="both",labelsize=8); ax.tick_params(axis="y",colors=CQ)
    axr.tick_params(axis="y",labelsize=8,colors=CW)
    ax.set_ylim(550,800); axr.set_ylim(0,20)
    ax.grid(alpha=0.3,lw=0.5)
    ax.set_title(title,fontsize=9.5,fontweight="bold",loc="left")

fig,(a1,a2)=plt.subplots(2,1,figsize=(3.5,4.7))
panel(a1,lat_x,lat_Q,lat_Qs,lat_W,lat_Ws,"latency [ms] (zero loss)","(a) Latency")
panel(a2,loss_x,loss_Q,loss_Qs,loss_W,loss_Ws,"packet loss [%] (at 100 ms)","(b) Packet loss")
fig.tight_layout(h_pad=1.6)
for e in ("pdf","png"): fig.savefig(f"{FIGOUT}/fig_comm.{e}",dpi=300)
print("wrote single-column fig_comm to",FIGOUT)
