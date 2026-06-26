#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Signature schematic: the Cloud-Based Reservation Scheduling Framework."""
import os
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, FancyArrowPatch, Ellipse

FIGOUT = os.environ.get("FIGOUT", ".")
CPOS = "#1f5fbf"; CNEG = "#e0820a"; COBS = "#c0392b"; GREY = "0.45"
WINDOWS = [(1.0, 3.3, +1, 3), (3.8, 5.2, -1, 1), (5.7, 7.5, +1, 2)]
GOPP_GAPS = [(3.3, 3.8), (5.2, 5.7)]

def car(ax, x, y, w=0.42, h=0.22, color=CPOS, facing=+1):
    ax.add_patch(FancyBboxPatch((x-w/2, y-h/2), w, h,
                 boxstyle="round,pad=0.01,rounding_size=0.06",
                 fc=color, ec="black", lw=0.8, zorder=5))
    ax.add_patch(Rectangle((x+facing*0.06-0.05, y-h/2+0.03), 0.10, h-0.06,
                 fc="white", ec="none", alpha=0.6, zorder=6))

def cloud(ax, cx, cy):
    for dx, dy, r in [(-0.55,0,0.34),(-0.18,0.12,0.40),(0.22,0.06,0.36),(0.55,-0.02,0.30),(0.0,-0.16,0.40)]:
        ax.add_patch(Ellipse((cx+dx, cy+dy), r*2, r*1.7, fc="white", ec=GREY, lw=1.4, zorder=4))
    ax.add_patch(Rectangle((cx-0.7, cy-0.30), 1.4, 0.32, fc="white", ec="none", zorder=4))
    ax.text(cx, cy+0.13, "Cloud coordinator", ha="center", va="center", fontsize=9.3, fontweight="bold", zorder=7)
    ax.text(cx, cy-0.27, "reservation scheduler $\\cdot$ fairness\ncontroller $\\cdot$ slot assignment", ha="center", va="center", fontsize=6.2, color="0.32", zorder=7)

def scene(ax):
    ax.set_xlim(0,10); ax.set_ylim(0,6.2); ax.axis("off")
    rx0, rx1, ry0, ry1 = 0.3, 9.7, 2.25, 3.75
    ax.add_patch(Rectangle((rx0,ry0), rx1-rx0, ry1-ry0, fc="0.88", ec="0.5", lw=1.2, zorder=1))
    ax.plot([rx0,rx1],[3.0,3.0], ls=(0,(6,6)), color="white", lw=1.6, zorder=2)
    for (a,b,c) in [(2.4,4.1,CPOS),(6.0,7.7,CNEG)]:
        ax.add_patch(Rectangle((a,ry0), b-a, ry1-ry0, fc=c, ec="none", alpha=0.13, zorder=1))
    ax.text(3.25,4.05,"precaution zone $L_D$", ha="center", fontsize=9, color=CPOS)
    ax.text(6.85,1.95,"precaution zone $L_D$", ha="center", fontsize=9, color=CNEG)
    sx0, sx1 = 4.55, 5.95
    ax.add_patch(Rectangle((sx0,ry0), sx1-sx0, ry1-ry0, fc="none", ec="0.3", lw=1.4, ls="--", zorder=3))
    ax.annotate("", xy=(sx1,1.95), xytext=(sx0,1.95), arrowprops=dict(arrowstyle="<->", color="0.3", lw=1.2))
    ax.text((sx0+sx1)/2,1.62,"shared section $L_o$", ha="center", fontsize=9)
    ax.add_patch(Rectangle((sx0+0.30,3.05),0.7,0.6, fc=COBS, ec="black", lw=0.8, zorder=5))
    ax.text(sx0+0.65,3.95,"obstacle", ha="center", fontsize=9, color=COBS, fontweight="bold")
    ax.annotate("", xy=(2.0,3.38), xytext=(0.7,3.38), arrowprops=dict(arrowstyle="-|>", color=CPOS, lw=2))
    ax.text(0.6,3.62,"direction $+$", fontsize=9, color=CPOS, va="center")
    ax.annotate("", xy=(8.0,2.62), xytext=(9.3,2.62), arrowprops=dict(arrowstyle="-|>", color=CNEG, lw=2))
    ax.text(9.35,2.40,"direction $-$", fontsize=9, color=CNEG, va="center", ha="right")
    for x,y,c,f in [(2.7,2.62,CPOS,1),(3.5,2.62,CPOS,1),(4.15,2.62,CPOS,1),(7.3,3.38,CNEG,-1),(8.2,3.38,CNEG,-1)]:
        car(ax,x,y,color=c,facing=f)
    cloud(ax,5.0,5.4)
    for (vx,vy) in [(3.5,2.62),(7.3,3.38),(4.15,2.62)]:
        ax.add_patch(FancyArrowPatch((5.0,4.95),(vx,vy+0.16), arrowstyle="<->", mutation_scale=8,
                     color=GREY, lw=1.0, ls=(0,(3,3)), zorder=3))
    ax.text(6.55,4.78,"V2X: report state,\nreceive slot + target speed", fontsize=8.2, color="0.3", va="center")
    ax.text(0.3,5.95,"(a) Corridor and cloud coordination", fontsize=10.5, fontweight="bold")

def trajectories(ax):
    ax.set_xlim(0,8.6); ax.set_ylim(0,10)
    by0, by1 = 4.4, 5.6
    ax.axhspan(by0,by1, color="0.85", zorder=0)
    ax.text(8.5,(by0+by1)/2,"shared\nsection", ha="right", va="center", fontsize=8.2, color="0.35")
    def traj(te, tx, d, color):
        ys = [1.0,by0,by1,9.0] if d>0 else [9.0,by1,by0,1.0]
        xs = [te-1.1, te, tx, tx+1.1]
        ax.plot(xs,ys, color=color, lw=2.0, solid_capstyle="round", zorder=2)
        ax.plot([te],[ys[1]],"o", color=color, ms=4, zorder=3)
    for (t0,t1,d,n) in WINDOWS:
        col = CPOS if d>0 else CNEG
        cw = min(0.55,(t1-t0)/max(n,1)*0.7)
        for k in range(n):
            te = t0+(k+0.5)*(t1-t0)/n - cw/2
            traj(te, te+cw, d, col)
    ax.set_yticks([]); ax.set_xticks([])
    ax.set_ylabel("position", fontsize=9.5)
    ax.set_title("(b) Reservation schedule", fontsize=10.5, fontweight="bold", loc="left")

def blocks(ax):
    ax.set_xlim(0,8.6); ax.set_ylim(0,1)
    for (t0,t1,d,n) in WINDOWS:
        col = CPOS if d>0 else CNEG
        ax.add_patch(Rectangle((t0,0.32), t1-t0, 0.40, fc=col, ec="black", lw=0.8, alpha=0.85, zorder=2))
        ax.text((t0+t1)/2,0.52,"dir $+$" if d>0 else "dir $-$", ha="center", va="center",
                color="white", fontsize=8.5, fontweight="bold", zorder=3)
        for k in range(n):
            xx = t0+(k+0.5)*(t1-t0)/n
            ax.plot([xx,xx],[0.34,0.70], color="white", lw=1.0, zorder=3)
    for (a,b) in GOPP_GAPS:
        ax.annotate("", xy=(b,0.16), xytext=(a,0.16), arrowprops=dict(arrowstyle="<->", color="0.3", lw=1.0))
    ax.text(GOPP_GAPS[0][0]-0.05,0.04,"$g_{\\mathrm{opp}}$", fontsize=8.5, color="0.3", ha="center")
    ax.annotate("same-direction batch\n(cap $B_{\\max}$)",
                xy=((WINDOWS[0][0]+WINDOWS[0][1])/2,0.74),
                xytext=((WINDOWS[0][0]+WINDOWS[0][1])/2,0.99), ha="center", va="top",
                fontsize=8.2, color="0.25", arrowprops=dict(arrowstyle="-", color="0.5", lw=0.8))
    ax.set_yticks([]); ax.set_xticks([])
    ax.set_xlabel("time $\\rightarrow$ (reserved section occupancy)", fontsize=9.5)
    for s in ("top","right","left"):
        ax.spines[s].set_visible(False)

fig = plt.figure(figsize=(10.2,4.5))
gs = fig.add_gridspec(2,2, width_ratios=[1.42,1.0], height_ratios=[1.15,1.0],
                      wspace=0.13, hspace=0.32, left=0.015, right=0.985, top=0.93, bottom=0.10)
scene(fig.add_subplot(gs[:,0]))
trajectories(fig.add_subplot(gs[0,1]))
blocks(fig.add_subplot(gs[1,1]))
for ext in ("pdf","png"):
    fig.savefig(f"{FIGOUT}/fig_framework.{ext}", dpi=300)
print("wrote fig_framework to", FIGOUT)
