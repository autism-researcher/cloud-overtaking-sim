#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Conceptual schematic figures for the Introduction and key argument."""
import os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, FancyArrowPatch, Ellipse, Circle

FIGOUT = os.environ.get("FIGOUT", ".")
CPOS="#1f5fbf"; CNEG="#e0820a"; COBS="#c0392b"; GREEN="#2e8b57"; GREY="0.45"

def car(ax,x,y,w=0.5,h=0.26,color=CPOS):
    ax.add_patch(FancyBboxPatch((x-w/2,y-h/2),w,h,boxstyle="round,pad=0.01,rounding_size=0.07",
                 fc=color,ec="black",lw=0.8,zorder=5))

def road(ax,x0,x1,yc,h=0.8):
    ax.add_patch(Rectangle((x0,yc-h/2),x1-x0,h,fc="0.9",ec="0.55",lw=1.0,zorder=1))
    ax.plot([x0,x1],[yc,yc],ls=(0,(5,5)),color="white",lw=1.3,zorder=2)

def obstacle(ax,x,yc,h=0.8):
    ax.add_patch(Rectangle((x-0.22,yc),0.44,h/2-0.03,fc=COBS,ec="black",lw=0.7,zorder=6))

# ---------------------------------------------------------------- before / after
def before_after():
    fig,axes=plt.subplots(2,1,figsize=(6.6,4.4))
    for ax in axes: ax.set_xlim(0,12); ax.set_ylim(0,2.6); ax.axis("off")
    a,b=axes
    # (a) without coordination
    road(a,0.4,11.6,1.3)
    obstacle(a,6.0,1.3)
    a.text(6.0,2.25,"obstacle",ha="center",color=COBS,fontsize=9,fontweight="bold")
    car(a,4.7,1.05,color=CPOS); car(a,5.4,1.05,color=CPOS)
    car(a,7.3,1.55,color=CNEG); car(a,8.0,1.55,color=CNEG)
    a.annotate("",xy=(5.7,1.05),xytext=(4.2,1.05),arrowprops=dict(arrowstyle="-|>",color=CPOS,lw=1.6))
    a.annotate("",xy=(6.9,1.55),xytext=(8.4,1.55),arrowprops=dict(arrowstyle="-|>",color=CNEG,lw=1.6))
    # conflict burst
    a.plot(6.05,1.3,marker=(10,1,0),ms=24,color="#ffcc00",mec=COBS,mew=1.2,zorder=4)
    a.text(6.05,1.3,"!",ha="center",va="center",fontsize=12,fontweight="bold",color=COBS,zorder=7)
    a.text(0.5,2.25,"(a) Without coordination",fontsize=11,fontweight="bold")
    a.text(9.6,1.05,"STOP · WAIT · CONFLICT",fontsize=9,color=COBS,va="center",ha="left",fontweight="bold")
    # (b) with coordinator
    road(b,0.4,11.6,1.3)
    obstacle(b,6.0,1.3)
    for i,x in enumerate([3.0,3.9,4.8]):
        car(b,x,1.05,color=CPOS)
    car(b,9.2,1.55,color=CNEG)
    b.annotate("",xy=(6.6,1.05),xytext=(5.2,1.05),arrowprops=dict(arrowstyle="-|>",color=CPOS,lw=1.8))
    b.text(8.4,1.55,"reserved next",fontsize=8,color=CNEG,va="center",ha="left")
    # mini cloud
    for dx,dy,r in [(-0.3,0,0.20),(0,0.08,0.24),(0.3,0,0.20)]:
        b.add_patch(Ellipse((6.0+dx,2.25+dy),r*2,r*1.6,fc="white",ec=GREY,lw=1.1,zorder=4))
    b.text(6.0,2.22,"cloud",ha="center",va="center",fontsize=7.5,zorder=6)
    b.add_patch(FancyArrowPatch((6.0,2.0),(4.8,1.2),arrowstyle="-|>",mutation_scale=8,color=GREY,lw=0.9,ls=(0,(3,3)),zorder=3))
    b.text(0.5,2.25,"(b) With cloud coordinator",fontsize=11,fontweight="bold")
    b.text(9.6,1.02,"RESERVED SLOTS · NO CONFLICT · SMOOTH FLOW",fontsize=8.2,color=GREEN,va="center",ha="left",fontweight="bold")
    fig.tight_layout(h_pad=1.2)
    for e in ("pdf","png"): fig.savefig(f"{FIGOUT}/fig_beforeafter.{e}",dpi=300)
    plt.close(fig)

# ---------------------------------------------------------------- analogy
def analogy():
    fig,axes=plt.subplots(1,2,figsize=(7.4,3.3))
    for ax in axes: ax.set_xlim(0,10); ax.set_ylim(0,10); ax.axis("off")
    L,R=axes
    def slots(ax,labels,color,title,icon):
        ax.text(5,9.4,title,ha="center",fontsize=10.5,fontweight="bold")
        for i,lab in enumerate(labels):
            y=7.3-i*1.5
            ax.add_patch(FancyBboxPatch((1.2,y-0.5),3.0,1.0,boxstyle="round,pad=0.02,rounding_size=0.1",
                         fc=color,ec="black",lw=0.7,alpha=0.85))
            ax.text(2.7,y,lab,ha="center",va="center",color="white",fontsize=9,fontweight="bold")
            ax.annotate("",xy=(7.0,y),xytext=(4.5,y),arrowprops=dict(arrowstyle="-|>",color="0.4",lw=1.4))
            ax.add_patch(Rectangle((7.2,y-0.42),2.2,0.84,fc="0.92",ec="0.5",lw=0.7))
            ax.text(8.3,y,f"slot {i+1}",ha="center",va="center",fontsize=8.5)
        ax.text(5,0.4,icon,ha="center",fontsize=8.5,color="0.4",style="italic")
    slots(L,["Plane 1","Plane 2","Plane 3"],"#5b6770","Air-traffic control","controller assigns runway slots")
    slots(R,["Car 1","Car 2","Car 3"],CPOS,"Cloud obstacle coordination","cloud assigns obstacle slots")
    fig.text(0.5,0.5,"≈",ha="center",va="center",fontsize=26,color="0.3")
    fig.tight_layout(w_pad=3.5)
    for e in ("pdf","png"): fig.savefig(f"{FIGOUT}/fig_analogy.{e}",dpi=300)
    plt.close(fig)

# ---------------------------------------------------------------- asymmetric concept
def asym_concept():
    fig=plt.figure(figsize=(6.6,3.9))
    gs=fig.add_gridspec(3,1,height_ratios=[1.0,1.0,1.0],hspace=0.55,left=0.16,right=0.97,top=0.9,bottom=0.08)
    # demand row
    ax0=fig.add_subplot(gs[0]); ax0.set_xlim(0,10); ax0.set_ylim(0,2); ax0.axis("off")
    ax0.text(0.0,1.0,"Demand",fontsize=10,fontweight="bold",va="center")
    ax0.annotate("",xy=(7.5,1.35),xytext=(2.5,1.35),arrowprops=dict(arrowstyle="-|>",color=CPOS,lw=5))
    ax0.text(8.7,1.35,"heavy (95%)",color=CPOS,fontsize=9,va="center")
    ax0.annotate("",xy=(2.5,0.6),xytext=(3.6,0.6),arrowprops=dict(arrowstyle="-|>",color=CNEG,lw=1.6))
    ax0.text(8.7,0.6,"light (5%)",color=CNEG,fontsize=9,va="center")
    def strip(ax,title,blocks,note,ncolor):
        ax.set_xlim(0,10); ax.set_ylim(0,1); ax.axis("off")
        ax.text(0.0,0.5,title,fontsize=9.5,fontweight="bold",va="center",ha="left")
        x=2.2
        for (w,d,empty) in blocks:
            col=CPOS if d>0 else CNEG
            ax.add_patch(Rectangle((x,0.25),w,0.5,fc=col,ec="black",lw=0.7,alpha=0.30 if empty else 0.9))
            ax.text(x+w/2,0.5,("A" if d>0 else "B"),ha="center",va="center",
                    color=(col if empty else "white"),fontsize=8.5,fontweight="bold")
            x+=w+0.08
        ax.text(x+0.15,0.5,note,fontsize=8.2,color=ncolor,va="center",ha="left",fontweight="bold")
    ax1=fig.add_subplot(gs[1])
    strip(ax1,"Fixed-time\nsignal",[(1.0,1,False),(1.0,-1,True),(1.0,1,False),(1.0,-1,True)],"wasted green",COBS)
    ax2=fig.add_subplot(gs[2])
    strip(ax2,"Cloud\nscheduler",[(2.6,1,False),(0.5,-1,False),(2.4,1,False),(0.5,-1,False)],"adaptive",GREEN)
    fig.suptitle("Why a fixed-time signal wastes capacity under asymmetric demand",fontsize=10,fontweight="bold",y=0.99)
    for e in ("pdf","png"): fig.savefig(f"{FIGOUT}/fig_asym_concept.{e}",dpi=300)
    plt.close(fig)

before_after(); analogy(); asym_concept()
print("wrote fig_beforeafter, fig_analogy, fig_asym_concept to",FIGOUT)
