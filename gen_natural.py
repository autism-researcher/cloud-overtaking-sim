#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Re-render Fig.1 (framework) and the before/after figure with realistic
top-view cars and road markings."""
import os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, FancyArrowPatch, Ellipse
FIGOUT=os.environ.get("FIGOUT",".")
CPOS="#1f5fbf"; CNEG="#e0820a"; COBS="#b8362c"; GREEN="#2e8b57"; GREY="0.5"
ASPH="#9a9fa6"; YEL="#f4c20d"

def shade(hexc,f):
    h=hexc.lstrip('#'); r,g,b=int(h[0:2],16),int(h[2:4],16),int(h[4:6],16)
    return (min(1,r/255*f),min(1,g/255*f),min(1,b/255*f))

def draw_car(ax,x,y,color,facing=1,L=0.62,W=0.34,z=5):
    wl,ww=L*0.17,W*0.22
    for sx in (-0.31,0.31):
        for sy in (-1,1):
            ax.add_patch(Rectangle((x+sx*L-wl/2, y+sy*(W/2)-ww/2), wl, ww, fc="#1b1b1b", ec="none", zorder=z))
    ax.add_patch(FancyBboxPatch((x-L/2,y-W/2),L,W,boxstyle="round,pad=0,rounding_size=0.09",
                 fc=color,ec="black",lw=0.9,zorder=z+1))
    rL,rW=L*0.50,W*0.70
    ax.add_patch(FancyBboxPatch((x-rL/2,y-rW/2),rL,rW,boxstyle="round,pad=0,rounding_size=0.05",
                 fc=shade(color,0.72),ec="black",lw=0.5,zorder=z+2))
    ws=rL*0.24
    ax.add_patch(Rectangle((x+facing*(rL/2-ws),y-rW*0.40),ws,rW*0.80,fc="#d8e8f6",ec="none",zorder=z+3))
    ax.add_patch(Rectangle((x-facing*(rL/2-ws*0.3),y-rW*0.40),ws*0.6,rW*0.80,fc="#b9d0e4",ec="none",zorder=z+3))

def draw_road(ax,x0,x1,yc,h,two_way=True):
    ax.add_patch(Rectangle((x0,yc-h/2-0.05*h),x1-x0,h+0.10*h,fc="#cdd1d6",ec="none",zorder=0))  # shoulder
    ax.add_patch(Rectangle((x0,yc-h/2),x1-x0,h,fc=ASPH,ec="none",zorder=1))                      # asphalt
    ax.plot([x0,x1],[yc-h/2+0.03*h]*2,color="white",lw=1.3,zorder=2)
    ax.plot([x0,x1],[yc+h/2-0.03*h]*2,color="white",lw=1.3,zorder=2)
    if two_way:
        ax.plot([x0,x1],[yc,yc],ls=(0,(7,7)),color=YEL,lw=1.7,zorder=2)

def soft_cloud(ax,cx,cy,s=1.0,modules=False):
    for dx,dy,r in [(-0.62,0,0.40),(-0.20,0.16,0.48),(0.26,0.09,0.44),(0.64,-0.02,0.36),(0.0,-0.18,0.48)]:
        ax.add_patch(Ellipse((cx+dx*s,cy+dy*s),r*2*s,r*1.7*s,fc="#f6f8fb",ec=GREY,lw=1.3,zorder=4))
    ax.add_patch(Rectangle((cx-0.82*s,cy-0.34*s),1.64*s,0.38*s,fc="#f6f8fb",ec="none",zorder=4))
    if modules:
        ax.text(cx,cy+0.13*s,"Cloud coordinator",ha="center",va="center",fontsize=9.3,fontweight="bold",zorder=7)
        ax.text(cx,cy-0.27*s,"reservation scheduler $\\cdot$ fairness\ncontroller $\\cdot$ slot assignment",
                ha="center",va="center",fontsize=6.2,color="0.32",zorder=7)
    else:
        ax.text(cx,cy,"cloud",ha="center",va="center",fontsize=7.6,zorder=7)

# ===================================================================== framework
WINDOWS=[(1.0,3.3,+1,3),(3.8,5.2,-1,1),(5.7,7.5,+1,2)]; GOPP_GAPS=[(3.3,3.8),(5.2,5.7)]
def scene(ax):
    ax.set_xlim(0,10); ax.set_ylim(0,6.2); ax.axis("off")
    draw_road(ax,0.3,9.7,3.0,1.5)
    for (a,b,c) in [(2.4,4.1,CPOS),(6.0,7.7,CNEG)]:
        ax.add_patch(Rectangle((a,2.25),b-a,1.5,fc=c,ec="none",alpha=0.16,zorder=2))
    ax.text(3.25,4.02,"precaution zone $L_D$",ha="center",fontsize=9,color=CPOS)
    ax.text(6.85,1.95,"precaution zone $L_D$",ha="center",fontsize=9,color=CNEG)
    sx0,sx1=4.55,5.95
    ax.add_patch(Rectangle((sx0,2.25),sx1-sx0,1.5,fc="none",ec="0.25",lw=1.4,ls="--",zorder=3))
    ax.annotate("",xy=(sx1,1.95),xytext=(sx0,1.95),arrowprops=dict(arrowstyle="<->",color="0.3",lw=1.2))
    ax.text((sx0+sx1)/2,1.62,"shared section $L_o$",ha="center",fontsize=9)
    # obstacle: hazard barrier in top lane
    ax.add_patch(Rectangle((sx0+0.32,3.05),0.66,0.58,fc=COBS,ec="black",lw=0.8,zorder=6))
    for k in range(3):
        ax.add_patch(Rectangle((sx0+0.32,3.05+k*0.19),0.66,0.095,fc="white",ec="none",alpha=0.55,zorder=7))
    ax.text(sx0+0.65,3.92,"obstacle",ha="center",fontsize=9,color=COBS,fontweight="bold")
    ax.annotate("",xy=(1.9,3.40),xytext=(0.7,3.40),arrowprops=dict(arrowstyle="-|>",color=CPOS,lw=2))
    ax.text(0.55,3.66,"direction $+$",fontsize=9,color=CPOS,va="center")
    ax.annotate("",xy=(8.1,2.60),xytext=(9.3,2.60),arrowprops=dict(arrowstyle="-|>",color=CNEG,lw=2))
    ax.text(9.35,2.36,"direction $-$",fontsize=9,color=CNEG,va="center",ha="right")
    for x in (2.7,3.5,4.2): draw_car(ax,x,2.62,CPOS,facing=+1,L=0.62,W=0.34)
    for x in (7.3,8.2): draw_car(ax,x,3.38,CNEG,facing=-1,L=0.62,W=0.34)
    soft_cloud(ax,5.0,5.45,s=1.0,modules=True)
    for (vx,vy) in [(3.5,2.62),(7.3,3.38),(4.2,2.62)]:
        ax.add_patch(FancyArrowPatch((5.0,4.92),(vx,vy+0.2),arrowstyle="<->",mutation_scale=8,color=GREY,lw=1.0,ls=(0,(3,3)),zorder=3))
    ax.text(6.6,4.75,"V2X: report state,\nreceive slot + target speed",fontsize=8.2,color="0.3",va="center")
    ax.text(0.3,5.98,"(a) Corridor and cloud coordination",fontsize=10.5,fontweight="bold")
def trajectories(ax):
    ax.set_xlim(0,8.6); ax.set_ylim(0,10); by0,by1=4.4,5.6
    ax.axhspan(by0,by1,color="0.85",zorder=0)
    ax.text(8.5,(by0+by1)/2,"shared\nsection",ha="right",va="center",fontsize=8.2,color="0.35")
    def traj(te,tx,d,color):
        ys=[1.0,by0,by1,9.0] if d>0 else [9.0,by1,by0,1.0]; xs=[te-1.1,te,tx,tx+1.1]
        ax.plot(xs,ys,color=color,lw=2.0,solid_capstyle="round",zorder=2); ax.plot([te],[ys[1]],"o",color=color,ms=4,zorder=3)
    for (t0,t1,d,n) in WINDOWS:
        col=CPOS if d>0 else CNEG; cw=min(0.55,(t1-t0)/max(n,1)*0.7)
        for k in range(n):
            te=t0+(k+0.5)*(t1-t0)/n-cw/2; traj(te,te+cw,d,col)
    ax.set_yticks([]); ax.set_xticks([]); ax.set_ylabel("position",fontsize=9.5)
    ax.set_title("(b) Reservation schedule",fontsize=10.5,fontweight="bold",loc="left")
def blocks(ax):
    ax.set_xlim(0,8.6); ax.set_ylim(0,1)
    for (t0,t1,d,n) in WINDOWS:
        col=CPOS if d>0 else CNEG
        ax.add_patch(Rectangle((t0,0.32),t1-t0,0.40,fc=col,ec="black",lw=0.8,alpha=0.9,zorder=2))
        ax.text((t0+t1)/2,0.52,"dir $+$" if d>0 else "dir $-$",ha="center",va="center",color="white",fontsize=8.5,fontweight="bold",zorder=3)
        for k in range(n):
            xx=t0+(k+0.5)*(t1-t0)/n; ax.plot([xx,xx],[0.34,0.70],color="white",lw=1.0,zorder=3)
    for (a,b) in GOPP_GAPS: ax.annotate("",xy=(b,0.16),xytext=(a,0.16),arrowprops=dict(arrowstyle="<->",color="0.3",lw=1.0))
    ax.text(GOPP_GAPS[0][0]-0.05,0.04,"$g_{\\mathrm{opp}}$",fontsize=8.5,color="0.3",ha="center")
    ax.annotate("same-direction batch\n(cap $B_{\\max}$)",xy=((WINDOWS[0][0]+WINDOWS[0][1])/2,0.74),
                xytext=((WINDOWS[0][0]+WINDOWS[0][1])/2,0.99),ha="center",va="top",fontsize=8.2,color="0.25",
                arrowprops=dict(arrowstyle="-",color="0.5",lw=0.8))
    ax.set_yticks([]); ax.set_xticks([]); ax.set_xlabel("time $\\rightarrow$ (reserved section occupancy)",fontsize=9.5)
    for s in ("top","right","left"): ax.spines[s].set_visible(False)
fig=plt.figure(figsize=(10.2,4.5))
gs=fig.add_gridspec(2,2,width_ratios=[1.42,1.0],height_ratios=[1.15,1.0],wspace=0.13,hspace=0.32,left=0.015,right=0.985,top=0.93,bottom=0.10)
scene(fig.add_subplot(gs[:,0])); trajectories(fig.add_subplot(gs[0,1])); blocks(fig.add_subplot(gs[1,1]))
for e in ("pdf","png"): fig.savefig(f"{FIGOUT}/fig_framework.{e}",dpi=300)
plt.close(fig)

# ===================================================================== before/after
fig,axes=plt.subplots(2,1,figsize=(6.8,4.8))
for ax in axes: ax.set_xlim(0,12); ax.set_ylim(0,3.0); ax.axis("off")
a,b=axes
draw_road(a,0.4,11.6,1.55,0.9); 
ax=a
ax.add_patch(Rectangle((5.78,1.55),0.44,0.40,fc=COBS,ec="black",lw=0.8,zorder=6))
for k in range(2): ax.add_patch(Rectangle((5.78,1.55+k*0.16),0.44,0.07,fc="white",ec="none",alpha=0.55,zorder=7))
a.text(6.0,2.16,"obstacle",ha="center",color=COBS,fontsize=8.5,fontweight="bold")
a.text(0.4,2.80,"(a) Without coordination",fontsize=11,fontweight="bold")
draw_car(a,4.45,1.32,CPOS,facing=+1,L=0.66,W=0.28); draw_car(a,5.34,1.32,CPOS,facing=+1,L=0.66,W=0.28)
draw_car(a,7.24,1.78,CNEG,facing=-1,L=0.66,W=0.28); draw_car(a,8.13,1.78,CNEG,facing=-1,L=0.66,W=0.28)
a.annotate("",xy=(5.55,1.32),xytext=(3.95,1.32),arrowprops=dict(arrowstyle="-|>",color=CPOS,lw=1.5))
a.annotate("",xy=(6.95,1.78),xytext=(8.55,1.78),arrowprops=dict(arrowstyle="-|>",color=CNEG,lw=1.5))
a.plot(6.0,1.55,marker=(10,1,0),ms=22,color="#ffcc00",mec=COBS,mew=1.2,zorder=4)
a.text(6.0,1.55,"!",ha="center",va="center",fontsize=11,fontweight="bold",color=COBS,zorder=8)
a.text(6.0,0.42,"STOP    ·    WAIT    ·    CONFLICT",ha="center",color=COBS,fontsize=10.5,fontweight="bold")
draw_road(b,0.4,11.6,1.55,0.9)
b.add_patch(Rectangle((5.78,1.55),0.44,0.40,fc=COBS,ec="black",lw=0.8,zorder=6))
for k in range(2): b.add_patch(Rectangle((5.78,1.55+k*0.16),0.44,0.07,fc="white",ec="none",alpha=0.55,zorder=7))
b.text(0.4,2.80,"(b) With cloud coordinator",fontsize=11,fontweight="bold")
for x in (2.6,3.5,4.4): draw_car(b,x,1.32,CPOS,facing=+1,L=0.66,W=0.28)
draw_car(b,10.3,1.78,CNEG,facing=-1,L=0.66,W=0.28)
b.annotate("",xy=(6.45,1.32),xytext=(5.05,1.32),arrowprops=dict(arrowstyle="-|>",color=CPOS,lw=1.8))
b.text(10.3,2.18,"waiting\n(reserved next)",ha="center",va="center",fontsize=7.3,color=CNEG)
soft_cloud(b,6.7,2.58,s=0.62)
b.add_patch(FancyArrowPatch((6.4,2.34),(4.4,1.5),arrowstyle="-|>",mutation_scale=7,color=GREY,lw=0.9,ls=(0,(3,3)),zorder=3))
b.add_patch(FancyArrowPatch((7.0,2.34),(10.3,1.96),arrowstyle="-|>",mutation_scale=7,color=GREY,lw=0.9,ls=(0,(3,3)),zorder=3))
b.text(6.0,0.42,"RESERVED SLOTS    ·    NO CONFLICT    ·    SMOOTH FLOW",ha="center",color=GREEN,fontsize=9,fontweight="bold")
fig.tight_layout(h_pad=1.4)
for e in ("pdf","png"): fig.savefig(f"{FIGOUT}/fig_beforeafter.{e}",dpi=300)
plt.close(fig)
print("wrote natural fig_framework and fig_beforeafter to",FIGOUT)
