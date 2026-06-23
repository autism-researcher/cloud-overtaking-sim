#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cloud-Assisted Intelligent Traffic Coordination for Obstacle Overtaking
=======================================================================
Microscopic simulator for a one-lane two-way road with a localized obstacle.

A localized obstacle blocks one travel direction over a short stretch (the
"shared section"). Vehicles from BOTH directions therefore contend for the same
piece of pavement at the obstacle: only one direction may occupy the shared
section at a time (same-direction vehicles may follow each other as a batch).

Two coordination policies are compared:
  * BASELINE   : decentralized, one vehicle at a time, right-of-way alternates
                 between directions (a best-case unsignalized bottleneck rule).
  * CLOUD      : centralized slot reservation + IDM-compliant target speed +
                 same-direction batching + fair alternation (the proposed method).

Longitudinal dynamics use the Intelligent Driver Model (IDM).

Outputs (written to ./out):
  fig_spacetime_cloud.pdf / .png   space-time diagram, proposed method
  fig_spacetime_base.pdf  / .png   space-time diagram, baseline
  fig_waiting_cdf.pdf     / .png   empirical CDF of per-vehicle waiting time
  fig_sweep_throughput.pdf/ .png   throughput vs arrival rate
  fig_sweep_waiting.pdf   / .png   mean & P95 waiting vs arrival rate
  results_table.csv                headline metrics at the representative point
  sweep.csv                        full density-sweep results

Run:  python cloud_overtaking_sim.py
Deps: numpy, matplotlib   (pip install numpy matplotlib)

Author: M. B. Hossain, O. Tayan, M. A. S. Kamal
"""

import os
import csv
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
class Cfg:
    # Geometry [m]
    L          = 1000.0      # corridor length (each direction runs 0 -> L)
    L_o        = 30.0        # obstacle / shared-section length
    L_p        = 300.0       # precaution-zone length upstream of the section
    S_entry    = L/2 - L_o/2 # progress at which the shared section begins
    S_exit     = L/2 + L_o/2 # progress at which the shared section ends
    S_prec     = S_entry - L_p   # progress at which the precaution zone begins

    # IDM parameters
    v0         = 25.0        # desired free-flow velocity [m/s]
    T_head     = 1.5         # safe time headway [s]
    s0         = 2.0         # minimum bumper-to-bumper spacing [m]
    a_max      = 1.2         # maximum acceleration [m/s^2]
    b_comf     = 1.8         # comfortable deceleration [m/s^2]
    delta      = 4.0         # acceleration exponent
    v_sec      = 8.0         # speed limit while crossing the obstacle [m/s]

    # Heterogeneity
    len_min    = 4.0
    len_max    = 7.0
    v0_spread  = 3.0         # +/- perturbation on desired velocity [m/s]

    # Cloud coordinator
    g_opp      = 1.5         # safety gap between opposing batches [s]
    g_same     = 1.2         # headway between same-direction vehicles in section [s]
    v_min      = 5.0
    v_max      = 25.0
    max_batch  = 12.0        # max batch duration per direction (fairness cap) [s]

    # Fixed-time signal baseline (portable work-zone signal / flagger)
    sig_green  = 15.0        # green duration per direction [s]
    sig_clear  = 6.0         # all-red clearance between greens [s]

    # Simulation
    dt         = 0.2         # time step [s]
    T_warm     = 40.0        # warm-up (excluded from metrics) [s]
    T_sim      = 360.0       # measured horizon [s]
    v_wait_th  = 2.0         # speed below which a vehicle is "waiting" [m/s]
    seed       = 7


# ----------------------------------------------------------------------------
# Vehicle
# ----------------------------------------------------------------------------
class Veh:
    __slots__ = ("vid","d","length","v0i","s","v","t_in","t_out","wait",
                 "reserved","t_s","D","done","occupying")
    def __init__(self, vid, d, length, v0i, t_in):
        self.vid = vid
        self.d   = d              # +1 (L->R) or -1 (R->L) -- both use progress s
        self.length = length
        self.v0i = v0i
        self.s   = 0.0
        self.v   = v0i
        self.t_in  = t_in
        self.t_out = None
        self.wait  = 0.0
        self.reserved = False     # cloud: slot assigned?
        self.t_s   = None         # cloud: reserved slot start time
        self.D     = None         # estimated crossing duration
        self.done  = False
        self.occupying = False    # currently inside the shared section?


# ----------------------------------------------------------------------------
# IDM acceleration
# ----------------------------------------------------------------------------
def idm_accel(v, v0_eff, gap, dv):
    """IDM acceleration. gap = bumper-to-bumper distance to (virtual) leader,
    dv = v - v_leader. v0_eff = effective desired speed."""
    gap = max(gap, 0.1)
    s_star = (Cfg.s0 + max(0.0, v*Cfg.T_head + v*dv/(2*math.sqrt(Cfg.a_max*Cfg.b_comf))))
    free = 1.0 - (v/max(v0_eff,0.1))**Cfg.delta
    interact = (s_star/gap)**2
    a = Cfg.a_max*(free - interact)
    # limit harsh braking for numerical stability
    return max(a, -2.5*Cfg.b_comf)


# ----------------------------------------------------------------------------
# Cloud slot-reservation manager
# ----------------------------------------------------------------------------
class Cloud:
    """Maintains a chronological list of section-occupancy blocks.
    Each block = dict(dir, start, end, last_entry). Same-direction vehicles are
    appended to the current block as a batch (subject to max_batch); a switch of
    direction opens a new block after an opposing safety gap g_opp."""
    def __init__(self):
        self.blocks = []

    def reserve(self, d, r_i, D_i):
        b = self.blocks[-1] if self.blocks else None
        if (b is not None and b["dir"] == d
                and r_i <= b["end"] + Cfg.g_same
                and (b["end"] - b["start"]) < Cfg.max_batch):
            # join the current same-direction batch
            t_s = max(r_i, b["last_entry"] + Cfg.g_same)
            b["last_entry"] = t_s
            b["end"] = max(b["end"], t_s + D_i)
        else:
            prev_end = b["end"] if b is not None else 0.0
            gap = Cfg.g_opp if (b is not None and b["dir"] != d) else 0.0
            t_s = max(r_i, prev_end + gap)
            self.blocks.append(dict(dir=d, start=t_s, end=t_s + D_i, last_entry=t_s))
        return t_s


# ----------------------------------------------------------------------------
# Simulation core
# ----------------------------------------------------------------------------
def in_section(v):
    return Cfg.S_entry <= v.s <= Cfg.S_exit + v.length

def simulate(lam, policy, record_tracks=False, rng=None):
    """Run one simulation.
    lam    : arrival rate per direction [veh/s]
    policy : 'cloud' or 'base'
    returns dict of metrics (+ optional per-vehicle tracks for plotting)."""
    if rng is None:
        rng = np.random.default_rng(Cfg.seed)
    dt = Cfg.dt
    T_total = Cfg.T_warm + Cfg.T_sim
    n_steps = int(T_total/dt)

    # Poisson arrivals: next headway per direction
    next_arr = {+1: rng.exponential(1.0/lam), -1: rng.exponential(1.0/lam)}
    active = {+1: [], -1: []}
    finished = []
    tracks = {}                     # vid -> (list_t, list_s, dir)
    cloud = Cloud()
    vid_ctr = 0

    # baseline section state
    sec_owner = 0
    sec_count = 0
    last_served = -1                # for alternation
    claim = 0                       # direction that currently owns the bridge
    claim_vid = None                # the vehicle holding the claim

    for k in range(n_steps):
        t = k*dt

        # ---- spawn arrivals (safe insertion behind the queue tail) ----
        for d in (+1, -1):
            while t >= next_arr[d]:
                length = rng.uniform(Cfg.len_min, Cfg.len_max)
                v0i = float(np.clip(Cfg.v0 + rng.uniform(-Cfg.v0_spread, Cfg.v0_spread),
                                    Cfg.v_min, Cfg.v_max))
                rear_veh = min(active[d], key=lambda x: x.s) if active[d] else None
                veh = Veh(vid_ctr, d, length, v0i, t)
                if rear_veh is not None:
                    veh.s = min(0.0, rear_veh.s - rear_veh.length - Cfg.s0)
                    veh.v = min(v0i, max(0.0, rear_veh.v))
                vid_ctr += 1
                active[d].append(veh)
                if record_tracks:
                    tracks[veh.vid] = ([], [], d)
                next_arr[d] += rng.exponential(1.0/lam)

        # ---- recompute section occupancy ----
        occ_dirs = set()
        sec_count = 0
        for d in (+1, -1):
            for v in active[d]:
                v.occupying = in_section(v)
                if v.occupying:
                    occ_dirs.add(d); sec_count += 1
        sec_owner = next(iter(occ_dirs)) if len(occ_dirs) == 1 else (0 if not occ_dirs else 99)

        # ---- per-direction control ----
        for d in (+1, -1):
            lane = sorted(active[d], key=lambda x: x.s, reverse=True)  # front first
            for idx, v in enumerate(lane):
                leader = lane[idx-1] if idx > 0 else None

                # ----- cloud: reserve a slot upon entering precaution zone -----
                if policy == "cloud" and (not v.reserved) and Cfg.S_prec <= v.s < Cfg.S_entry:
                    r_i = t + (Cfg.S_entry - v.s)/max(v.v, Cfg.v_min)
                    v.D = (Cfg.L_o + v.length)/Cfg.v_sec + Cfg.g_same
                    v.t_s = cloud.reserve(d, r_i, v.D)
                    v.reserved = True

                # ----- desired speed -----
                v0_eff = v.v0i
                if policy == "cloud" and v.reserved and v.s < Cfg.S_entry and v.t_s is not None:
                    dist = Cfg.S_entry - v.s
                    rem = v.t_s - t
                    if rem > 0.1:
                        v0_eff = float(np.clip(dist/rem, Cfg.v_min, Cfg.v_max))
                    else:
                        v0_eff = v.v0i
                if Cfg.S_entry - 5 <= v.s <= Cfg.S_exit:
                    v0_eff = min(v0_eff, Cfg.v_sec)

                # ----- gap to the (real) leader -----
                if leader is not None:
                    gap = leader.s - leader.length - v.s
                    dv = v.v - leader.v
                else:
                    gap = 1e6; dv = 0.0

                # ----- permission to enter the shared section -----
                allow_enter = True
                if v.s < Cfg.S_entry:
                    if policy == "base":
                        # One vehicle at a time. A direction CLAIMS the bridge
                        # the moment it is granted (like a one-lane bridge), so
                        # the opposing stream cannot be granted during approach.
                        front = (idx == 0)
                        if claim == 0:
                            opp_waiting = any(
                                (Cfg.S_prec <= vv.s < Cfg.S_entry) for vv in active[-d])
                            my_turn = (last_served != d) or (not opp_waiting)
                            grant = front and my_turn
                            if grant:
                                claim = d; claim_vid = v.vid
                                allow_enter = True
                            else:
                                allow_enter = False
                        else:
                            allow_enter = (claim == d and v.vid == claim_vid)
                    elif policy == "signal":
                        # Fixed-time signal: green alternates between directions
                        # with an all-red clearance so the section drains before
                        # the opposing green. Vehicles flow as a batch during
                        # green but STOP at red (no en-route speed shaping).
                        period = 2.0*(Cfg.sig_green + Cfg.sig_clear)
                        ph = t % period
                        if ph < Cfg.sig_green:
                            green_dir = +1
                        elif (Cfg.sig_green + Cfg.sig_clear) <= ph < (2*Cfg.sig_green + Cfg.sig_clear):
                            green_dir = -1
                        else:
                            green_dir = 0
                        opp_in = (len(occ_dirs) == 1 and (sec_owner == -d))
                        allow_enter = (green_dir == d) and (not opp_in)
                    else:  # cloud
                        opp_in = (len(occ_dirs) == 1 and (sec_owner == -d))
                        allow_enter = (t >= (v.t_s or t)) and (not opp_in)

                # virtual stop-line leader at S_entry if not allowed to enter
                if (not allow_enter) and v.s < Cfg.S_entry:
                    sl_gap = Cfg.S_entry - v.s
                    if sl_gap < gap:
                        gap = sl_gap; dv = v.v

                # ----- integrate IDM -----
                a = idm_accel(v.v, v0_eff, gap, dv)
                v.v = max(0.0, v.v + a*dt)
                s_prev = v.s
                v.s += v.v*dt
                if leader is not None:
                    _rear = leader.s - leader.length
                    if v.s > _rear:
                        v.s = _rear; v.v = min(v.v, leader.v)

                # baseline: release the bridge claim once the holder has cleared
                if policy == "base" and v.vid == claim_vid:
                    if v.s > Cfg.S_exit + v.length:
                        claim = 0; claim_vid = None; last_served = d

                # ----- waiting time (after warm-up only) -----
                if t >= Cfg.T_warm and v.v < Cfg.v_wait_th and v.s < Cfg.S_exit:
                    v.wait += dt

                if record_tracks and (k % 2 == 0):
                    tracks[v.vid][0].append(t); tracks[v.vid][1].append(v.s)

                # ----- completion -----
                if v.s >= Cfg.L and not v.done:
                    v.done = True
                    v.t_out = t

        # ---- retire finished vehicles ----
        for d in (+1, -1):
            keep = []
            for v in active[d]:
                if v.done:
                    finished.append(v)
                else:
                    keep.append(v)
            active[d] = keep

    # ---- metrics (departure-based flow over the measurement window) ----
    T_total = Cfg.T_warm + Cfg.T_sim
    meas = [v for v in finished
            if v.t_out is not None and Cfg.T_warm <= v.t_out <= T_total]
    waits = np.array([v.wait for v in meas]) if meas else np.array([0.0])
    travels = np.array([v.t_out - v.t_in for v in meas]) if meas else np.array([0.0])
    Q = len(meas)/Cfg.T_sim*3600.0
    res = dict(
        lam=lam, policy=policy, n=len(meas),
        Q=Q,
        wait_mean=float(np.mean(waits)),
        wait_p95=float(np.percentile(waits, 95)),
        travel_mean=float(np.mean(travels)),
        waits=waits,
    )
    if record_tracks:
        res["tracks"] = tracks
    return res



# ----------------------------------------------------------------------------
# Three-way baseline comparison  (base vs fixed-time signal vs cloud)
# ----------------------------------------------------------------------------
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

POL   = ["base", "signal", "cloud"]
LAB   = {"base":"Decentralized bridge", "signal":"Fixed-time signal", "cloud":"Cloud-assisted (proposed)"}
COL   = {"base":"#c0392b", "signal":"#e08e0b", "cloud":"#1f5fbf"}
MK    = {"base":"o", "signal":"^", "cloud":"s"}

def run_cell(lam, pol, seeds):
    Q=[];W=[];P=[];T=[]
    for s in seeds:
        r=simulate(lam, pol, rng=np.random.default_rng(s))
        Q.append(r["Q"]);W.append(r["wait_mean"]);P.append(r["wait_p95"]);T.append(r["travel_mean"])
    return dict(Q=float(np.mean(Q)),Qsd=float(np.std(Q)),
                W=float(np.mean(W)),P95=float(np.mean(P)),Tt=float(np.mean(T)))

def main():
    os.makedirs("out_baseline", exist_ok=True)
    lam_rep = 0.30
    rep_seeds   = [1,2,3,4,5]
    sweep_seeds = [1,2]
    lams = [0.05,0.10,0.15,0.20,0.25,0.30,0.40,0.50]

    # ---- representative-point table (averaged over seeds) ----
    rep = {p: run_cell(lam_rep, p, rep_seeds) for p in POL}
    with open("out_baseline/results_table_3way.csv","w",newline="") as f:
        w=csv.writer(f); w.writerow(["Metric"]+[LAB[p] for p in POL])
        w.writerow(["Throughput Q [veh/h]"]      +[f"{rep[p]['Q']:.0f}"  for p in POL])
        w.writerow(["Average waiting Wbar [s]"]  +[f"{rep[p]['W']:.1f}"  for p in POL])
        w.writerow(["P95 waiting W95 [s]"]       +[f"{rep[p]['P95']:.1f}"for p in POL])
        w.writerow(["Average travel time Tt [s]"]+[f"{rep[p]['Tt']:.1f}" for p in POL])

    # ---- density sweep ----
    sweep={p:[] for p in POL}
    for lam in lams:
        for p in POL:
            sweep[p].append(run_cell(lam,p,sweep_seeds))
    with open("out_baseline/sweep_3way.csv","w",newline="") as f:
        w=csv.writer(f); w.writerow(["lambda","policy","Q_vehph","wait_mean_s","wait_p95_s","travel_mean_s"])
        for p in POL:
            for lam,c in zip(lams,sweep[p]):
                w.writerow([lam,p,f"{c['Q']:.1f}",f"{c['W']:.2f}",f"{c['P95']:.2f}",f"{c['Tt']:.2f}"])

    # ---- figure: throughput vs demand ----
    fig,ax=plt.subplots(figsize=(5.4,3.7))
    for p in POL:
        ax.plot(lams,[c["Q"] for c in sweep[p]],MK[p]+"-",color=COL[p],label=LAB[p],lw=1.8,ms=5)
    ax.set_xlabel(r"arrival rate $\lambda$ [veh/s/dir]");ax.set_ylabel("throughput Q [veh/h]")
    ax.legend(fontsize=8);ax.grid(alpha=0.3);fig.tight_layout()
    for e in ("pdf","png"): fig.savefig(f"out_baseline/fig_throughput_3way.{e}",dpi=160)
    plt.close(fig)

    # ---- figure: mean waiting vs demand ----
    fig,ax=plt.subplots(figsize=(5.4,3.7))
    for p in POL:
        ax.plot(lams,[c["W"] for c in sweep[p]],MK[p]+"-",color=COL[p],label=LAB[p],lw=1.8,ms=5)
    ax.set_xlabel(r"arrival rate $\lambda$ [veh/s/dir]");ax.set_ylabel("average waiting time [s]")
    ax.legend(fontsize=8);ax.grid(alpha=0.3);fig.tight_layout()
    for e in ("pdf","png"): fig.savefig(f"out_baseline/fig_waiting_3way.{e}",dpi=160)
    plt.close(fig)

    print(f"{'Metric':26s}"+"".join(f"{LAB[p]:>26s}" for p in POL))
    print(f"{'Throughput Q [veh/h]':26s}"+"".join(f"{rep[p]['Q']:>26.0f}" for p in POL))
    print(f"{'Avg waiting Wbar [s]':26s}"+"".join(f"{rep[p]['W']:>26.1f}" for p in POL))
    print(f"{'P95 waiting W95 [s]':26s}"+"".join(f"{rep[p]['P95']:>26.1f}" for p in POL))
    print(f"{'Avg travel time Tt [s]':26s}"+"".join(f"{rep[p]['Tt']:>26.1f}" for p in POL))
    print("\nWrote out_baseline/  (table + sweep csv + 2 figures)")

if __name__ == "__main__":
    main()
