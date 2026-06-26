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
                 "reserved","t_s","D","done","occupying","stops","stopped",
                 "Th","am","bc","vsec")
    def __init__(self, vid, d, length, v0i, t_in, Th=None, am=None, bc=None, vsec=None):
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
        self.stops = 0            # number of full stops on the approach
        self.stopped = False      # hysteresis flag for stop counting
        self.Th = Th if Th is not None else Cfg.T_head
        self.am = am if am is not None else Cfg.a_max
        self.bc = bc if bc is not None else Cfg.b_comf
        self.vsec = vsec if vsec is not None else Cfg.v_sec


# ----------------------------------------------------------------------------
# IDM acceleration
# ----------------------------------------------------------------------------
def idm_accel(v, v0_eff, gap, dv, Th=None, am=None, bc=None):
    """IDM acceleration with optional per-vehicle parameters (default to Cfg)."""
    Th = Th if Th is not None else Cfg.T_head
    am = am if am is not None else Cfg.a_max
    bc = bc if bc is not None else Cfg.b_comf
    gap = max(gap, 0.1)
    s_star = (Cfg.s0 + max(0.0, v*Th + v*dv/(2*math.sqrt(am*bc))))
    free = 1.0 - (v/max(v0_eff,0.1))**Cfg.delta
    interact = (s_star/gap)**2
    a = am*(free - interact)
    return max(a, -2.5*bc)


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

    # Poisson arrivals: next headway per direction.
    # `lam` may be a scalar (symmetric) or a dict {+1: ..., -1: ...} (asymmetric).
    lam_d = lam if isinstance(lam, dict) else {+1: lam, -1: lam}
    next_arr = {+1: rng.exponential(1.0/lam_d[+1]), -1: rng.exponential(1.0/lam_d[-1])}
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

        # ---- spawn arrivals (safe insertion behind the queue tail) ---------
        # A new vehicle is placed at the origin, or, once the queue has reached
        # the origin, a minimum gap s0 behind the rear-most vehicle (the approach
        # extends upstream as a physical feeder queue) and at a speed no greater
        # than that vehicle's. This prevents the non-physical injection of a
        # vehicle onto already-occupied pavement under heavy demand.
        for d in (+1, -1):
            while t >= next_arr[d]:
                Th=am=bc=vsec=None
                if getattr(Cfg, "het_drivers", False):
                    Th = rng.uniform(1.0, 2.2)
                    if rng.random() < getattr(Cfg, "truck_frac", 0.0):
                        length = rng.uniform(10.0, 16.0); am = rng.uniform(0.5, 0.8)
                        bc = rng.uniform(1.2, 1.6); vsec = Cfg.v_sec*0.8
                        v0i = float(np.clip(Cfg.v0*0.8 + rng.uniform(-2,2), Cfg.v_min, Cfg.v_max))
                    else:
                        length = rng.uniform(Cfg.len_min, Cfg.len_max); am = rng.uniform(0.9, 1.6)
                        bc = rng.uniform(1.4, 2.4); vsec = Cfg.v_sec
                        v0i = float(np.clip(Cfg.v0 + rng.uniform(-Cfg.v0_spread, Cfg.v0_spread), Cfg.v_min, Cfg.v_max))
                else:
                    length = rng.uniform(Cfg.len_min, Cfg.len_max)
                    v0i = float(np.clip(Cfg.v0 + rng.uniform(-Cfg.v0_spread, Cfg.v0_spread),
                                        Cfg.v_min, Cfg.v_max))
                rear_veh = min(active[d], key=lambda x: x.s) if active[d] else None
                veh = Veh(vid_ctr, d, length, v0i, t, Th=Th, am=am, bc=bc, vsec=vsec)
                if rear_veh is not None:
                    veh.s = min(0.0, rear_veh.s - rear_veh.length - Cfg.s0)
                    veh.v = min(v0i, max(0.0, rear_veh.v))
                vid_ctr += 1
                active[d].append(veh)
                if record_tracks:
                    tracks[veh.vid] = ([], [], d)
                next_arr[d] += rng.exponential(1.0/lam_d[d])

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
                    v.D = (Cfg.L_o + v.length)/v.vsec + Cfg.g_same
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
                a = idm_accel(v.v, v0_eff, gap, dv, v.Th, v.am, v.bc)
                v.v = max(0.0, v.v + a*dt)
                s_prev = v.s
                v.s += v.v*dt
                if leader is not None:                 # hard no-overlap guard
                    _rear = leader.s - leader.length
                    if v.s > _rear:
                        v.s = _rear; v.v = min(v.v, leader.v)

                # count full stops on the approach (with hysteresis)
                if v.s < Cfg.S_exit:
                    if v.v < 0.3 and not v.stopped:
                        v.stops += 1; v.stopped = True
                    elif v.v > 1.0:
                        v.stopped = False

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
    stops = np.array([v.stops for v in meas]) if meas else np.array([0.0])
    Q = len(meas)/Cfg.T_sim*3600.0
    # per-direction metrics (for fairness analysis)
    meas_pos = [v for v in meas if v.d == +1]
    meas_neg = [v for v in meas if v.d == -1]
    wpos = float(np.mean([v.wait for v in meas_pos])) if meas_pos else 0.0
    wneg = float(np.mean([v.wait for v in meas_neg])) if meas_neg else 0.0
    Qpos = len(meas_pos)/Cfg.T_sim*3600.0
    Qneg = len(meas_neg)/Cfg.T_sim*3600.0
    res = dict(
        wait_pos=wpos, wait_neg=wneg, Q_pos=Qpos, Q_neg=Qneg,
        n_pos=len(meas_pos), n_neg=len(meas_neg),
        lam=lam, policy=policy, n=len(meas),
        Q=Q,
        wait_mean=float(np.mean(waits)),
        wait_p95=float(np.percentile(waits, 95)),
        travel_mean=float(np.mean(travels)),
        stops_mean=float(np.mean(stops)),
        waits=waits,
    )
    if record_tracks:
        res["tracks"] = tracks
    return res


# ----------------------------------------------------------------------------
# Plotting helpers
# ----------------------------------------------------------------------------
def _spacetime(tracks, title, fname, t0=Cfg.T_warm, t1=Cfg.T_warm+180):
    fig, ax = plt.subplots(figsize=(5.2, 3.6))
    for vid,(ts,ss,d) in tracks.items():
        if not ts: continue
        ts = np.array(ts); ss = np.array(ss)
        m = (ts >= t0) & (ts <= t1)
        if m.sum() < 2: continue
        c = "#1f5fbf" if d == +1 else "#c0392b"
        ax.plot(ts[m], ss[m], color=c, lw=0.7, alpha=0.85)
    ax.axhspan(Cfg.S_entry, Cfg.S_exit, color="0.75", alpha=0.6, zorder=0)
    ax.axhline(Cfg.S_entry, color="k", ls="--", lw=0.6)
    ax.set_xlim(t0, t1); ax.set_ylim(0, Cfg.L)
    ax.set_xlabel("time [s]"); ax.set_ylabel("position along corridor [m]")
    ax.set_title(title, fontsize=10)
    from matplotlib.lines import Line2D
    ax.legend([Line2D([0],[0],color="#1f5fbf"),Line2D([0],[0],color="#c0392b")],
              ["direction L->R","direction R->L"], fontsize=8, loc="lower right")
    fig.tight_layout()
    for ext in ("pdf","png"):
        fig.savefig(f"out/{fname}.{ext}", dpi=160)
    plt.close(fig)


def main():
    os.makedirs("out", exist_ok=True)
    lam_rep = 0.30   # representative arrival rate [veh/s/direction]

    print("Running representative scenario (lambda = %.2f veh/s/dir)..." % lam_rep)
    r_cloud = simulate(lam_rep, "cloud", record_tracks=True,
                       rng=np.random.default_rng(Cfg.seed))
    r_base  = simulate(lam_rep, "base",  record_tracks=True,
                       rng=np.random.default_rng(Cfg.seed))

    _spacetime(r_cloud["tracks"], "Proposed cloud-assisted coordination",
               "fig_spacetime_cloud")
    _spacetime(r_base["tracks"],  "Decentralized baseline",
               "fig_spacetime_base")

    # waiting-time CDF
    fig, ax = plt.subplots(figsize=(5.2, 3.6))
    for r,lab,c in ((r_base,"Decentralized","#c0392b"),
                    (r_cloud,"Cloud-assisted","#1f5fbf")):
        w = np.sort(r["waits"]); y = np.arange(1,len(w)+1)/len(w)
        ax.plot(w, y, color=c, lw=1.8, label=lab)
    ax.set_xlabel("per-vehicle waiting time [s]"); ax.set_ylabel("empirical CDF")
    ax.set_ylim(0,1.02); ax.legend(fontsize=9); ax.grid(alpha=0.3)
    fig.tight_layout()
    for ext in ("pdf","png"): fig.savefig(f"out/fig_waiting_cdf.{ext}", dpi=160)
    plt.close(fig)

    # density sweep
    lams = [0.05,0.10,0.15,0.20,0.25,0.30,0.35,0.40,0.45,0.50]
    sweep = {"cloud":[], "base":[]}
    print("Density sweep...")
    for lam in lams:
        for pol in ("cloud","base"):
            r = simulate(lam, pol, rng=np.random.default_rng(Cfg.seed))
            sweep[pol].append(r)
            print(f"  lam={lam:.2f} {pol:5s}  Q={r['Q']:6.0f}  "
                  f"Wmean={r['wait_mean']:5.1f}  P95={r['wait_p95']:5.1f}  "
                  f"Tt={r['travel_mean']:5.1f}")

    fig, ax = plt.subplots(figsize=(5.2,3.6))
    ax.plot(lams,[r["Q"] for r in sweep["base"]], "o-", color="#c0392b", label="Decentralized")
    ax.plot(lams,[r["Q"] for r in sweep["cloud"]],"s-", color="#1f5fbf", label="Cloud-assisted")
    ax.set_xlabel("arrival rate $\\lambda$ [veh/s/dir]"); ax.set_ylabel("throughput Q [veh/h]")
    ax.legend(fontsize=9); ax.grid(alpha=0.3); fig.tight_layout()
    for ext in ("pdf","png"): fig.savefig(f"out/fig_sweep_throughput.{ext}", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(5.2,3.6))
    ax.plot(lams,[r["wait_mean"] for r in sweep["base"]], "o-", color="#c0392b", label="Decentralized (mean)")
    ax.plot(lams,[r["wait_mean"] for r in sweep["cloud"]],"s-", color="#1f5fbf", label="Cloud-assisted (mean)")
    ax.plot(lams,[r["wait_p95"]  for r in sweep["base"]], "o--", color="#c0392b", alpha=0.6, label="Decentralized (P95)")
    ax.plot(lams,[r["wait_p95"]  for r in sweep["cloud"]],"s--", color="#1f5fbf", alpha=0.6, label="Cloud-assisted (P95)")
    ax.set_xlabel("arrival rate $\\lambda$ [veh/s/dir]"); ax.set_ylabel("waiting time [s]")
    ax.legend(fontsize=7); ax.grid(alpha=0.3); fig.tight_layout()
    for ext in ("pdf","png"): fig.savefig(f"out/fig_sweep_waiting.{ext}", dpi=160)
    plt.close(fig)

    # CSVs
    with open("out/sweep.csv","w",newline="") as f:
        w = csv.writer(f); w.writerow(["lambda","policy","n","Q_vehph","wait_mean_s","wait_p95_s","travel_mean_s"])
        for pol in ("base","cloud"):
            for r in sweep[pol]:
                w.writerow([r["lam"],pol,r["n"],f"{r['Q']:.1f}",f"{r['wait_mean']:.2f}",
                            f"{r['wait_p95']:.2f}",f"{r['travel_mean']:.2f}"])

    cb = next(r for r in sweep["base"]  if abs(r["lam"]-lam_rep)<1e-9)
    cc = next(r for r in sweep["cloud"] if abs(r["lam"]-lam_rep)<1e-9)
    with open("out/results_table.csv","w",newline="") as f:
        w = csv.writer(f)
        w.writerow(["Metric","Decentralized","Cloud-Assisted"])
        w.writerow(["Throughput Q [veh/h]", f"{cb['Q']:.0f}", f"{cc['Q']:.0f}"])
        w.writerow(["Average waiting Wbar [s]", f"{cb['wait_mean']:.1f}", f"{cc['wait_mean']:.1f}"])
        w.writerow(["P95 waiting W95 [s]", f"{cb['wait_p95']:.1f}", f"{cc['wait_p95']:.1f}"])
        w.writerow(["Average travel time Tt [s]", f"{cb['travel_mean']:.1f}", f"{cc['travel_mean']:.1f}"])

    print("\n=== Representative scenario (lambda = %.2f) ===" % lam_rep)
    print(f"{'Metric':28s}{'Decentralized':>15s}{'Cloud-Assisted':>16s}")
    print(f"{'Throughput Q [veh/h]':28s}{cb['Q']:>15.0f}{cc['Q']:>16.0f}")
    print(f"{'Avg waiting Wbar [s]':28s}{cb['wait_mean']:>15.1f}{cc['wait_mean']:>16.1f}")
    print(f"{'P95 waiting W95 [s]':28s}{cb['wait_p95']:>15.1f}{cc['wait_p95']:>16.1f}")
    print(f"{'Avg travel time Tt [s]':28s}{cb['travel_mean']:>15.1f}{cc['travel_mean']:>16.1f}")
    print("\nFigures and CSVs written to ./out/")


if __name__ == "__main__":
    main()
