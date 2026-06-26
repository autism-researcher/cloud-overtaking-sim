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
    # Vehicle-actuated signal baseline (demand-responsive work-zone signal)
    act_gmin   = 5.0         # minimum green per direction [s]
    act_gmax   = 20.0        # maximum green per direction [s]
    # Communication-imperfection model (cloud target-speed command staleness)
    comm_delay = 0.0         # latency before a vehicle's first command applies [s]
    comm_loss  = 0.0         # per-step probability the target-speed update is dropped

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
                 "Th","am","bc","vsec","cmd_v0")
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
        self.cmd_v0 = None        # last cloud target-speed command actually received


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
SAFETY_VIOL = [0]   # counts steps where both directions occupy the shared section

def in_section(v):
    return Cfg.S_entry <= v.s <= Cfg.S_exit + v.length

def blocks_opposing(v):
    """A vehicle blocks an opposing entry while it is anywhere in the shared
    section OR within the opposing safety clearance (g_opp) of the exit, so that
    an entering vehicle is guaranteed real spatial separation from it. This is the
    physical interlock that enforces strict directional mutual exclusion."""
    return Cfg.S_entry <= v.s <= (Cfg.S_exit + v.length + Cfg.g_opp*Cfg.v_sec)

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
    sec_claim = 0                   # direction that physically owns the shared section
                                    # (hard mutual-exclusion interlock for signal/
                                    # actuated/cloud; 0 = section free)

    # actuated-signal state
    act_dir = +1                    # direction currently holding (or last held) green
    act_t0 = 0.0                    # time the current green began
    act_clearing = False            # True during all-red clearance
    act_clear_t0 = 0.0              # time the clearance began
    act_green = 0                   # green direction this step (0 = all-red)

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
                if getattr(Cfg, "bursty", False):
                    import math as _m
                    _mult = 1.0 + 0.9*_m.sin(2*_m.pi*next_arr[d]/getattr(Cfg, "burst_period", 180.0))
                    _lt = max(lam_d[d]*_mult, 0.02*lam_d[d])
                    next_arr[d] += rng.exponential(1.0/_lt)
                else:
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
        if len(occ_dirs) > 1:
            SAFETY_VIOL[0] += 1

        # ---- actuated-signal phase (computed once per step) ----
        if policy == "actuated":
            def _demand(dd):
                return any(Cfg.S_prec <= vv.s < Cfg.S_entry for vv in active[dd])
            if act_clearing:
                act_green = 0
                if t - act_clear_t0 >= Cfg.sig_clear:
                    act_clearing = False
                    act_dir = -act_dir          # hand green to the opposing approach
                    act_t0 = t
            else:
                elapsed = t - act_t0
                dem_cur = _demand(act_dir)
                dem_opp = _demand(-act_dir)
                end_phase = False
                if elapsed >= Cfg.act_gmin:
                    if elapsed >= Cfg.act_gmax and dem_opp:
                        end_phase = True        # max-out: yield to a waiting opponent
                    elif (not dem_cur) and dem_opp:
                        end_phase = True        # gap-out: current queue cleared, opp waits
                if end_phase:
                    act_clearing = True; act_clear_t0 = t; act_green = 0
                else:
                    act_green = act_dir

        # ---- physical section-ownership interlock (hard mutual exclusion) ----
        # Release the section once its owning direction has fully cleared the
        # section and the opposing safety clearance; if the section is free but
        # still physically occupied (e.g. just after release), reclaim it for the
        # occupying direction. This guarantees only one direction is ever inside.
        if sec_claim and not any(blocks_opposing(vv) for vv in active[sec_claim]):
            sec_claim = 0

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
                    new_cmd = float(np.clip(dist/rem, Cfg.v_min, Cfg.v_max)) if rem > 0.1 else v.v0i
                    # command staleness under latency + packet loss: the first command
                    # applies only after the link delay, and each update may be dropped;
                    # otherwise the vehicle holds its last received target speed.
                    cmd_ready = (t >= v.t_in + Cfg.comm_delay)
                    dropped = (Cfg.comm_loss > 0.0 and rng.random() < Cfg.comm_loss)
                    if cmd_ready and not dropped:
                        v.cmd_v0 = new_cmd
                    v0_eff = v.cmd_v0 if v.cmd_v0 is not None else v.v0i
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
                        opp_in = (sec_claim not in (0, d))
                        allow_enter = (green_dir == d) and (not opp_in)
                    elif policy == "actuated":
                        # Vehicle-actuated signal: green extends while the served
                        # direction has demand (up to a max green) and gaps out to
                        # the opposing approach when its queue clears, with the same
                        # all-red clearance. Vehicles STOP at red (no speed shaping).
                        opp_in = (sec_claim not in (0, d))
                        allow_enter = (act_green == d) and (not opp_in)
                    else:  # cloud
                        opp_in = (sec_claim not in (0, d))
                        # the "you may enter" grant is delayed by the link latency and
                        # may be dropped (held a step) by packet loss; safety is still
                        # enforced locally by the interlock above, independent of the link.
                        slot_open = (v.t_s is None) or (t >= v.t_s + Cfg.comm_delay)
                        if Cfg.comm_loss > 0.0 and rng.random() < Cfg.comm_loss:
                            slot_open = False
                        allow_enter = slot_open and (not opp_in)

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
                # hard stop-line: a gated vehicle physically cannot run the red into
                # the shared section (prevents IDM overshoot past the entry line).
                if (not allow_enter) and s_prev < Cfg.S_entry and v.s >= Cfg.S_entry:
                    v.s = Cfg.S_entry - 0.1; v.v = 0.0
                # claim the section the instant a PERMITTED vehicle crosses the entry
                # line, so the opposing direction (processed later this step, or in
                # later steps) is held out until this direction fully clears. A blocked
                # vehicle is parked just behind the line and never claims.
                if allow_enter and s_prev < Cfg.S_entry <= v.s:
                    sec_claim = d

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

