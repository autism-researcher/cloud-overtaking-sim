#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robustness to communication imperfections
=========================================
Extends the cloud-overtaking study with imperfect V2X links and measures how
the proposed coordinator degrades under (i) communication latency and
(ii) packet loss. Reuses the validated dynamics (IDM, slot reservation) from
cloud_overtaking_sim.py, so results at zero delay / zero loss reproduce the
ideal-communication case.

Model
-----
* Latency tau: a vehicle's reservation request reaches the cloud, and the slot
  + target-velocity command returns, only after a delay tau. Until the command
  is received the vehicle has no slot and is held conservatively at the section
  entry by the IDM (treating the entry as a stationary virtual leader).
* Packet loss p: at each control step a command update is dropped with
  probability p; the vehicle then holds its last received command. The IDM
  safety filter remains active at all times, so safety never depends on the
  link.

Outputs (./out_comm):
  comm_latency.csv   Q, Wbar, min-gap vs latency (loss = 0)
  comm_loss.csv      Q, Wbar, min-gap vs packet loss (latency = 100 ms)
  fig_comm.pdf/.png  two-panel robustness figure

Run:  python comm_delay_experiment.py
Author: M. B. Hossain, O. Tayan, M. A. S. Kamal
"""
import os, csv
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from cloud_overtaking_sim import Cfg, idm_accel, Cloud, in_section

# ---- production configuration (match the main study) ----
Cfg.dt = 0.1; Cfg.T_warm = 60.0; Cfg.T_sim = 600.0
LAM      = 0.30
N_SEEDS  = 10
BASE_SEED = 1
LATENCIES = [0.0, 0.05, 0.10, 0.20, 0.50]   # s
LOSSES    = [0.0, 0.05, 0.10, 0.20]          # fraction
LOSS_LAT  = 0.10                              # latency used for the loss sweep [s]
OUT = "out_comm"
CB, CC = "#c0392b", "#1f5fbf"


class V:
    __slots__ = ("vid","d","length","v0i","s","v","t_in","t_out","wait",
                 "reserved","t_s","D","done","cmd_at","last_cmd")
    def __init__(self, vid, d, length, v0i, t_in):
        self.vid=vid; self.d=d; self.length=length; self.v0i=v0i
        self.s=0.0; self.v=v0i; self.t_in=t_in; self.t_out=None; self.wait=0.0
        self.reserved=False; self.t_s=None; self.D=None; self.done=False
        self.cmd_at=None; self.last_cmd=None


def simulate_comm(lam, comm_delay=0.0, packet_loss=0.0, rng=None):
    if rng is None: rng = np.random.default_rng(Cfg.seed)
    dt = Cfg.dt; n = int((Cfg.T_warm+Cfg.T_sim)/dt)
    next_arr = {+1: rng.exponential(1/lam), -1: rng.exponential(1/lam)}
    active = {+1: [], -1: []}; finished = []; cloud = Cloud(); vid = 0
    min_gap = 1e9
    for k in range(n):
        t = k*dt
        for d in (+1,-1):
            while t >= next_arr[d]:
                length = rng.uniform(Cfg.len_min, Cfg.len_max)
                v0i = float(np.clip(Cfg.v0 + rng.uniform(-Cfg.v0_spread, Cfg.v0_spread),
                                    Cfg.v_min, Cfg.v_max))
                nv = V(vid,d,length,v0i,t)
                rear = min((vv.s for vv in active[d]), default=Cfg.L)
                # inject at the origin, or just behind the rear-most vehicle (as a
                # feeder queue) so two arrivals never overlap at s=0
                nv.s = min(0.0, rear - (length + Cfg.s0))
                active[d].append(nv); vid += 1
                next_arr[d] += rng.exponential(1/lam)
        occ = set()
        for d in (+1,-1):
            for v in active[d]:
                if in_section(v): occ.add(d)
        owner = next(iter(occ)) if len(occ)==1 else (0 if not occ else 99)
        for d in (+1,-1):
            lane = sorted(active[d], key=lambda x: x.s, reverse=True)
            for idx, v in enumerate(lane):
                leader = lane[idx-1] if idx>0 else None
                # reservation request (slot + command available only after comm_delay)
                if (not v.reserved) and Cfg.S_prec <= v.s < Cfg.S_entry:
                    r_i = t + comm_delay + (Cfg.S_entry - v.s)/max(v.v, Cfg.v_min)
                    v.D = (Cfg.L_o + v.length)/Cfg.v_sec + Cfg.g_same
                    v.t_s = cloud.reserve(d, r_i, v.D); v.reserved = True
                    v.cmd_at = t + comm_delay
                # command availability under delay + loss
                cmd_ok = v.reserved and v.cmd_at is not None and t >= v.cmd_at
                if cmd_ok and packet_loss > 0 and rng.random() < packet_loss:
                    cmd_ok = False
                v0_eff = v.v0i
                if cmd_ok and v.s < Cfg.S_entry and v.t_s is not None:
                    dist = Cfg.S_entry - v.s; rem = v.t_s - t
                    v0_eff = float(np.clip(dist/rem, Cfg.v_min, Cfg.v_max)) if rem > 0.1 else v.v0i
                    v.last_cmd = v0_eff
                elif v.reserved and (not cmd_ok) and v.last_cmd is not None and v.s < Cfg.S_entry:
                    v0_eff = v.last_cmd
                if Cfg.S_entry - 5 <= v.s <= Cfg.S_exit:
                    v0_eff = min(v0_eff, Cfg.v_sec)
                if leader is not None:
                    gap = leader.s - leader.length - v.s; dv = v.v - leader.v
                else:
                    gap = 1e6; dv = 0.0
                # entry permission: command received, slot open, no opposing in section
                allow = True
                if v.s < Cfg.S_entry:
                    opp_in = (len(occ)==1 and owner == -d)
                    granted = (v.reserved and v.cmd_at is not None and t >= v.cmd_at
                               and t >= (v.t_s or t))
                    allow = granted and (not opp_in)
                if (not allow) and v.s < Cfg.S_entry:
                    slg = Cfg.S_entry - v.s
                    if slg < gap: gap = slg; dv = v.v
                a = idm_accel(v.v, v0_eff, gap, dv)
                v.v = max(0.0, v.v + a*dt); v.s += v.v*dt
                # hard non-overlap constraint: explicit-Euler integration of the
                # IDM can let a follower creep a fraction past a stopped leader in
                # a dense standing queue, and since vehicles cannot reverse the
                # penetration would freeze. Continuous-time IDM never overlaps, so
                # we clamp the follower to the leader's rear bumper. This affects
                # only would-be overlaps (jammed cells where speeds are ~0) and
                # leaves free-flow dynamics, throughput, and waiting unchanged.
                if leader is not None:
                    rear = leader.s - leader.length
                    if v.s > rear:
                        v.s = rear; v.v = min(v.v, leader.v)
                if t >= Cfg.T_warm and v.v < Cfg.v_wait_th and v.s < Cfg.S_exit:
                    v.wait += dt
                if v.s >= Cfg.L and not v.done:
                    v.done = True; v.t_out = t
        # consistent end-of-step minimum-gap check: a single simultaneous
        # snapshot after every vehicle has moved (avoids mixing pre/post-update
        # positions). This is the physically meaningful collision metric.
        for d in (+1,-1):
            ln = sorted(active[d], key=lambda x: x.s, reverse=True)
            for i in range(1, len(ln)):
                g = ln[i-1].s - ln[i-1].length - ln[i].s
                if g < min_gap: min_gap = g
        for d in (+1,-1):
            keep = []
            for v in active[d]:
                (finished if v.done else keep).append(v)
            active[d] = keep
    T_total = Cfg.T_warm + Cfg.T_sim
    meas = [v for v in finished if v.t_out is not None and Cfg.T_warm <= v.t_out <= T_total]
    waits = np.array([v.wait for v in meas]) if meas else np.array([0.0])
    return dict(Q=len(meas)/Cfg.T_sim*3600.0, wait=float(np.mean(waits)), min_gap=min_gap)


def agg(delay, loss):
    Q=[]; W=[]; mg=1e9
    for k in range(N_SEEDS):
        r = simulate_comm(LAM, delay, loss, rng=np.random.default_rng(BASE_SEED+k))
        Q.append(r["Q"]); W.append(r["wait"]); mg = min(mg, r["min_gap"])
    return (float(np.mean(Q)), float(np.std(Q)),
            float(np.mean(W)), float(np.std(W)), mg)


def main():
    os.makedirs(OUT, exist_ok=True)
    print(f"Communication-robustness sweep: lam={LAM}, {N_SEEDS} seeds, "
          f"dt={Cfg.dt}, T_sim={Cfg.T_sim}s")
    lat_rows=[]; loss_rows=[]
    print("-- latency sweep (loss=0) --")
    for tau in LATENCIES:
        Qm,Qs,Wm,Ws,mg = agg(tau, 0.0)
        coll = "yes" if mg < -0.1 else "no"
        lat_rows.append([int(tau*1000),f"{Qm:.1f}",f"{Qs:.1f}",f"{Wm:.1f}",f"{Ws:.1f}",f"{mg:.2f}",coll])
        print(f"  tau={int(tau*1000):3d} ms  Q={Qm:6.1f}+-{Qs:4.1f}  W={Wm:6.1f}+-{Ws:4.1f}  min_gap={mg:5.2f}  collision={coll}")
    print("-- packet-loss sweep (latency=%d ms) --" % int(LOSS_LAT*1000))
    for p in LOSSES:
        Qm,Qs,Wm,Ws,mg = agg(LOSS_LAT, p)
        coll = "yes" if mg < -0.1 else "no"
        loss_rows.append([f"{p*100:.0f}",f"{Qm:.1f}",f"{Qs:.1f}",f"{Wm:.1f}",f"{Ws:.1f}",f"{mg:.2f}",coll])
        print(f"  loss={p*100:4.0f}%  Q={Qm:6.1f}+-{Qs:4.1f}  W={Wm:6.1f}+-{Ws:4.1f}  min_gap={mg:5.2f}  collision={coll}")

    with open(f"{OUT}/comm_latency.csv","w",newline="") as f:
        w=csv.writer(f); w.writerow(["latency_ms","Q_mean","Q_std","W_mean","W_std","min_gap_m","collision"]); w.writerows(lat_rows)
    with open(f"{OUT}/comm_loss.csv","w",newline="") as f:
        w=csv.writer(f); w.writerow(["loss_pct","Q_mean","Q_std","W_mean","W_std","min_gap_m","collision"]); w.writerows(loss_rows)

    # ---- figure: two panels ----
    fig,(ax1,ax2)=plt.subplots(1,2,figsize=(7.4,3.2))
    lat=[r[0] for r in lat_rows]; Ql=[float(r[1]) for r in lat_rows]; Qle=[float(r[2]) for r in lat_rows]
    Wl=[float(r[3]) for r in lat_rows]; Wle=[float(r[4]) for r in lat_rows]
    ax1.errorbar(lat,Ql,yerr=Qle,fmt="s-",color=CC,capsize=3,label="throughput")
    ax1.set_xlabel("communication latency [ms]"); ax1.set_ylabel("throughput Q [veh/h]",color=CC)
    ax1b=ax1.twinx(); ax1b.errorbar(lat,Wl,yerr=Wle,fmt="o--",color=CB,capsize=3,alpha=0.8,label="waiting")
    ax1b.set_ylabel("avg. waiting [s]",color=CB)
    ax1.set_title("(a) latency sweep (no loss)",fontsize=9)
    lp=[float(r[0]) for r in loss_rows]; Qp=[float(r[1]) for r in loss_rows]; Qpe=[float(r[2]) for r in loss_rows]
    Wp=[float(r[3]) for r in loss_rows]; Wpe=[float(r[4]) for r in loss_rows]
    ax2.errorbar(lp,Qp,yerr=Qpe,fmt="s-",color=CC,capsize=3)
    ax2.set_xlabel("packet loss [%]"); ax2.set_ylabel("throughput Q [veh/h]",color=CC)
    ax2b=ax2.twinx(); ax2b.errorbar(lp,Wp,yerr=Wpe,fmt="o--",color=CB,capsize=3,alpha=0.8)
    ax2b.set_ylabel("avg. waiting [s]",color=CB)
    ax2.set_title("(b) packet-loss sweep (100 ms latency)",fontsize=9)
    fig.tight_layout()
    for e in ("pdf","png"): fig.savefig(f"{OUT}/fig_comm.{e}",dpi=160)
    plt.close(fig)
    print(f"\nOutputs written to ./{OUT}/")


if __name__ == "__main__":
    main()
