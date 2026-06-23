#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Reviewer-response experiments for the IJAE paper (NEW results).
===============================================================
Extends the validated single-obstacle simulator to answer reviewer concerns:

  (#4) Partial V2X penetration : fraction `pen` of vehicles are connected and
       receive the cloud speed command; the rest are camera-detected only, so the
       cloud still reserves a conflict-free slot for them (safety unaffected) but
       they get no en-route speed advice and stop at the section gate.
  (#5) Multi-obstacle corridor : two obstacles in series, each with its own slot
       scheduler.
  (#2) Adversarial communication : positioning error on the reported position
       (hence the slot timing) and large latency -- shows where efficiency
       degrades while the LOCAL physical interlock keeps the section collision-free.
  (#3) Per-direction fairness numbers under asymmetric demand.
  (#6) Pre-clamp explicit-Euler overlap vs time step dt.

CLI (each call runs one config, 5 seeds, T=600 s, and appends to out_reviewer/):
  python3 reviewer_experiments.py val
  python3 reviewer_experiments.py pen   <p>          # p in [0,1]
  python3 reviewer_experiments.py multi <n_obs> <pen>
  python3 reviewer_experiments.py adv   pos <sigma_m>
  python3 reviewer_experiments.py adv   lat <delay_s>
  python3 reviewer_experiments.py fair                 # asymmetric, per-direction
  python3 reviewer_experiments.py overlap <dt>
"""
import os, sys, csv, math
import numpy as np
from cloud_overtaking_sim import Cfg, idm_accel, Cloud

OUT = "out_reviewer"
Cfg.dt = 0.1; Cfg.T_warm = 60.0; Cfg.T_sim = 600.0
NSEED = 5
E1 = Cfg.L / 2 - Cfg.L_o / 2          # single-obstacle entry (= 485)


class Vh:
    __slots__ = ("vid", "d", "length", "v0i", "s", "v", "t_in", "t_out", "wait",
                 "cv", "resv", "ts", "done", "stops", "stopped")
    def __init__(self, vid, d, length, v0i, t_in, cv):
        self.vid = vid; self.d = d; self.length = length; self.v0i = v0i
        self.s = 0.0; self.v = v0i; self.t_in = t_in; self.t_out = None
        self.wait = 0.0; self.cv = cv
        self.resv = {}; self.ts = {}
        self.done = False; self.stops = 0; self.stopped = False


def simulate_ext(lam, entries, pen=1.0, pos_err=0.0, comm_delay=0.0, dt=None, rng=None):
    if rng is None:
        rng = np.random.default_rng(7)
    if dt is None:
        dt = Cfg.dt
    lam_d = lam if isinstance(lam, dict) else {+1: lam, -1: lam}
    n = int((Cfg.T_warm + Cfg.T_sim) / dt)
    Lo = Cfg.L_o
    secs = [(E, E + Lo, E - Cfg.L_p) for E in entries]
    clouds = [Cloud() for _ in entries]
    next_arr = {+1: rng.exponential(1 / lam_d[+1]), -1: rng.exponential(1 / lam_d[-1])}
    active = {+1: [], -1: []}; finished = []; vid = 0
    min_gap = 1e9; max_overlap = 0.0

    def in_sec(v, k):
        E, X, _ = secs[k]
        return E <= v.s <= X + v.length

    for it in range(n):
        t = it * dt
        for d in (+1, -1):
            while t >= next_arr[d]:
                length = rng.uniform(Cfg.len_min, Cfg.len_max)
                v0i = float(np.clip(Cfg.v0 + rng.uniform(-Cfg.v0_spread, Cfg.v0_spread),
                                    Cfg.v_min, Cfg.v_max))
                cv = (rng.random() < pen)
                nv = Vh(vid, d, length, v0i, t, cv)
                rear = min((vv.s for vv in active[d]), default=Cfg.L)
                nv.s = min(0.0, rear - (length + Cfg.s0))
                active[d].append(nv); vid += 1
                next_arr[d] += rng.exponential(1 / lam_d[d])

        occ = [set() for _ in secs]
        for d in (+1, -1):
            for v in active[d]:
                for k in range(len(secs)):
                    if in_sec(v, k):
                        occ[k].add(d)
        owner = [(next(iter(o)) if len(o) == 1 else (0 if not o else 99)) for o in occ]

        for d in (+1, -1):
            lane = sorted(active[d], key=lambda x: x.s, reverse=True)
            for idx, v in enumerate(lane):
                leader = lane[idx - 1] if idx > 0 else None
                for k, (E, X, P) in enumerate(secs):
                    if (k not in v.resv) and P <= v.s < E:
                        rep_s = v.s + (rng.normal(0, pos_err) if pos_err > 0 else 0.0)
                        r_i = t + comm_delay + (E - rep_s) / max(v.v, Cfg.v_min)
                        D_i = (Lo + v.length) / Cfg.v_sec + Cfg.g_same
                        v.ts[k] = clouds[k].reserve(d, r_i, D_i)
                        v.resv[k] = True
                kx = None
                for k, (E, X, P) in enumerate(secs):
                    if v.s < E:
                        kx = k; break
                v0_eff = v.v0i
                if kx is not None and v.cv and v.resv.get(kx) and v.ts.get(kx) is not None:
                    E = secs[kx][0]; rem = v.ts[kx] - t
                    if rem > 0.1:
                        v0_eff = float(np.clip((E - v.s) / rem, Cfg.v_min, Cfg.v_max))
                for (E, X, P) in secs:
                    if E - 5 <= v.s <= X:
                        v0_eff = min(v0_eff, Cfg.v_sec)
                if leader is not None:
                    gap = leader.s - leader.length - v.s; dv = v.v - leader.v
                else:
                    gap = 1e6; dv = 0.0
                if kx is not None:
                    E = secs[kx][0]
                    opp_in = (len(occ[kx]) == 1 and owner[kx] == -d)
                    granted = (v.resv.get(kx) and t >= (v.ts.get(kx) or t))
                    allow = granted and (not opp_in)
                    if not allow:
                        slg = E - v.s
                        if slg < gap:
                            gap = slg; dv = v.v
                a = idm_accel(v.v, v0_eff, gap, dv)
                v.v = max(0.0, v.v + a * dt); v.s += v.v * dt
                if leader is not None:
                    rear = leader.s - leader.length
                    if v.s > rear:
                        ov = v.s - rear
                        if ov > max_overlap:
                            max_overlap = ov
                        v.s = rear; v.v = min(v.v, leader.v)
                if kx is not None:
                    if v.v < 0.3 and not v.stopped:
                        v.stops += 1; v.stopped = True
                    elif v.v > 1.0:
                        v.stopped = False
                if t >= Cfg.T_warm and v.v < Cfg.v_wait_th and (kx is not None):
                    v.wait += dt
                if v.s >= Cfg.L and not v.done:
                    v.done = True; v.t_out = t
        for d in (+1, -1):
            ln = sorted(active[d], key=lambda x: x.s, reverse=True)
            for i in range(1, len(ln)):
                g = ln[i - 1].s - ln[i - 1].length - ln[i].s
                if g < min_gap:
                    min_gap = g
        for d in (+1, -1):
            keep = []
            for v in active[d]:
                (finished if v.done else keep).append(v)
            active[d] = keep

    T_total = Cfg.T_warm + Cfg.T_sim
    meas = [v for v in finished if v.t_out is not None and Cfg.T_warm <= v.t_out <= T_total]
    waits = np.array([v.wait for v in meas]) if meas else np.array([0.0])
    stops = np.array([v.stops for v in meas]) if meas else np.array([0.0])
    wpos = [v.wait for v in meas if v.d == +1]; wneg = [v.wait for v in meas if v.d == -1]
    return dict(Q=len(meas) / Cfg.T_sim * 3600.0,
                wait=float(np.mean(waits)), p95=float(np.percentile(waits, 95)),
                stops=float(np.mean(stops)), min_gap=min_gap, max_overlap=max_overlap,
                w_pos=float(np.mean(wpos)) if wpos else 0.0,
                w_neg=float(np.mean(wneg)) if wneg else 0.0,
                q_pos=len(wpos) / Cfg.T_sim * 3600.0, q_neg=len(wneg) / Cfg.T_sim * 3600.0)


def msd(vals):
    return float(np.mean(vals)), float(np.std(vals))


def run(fn):
    rs = [fn(np.random.default_rng(s)) for s in range(1, NSEED + 1)]
    out = {k: msd([r[k] for r in rs]) for k in rs[0]}
    out["min_gap"] = (min(r["min_gap"] for r in rs), 0.0)
    out["max_overlap"] = (max(r["max_overlap"] for r in rs), 0.0)
    return out


def append_row(fname, header, row):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, fname)
    new = not os.path.exists(p)
    with open(p, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(header)
        w.writerow(row)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "val"

    if cmd == "val":
        a = run(lambda r: simulate_ext(0.30, [E1], pen=1.0, rng=r))
        print(f"VAL pen=1 single: Q={a['Q'][0]:.0f}+-{a['Q'][1]:.0f} "
              f"W={a['wait'][0]:.0f} P95={a['p95'][0]:.0f} mingap={a['min_gap'][0]:.3f}")
        print("  (published: Q=581 W=167 P95=384)")

    elif cmd == "pen":
        p = float(sys.argv[2])
        lamp = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15
        a = run(lambda r: simulate_ext(lamp, [E1], pen=p, rng=r))
        append_row("penetration.csv",
                   ["lam", "pen", "Q", "Qsd", "W", "P95", "stops", "min_gap"],
                   [lamp, p, f"{a['Q'][0]:.1f}", f"{a['Q'][1]:.1f}", f"{a['wait'][0]:.1f}",
                    f"{a['p95'][0]:.1f}", f"{a['stops'][0]:.2f}", f"{a['min_gap'][0]:.3f}"])
        print(f"PEN {p}: Q={a['Q'][0]:.0f}+-{a['Q'][1]:.0f} W={a['wait'][0]:.0f} "
              f"stops={a['stops'][0]:.1f} mingap={a['min_gap'][0]:.3f}")

    elif cmd == "multi":
        nob = int(sys.argv[2]); pen = float(sys.argv[3])
        if nob == 1:
            ent = [E1]
        else:
            ent = [Cfg.L / 3 - Cfg.L_o / 2, 2 * Cfg.L / 3 - Cfg.L_o / 2]
        a = run(lambda r: simulate_ext(0.30, ent, pen=pen, rng=r))
        append_row("multi.csv",
                   ["n_obs", "pen", "Q", "Qsd", "W", "P95", "stops", "min_gap"],
                   [nob, pen, f"{a['Q'][0]:.1f}", f"{a['Q'][1]:.1f}", f"{a['wait'][0]:.1f}",
                    f"{a['p95'][0]:.1f}", f"{a['stops'][0]:.2f}", f"{a['min_gap'][0]:.3f}"])
        print(f"MULTI n={nob} pen={pen}: Q={a['Q'][0]:.0f}+-{a['Q'][1]:.0f} "
              f"W={a['wait'][0]:.0f} stops={a['stops'][0]:.1f} mingap={a['min_gap'][0]:.3f}")

    elif cmd == "adv":
        kind = sys.argv[2]; val = float(sys.argv[3])
        if kind == "pos":
            a = run(lambda r: simulate_ext(0.30, [E1], pen=1.0, pos_err=val, rng=r))
            tag = f"pos={val}m"
        else:
            a = run(lambda r: simulate_ext(0.30, [E1], pen=1.0, comm_delay=val, rng=r))
            tag = f"lat={val}s"
        append_row("adversarial.csv",
                   ["kind", "value", "Q", "Qsd", "W", "min_gap", "max_overlap"],
                   [kind, val, f"{a['Q'][0]:.1f}", f"{a['Q'][1]:.1f}", f"{a['wait'][0]:.1f}",
                    f"{a['min_gap'][0]:.3f}", f"{a['max_overlap'][0]:.4f}"])
        print(f"ADV {tag}: Q={a['Q'][0]:.0f}+-{a['Q'][1]:.0f} W={a['wait'][0]:.0f} "
              f"mingap={a['min_gap'][0]:.3f} maxoverlap={a['max_overlap'][0]:.4f}")

    elif cmd == "fair":
        a = run(lambda r: simulate_ext({+1: 0.40, -1: 0.08}, [E1], pen=1.0, rng=r))
        append_row("fairness.csv",
                   ["case", "Q_heavy", "Q_light", "W_heavy", "W_light"],
                   ["asym_0p40_0p08", f"{a['q_pos'][0]:.1f}", f"{a['q_neg'][0]:.1f}",
                    f"{a['w_pos'][0]:.1f}", f"{a['w_neg'][0]:.1f}"])
        print(f"FAIR asym: Qheavy={a['q_pos'][0]:.0f} Qlight={a['q_neg'][0]:.0f} "
              f"Wheavy={a['w_pos'][0]:.1f} Wlight={a['w_neg'][0]:.1f}")

    elif cmd == "overlap":
        dtv = float(sys.argv[2])
        a = run(lambda r: simulate_ext(0.30, [E1], pen=1.0, dt=dtv, rng=r))
        append_row("overlap.csv",
                   ["dt", "Q", "max_overlap_m", "min_gap"],
                   [dtv, f"{a['Q'][0]:.1f}", f"{a['max_overlap'][0]:.5f}", f"{a['min_gap'][0]:.3f}"])
        print(f"OVERLAP dt={dtv}: Q={a['Q'][0]:.0f} max_overlap={a['max_overlap'][0]*100:.2f}cm")
