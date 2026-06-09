#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-seed production run for the cloud-overtaking study.
=========================================================
Reuses cloud_overtaking_sim.py and repeats every operating point over N_SEEDS
random seeds, reporting mean +/- std so the paper numbers are statistically
honest (not a single run). Writes everything to ./out_final/.

Run:
    python run_multiseed.py

This is the run whose output should go into the FINAL paper. It takes several
minutes (production config: dt=0.1, T_sim=600 s, 10 seeds, 10 densities x 2
policies). Lower N_SEEDS or T_sim for a quick check.

Outputs (./out_final/):
    sweep_agg.csv             mean & std of Q, Wbar, W95, Tt at each lambda
    results_table_final.csv   headline table (mean +/- std) at lambda=0.30
    fig_sweep_throughput.pdf  throughput vs lambda, with std error bars
    fig_sweep_waiting.pdf      mean & P95 waiting vs lambda, with error bars
    fig_waiting_cdf.pdf        waiting CDF pooled over all seeds at lambda=0.30
    fig_spacetime_cloud.pdf    representative space-time (single seed)
    fig_spacetime_base.pdf     representative space-time (single seed)

Author: M. B. Hossain, O. Tayan, M. A. S. Kamal
"""
import os, csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from cloud_overtaking_sim import Cfg, simulate

# ---------------------------------------------------------------------------
# Production configuration  (override the simulator defaults at runtime)
# ---------------------------------------------------------------------------
Cfg.dt      = 0.1            # finer step for the final numbers
Cfg.T_warm  = 60.0
Cfg.T_sim   = 600.0

N_SEEDS   = 10               # number of random seeds per operating point
BASE_SEED = 1                # seeds used are BASE_SEED .. BASE_SEED+N_SEEDS-1
LAMS      = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]
LAM_REP   = 0.30             # representative density for the headline table
OUT       = "out_final"
C_BASE, C_CLOUD = "#c0392b", "#1f5fbf"


def run_seeds(lam, policy):
    """Run N_SEEDS independent replications; return per-seed metric arrays."""
    Q, Wm, Wp, Tt, waits = [], [], [], [], []
    for k in range(N_SEEDS):
        rng = np.random.default_rng(BASE_SEED + k)
        r = simulate(lam, policy, rng=rng)
        Q.append(r["Q"]); Wm.append(r["wait_mean"])
        Wp.append(r["wait_p95"]); Tt.append(r["travel_mean"])
        waits.append(r["waits"])
    return dict(Q=np.array(Q), Wm=np.array(Wm), Wp=np.array(Wp),
                Tt=np.array(Tt), waits=np.concatenate(waits))


def ms(a):                    # mean, std helper
    return float(np.mean(a)), float(np.std(a))


def spacetime(lam, policy, title, fname, t0, t1):
    r = simulate(lam, policy, record_tracks=True,
                 rng=np.random.default_rng(BASE_SEED))
    fig, ax = plt.subplots(figsize=(5.2, 3.6))
    for _, (ts, ss, d) in r["tracks"].items():
        if not ts:
            continue
        ts = np.array(ts); ss = np.array(ss)
        m = (ts >= t0) & (ts <= t1)
        if m.sum() < 2:
            continue
        ax.plot(ts[m], ss[m], color=(C_CLOUD if d == +1 else C_BASE),
                lw=0.7, alpha=0.85)
    ax.axhspan(Cfg.S_entry, Cfg.S_exit, color="0.75", alpha=0.6, zorder=0)
    ax.axhline(Cfg.S_entry, color="k", ls="--", lw=0.6)
    ax.set_xlim(t0, t1); ax.set_ylim(0, Cfg.L)
    ax.set_xlabel("time [s]"); ax.set_ylabel("position along corridor [m]")
    ax.set_title(title, fontsize=10)
    ax.legend([Line2D([0], [0], color=C_CLOUD), Line2D([0], [0], color=C_BASE)],
              ["direction L->R", "direction R->L"], fontsize=8, loc="lower right")
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(f"{OUT}/{fname}.{ext}", dpi=160)
    plt.close(fig)
    return r["waits"]


def main():
    os.makedirs(OUT, exist_ok=True)
    print(f"Multi-seed run: {N_SEEDS} seeds, dt={Cfg.dt}, "
          f"T_sim={Cfg.T_sim}s, {len(LAMS)} densities x 2 policies")

    # ---- density sweep (aggregated over seeds) ----
    agg = {"base": {}, "cloud": {}}
    rows = []
    for lam in LAMS:
        for pol in ("base", "cloud"):
            s = run_seeds(lam, pol)
            agg[pol][lam] = s
            (Qm, Qs), (Wmm, Wms) = ms(s["Q"]), ms(s["Wm"])
            (Wpm, Wps), (Ttm, Tts) = ms(s["Wp"]), ms(s["Tt"])
            rows.append([lam, pol, f"{Qm:.1f}", f"{Qs:.1f}",
                         f"{Wmm:.2f}", f"{Wms:.2f}", f"{Wpm:.2f}", f"{Wps:.2f}",
                         f"{Ttm:.2f}", f"{Tts:.2f}"])
            print(f"  lam={lam:.2f} {pol:5s}  Q={Qm:6.1f}+-{Qs:4.1f}  "
                  f"Wmean={Wmm:6.1f}+-{Wms:4.1f}  P95={Wpm:6.1f}  Tt={Ttm:6.1f}")

    with open(f"{OUT}/sweep_agg.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["lambda", "policy", "Q_mean", "Q_std", "wait_mean",
                    "wait_std", "p95_mean", "p95_std", "travel_mean", "travel_std"])
        w.writerows(rows)

    # ---- throughput sweep figure (error bars = std) ----
    fig, ax = plt.subplots(figsize=(5.2, 3.6))
    for pol, c, lab in (("base", C_BASE, "Decentralized"),
                        ("cloud", C_CLOUD, "Cloud-assisted")):
        m = [ms(agg[pol][l]["Q"])[0] for l in LAMS]
        sd = [ms(agg[pol][l]["Q"])[1] for l in LAMS]
        ax.errorbar(LAMS, m, yerr=sd, fmt="o-" if pol == "base" else "s-",
                    color=c, capsize=3, label=lab)
    ax.set_xlabel(r"arrival rate $\lambda$ [veh/s/dir]")
    ax.set_ylabel("throughput Q [veh/h]")
    ax.legend(fontsize=9); ax.grid(alpha=0.3); fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(f"{OUT}/fig_sweep_throughput.{ext}", dpi=160)
    plt.close(fig)

    # ---- waiting sweep figure (mean & P95, error bars) ----
    fig, ax = plt.subplots(figsize=(5.2, 3.6))
    for pol, c, lab in (("base", C_BASE, "Decentralized"),
                        ("cloud", C_CLOUD, "Cloud-assisted")):
        mm = [ms(agg[pol][l]["Wm"])[0] for l in LAMS]
        ms_ = [ms(agg[pol][l]["Wm"])[1] for l in LAMS]
        pm = [ms(agg[pol][l]["Wp"])[0] for l in LAMS]
        ax.errorbar(LAMS, mm, yerr=ms_, fmt="o-" if pol == "base" else "s-",
                    color=c, capsize=3, label=f"{lab} (mean)")
        ax.plot(LAMS, pm, "o--" if pol == "base" else "s--", color=c,
                alpha=0.55, label=f"{lab} (P95)")
    ax.set_xlabel(r"arrival rate $\lambda$ [veh/s/dir]")
    ax.set_ylabel("waiting time [s]")
    ax.legend(fontsize=7); ax.grid(alpha=0.3); fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(f"{OUT}/fig_sweep_waiting.{ext}", dpi=160)
    plt.close(fig)

    # ---- waiting CDF pooled over all seeds at LAM_REP ----
    fig, ax = plt.subplots(figsize=(5.2, 3.6))
    for pol, c, lab in (("base", C_BASE, "Decentralized"),
                        ("cloud", C_CLOUD, "Cloud-assisted")):
        w = np.sort(agg[pol][LAM_REP]["waits"])
        y = np.arange(1, len(w) + 1) / len(w)
        ax.plot(w, y, color=c, lw=1.8, label=lab)
    ax.set_xlabel("per-vehicle waiting time [s]"); ax.set_ylabel("empirical CDF")
    ax.set_ylim(0, 1.02); ax.legend(fontsize=9); ax.grid(alpha=0.3)
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(f"{OUT}/fig_waiting_cdf.{ext}", dpi=160)
    plt.close(fig)

    # ---- representative space-time diagrams (single seed, illustrative) ----
    t0, t1 = Cfg.T_warm, Cfg.T_warm + 180
    spacetime(LAM_REP, "cloud", "Proposed cloud-assisted coordination",
              "fig_spacetime_cloud", t0, t1)
    spacetime(LAM_REP, "base", "Decentralized baseline",
              "fig_spacetime_base", t0, t1)

    # ---- headline table (mean +/- std) at LAM_REP ----
    b, c = agg["base"][LAM_REP], agg["cloud"][LAM_REP]
    def cell(a): m, s = ms(a); return f"{m:.1f} +/- {s:.1f}"
    with open(f"{OUT}/results_table_final.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Metric", "Decentralized (mean+/-std)", "Cloud-Assisted (mean+/-std)"])
        w.writerow(["Throughput Q [veh/h]", cell(b["Q"]), cell(c["Q"])])
        w.writerow(["Average waiting Wbar [s]", cell(b["Wm"]), cell(c["Wm"])])
        w.writerow(["P95 waiting W95 [s]", cell(b["Wp"]), cell(c["Wp"])])
        w.writerow(["Average travel time Tt [s]", cell(b["Tt"]), cell(c["Tt"])])

    print(f"\n=== Headline metrics at lambda={LAM_REP} (mean +/- std over "
          f"{N_SEEDS} seeds) ===")
    print(f"{'Metric':26s}{'Decentralized':>18s}{'Cloud-Assisted':>18s}")
    for name, key in (("Throughput Q [veh/h]", "Q"),
                      ("Avg waiting Wbar [s]", "Wm"),
                      ("P95 waiting W95 [s]", "Wp"),
                      ("Avg travel time Tt [s]", "Tt")):
        print(f"{name:26s}{cell(b[key]):>18s}{cell(c[key]):>18s}")
    print(f"\nAll outputs written to ./{OUT}/")


if __name__ == "__main__":
    main()
