#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regenerate the two space-time diagrams with larger fonts/legends at 600 dpi,
writing PDF (vector) + PNG (600 dpi) straight into the IJAE LaTeX folder."""
import os, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from cloud_overtaking_sim import Cfg, simulate

# match the paper's production config
Cfg.dt = 0.1; Cfg.T_warm = 60.0; Cfg.T_sim = 600.0
SEED = 1
OUT = os.environ.get("FIGOUT", ".")
C_CLOUD, C_BASE = "#1f5fbf", "#c0392b"


def spacetime(policy, title, fname, t0, t1):
    r = simulate(0.30, policy, record_tracks=True,
                 rng=np.random.default_rng(SEED))
    fig, ax = plt.subplots(figsize=(5.4, 3.7))
    for _, (ts, ss, d) in r["tracks"].items():
        if not ts:
            continue
        ts = np.array(ts); ss = np.array(ss)
        m = (ts >= t0) & (ts <= t1)
        if m.sum() < 2:
            continue
        ax.plot(ts[m], ss[m], color=(C_CLOUD if d == +1 else C_BASE),
                lw=1.0, alpha=0.9)
    ax.axhspan(Cfg.S_entry, Cfg.S_exit, color="0.75", alpha=0.6, zorder=0)
    ax.axhline(Cfg.S_entry, color="k", ls="--", lw=0.8)
    ax.set_xlim(t0, t1); ax.set_ylim(0, Cfg.L)
    ax.set_xlabel("time [s]", fontsize=15)
    ax.set_ylabel("position along corridor [m]", fontsize=15)
    ax.tick_params(axis="both", labelsize=13)
    ax.set_title(title, fontsize=15)
    leg = ax.legend([Line2D([0], [0], color=C_CLOUD, lw=3),
                     Line2D([0], [0], color=C_BASE, lw=3)],
                    ["direction L$\\rightarrow$R", "direction R$\\rightarrow$L"],
                    fontsize=13, loc="lower right", handlelength=2.2,
                    framealpha=0.9, borderpad=0.5)
    leg.get_frame().set_edgecolor("0.6")
    fig.tight_layout()
    fig.savefig(f"{OUT}/{fname}.pdf", dpi=600, bbox_inches="tight")
    fig.savefig(f"{OUT}/{fname}.png", dpi=600, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}/{fname}.pdf/.png  (tracks={len(r['tracks'])})")


def main():
    t0, t1 = Cfg.T_warm, Cfg.T_warm + 180
    spacetime("cloud", "Proposed cloud-assisted coordination",
              "fig_spacetime_cloud", t0, t1)
    spacetime("base", "Decentralized baseline",
              "fig_spacetime_base", t0, t1)


if __name__ == "__main__":
    main()
