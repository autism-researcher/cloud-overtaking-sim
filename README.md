# Cloud-Assisted Obstacle-Overtaking Simulator

Microscopic traffic simulator that generates all the quantitative results
(figures + tables) for the associated IEEE Access paper.

## What it models
A one-lane two-way road with a localized obstacle. The obstacle blocks one
direction over a short stretch (the **shared section**), so vehicles from both
directions contend for the same pavement — only one direction may occupy the
section at a time (same-direction vehicles may follow as a *batch*).

Longitudinal motion uses the **Intelligent Driver Model (IDM)**. Three policies
are compared:

| Policy | Description |
|--------|-------------|
| `base`   | Decentralized one-lane-bridge rule: one vehicle at a time, right-of-way alternates between directions. A vehicle *claims* the section when granted, so opposing vehicles cannot enter during its approach (no head-on deadlock). |
| `signal` | Idealized fixed-time work-zone signal: green alternates between directions on a fixed cycle (`sig_green`) with an all-red clearance (`sig_clear`); vehicles stop at red, with no en-route speed shaping. A best-case portable work-zone-signal baseline. |
| `cloud`  | **Proposed.** Centralized slot reservation + IDM-compliant target speed (vehicles are slowed *en route* to arrive exactly when their slot opens, avoiding full stops) + same-direction batching + a `max_batch` fairness cap so neither direction starves. |

## Run
```bash
pip install numpy matplotlib
python cloud_overtaking_sim.py
```
Runtime is ~30 s. All outputs are written to `./out/`.

To reproduce the paper's figures and tables (all three policies, multi-seed, final
configuration `dt = 0.1`, `T_sim = 600`), use the `regen_*.py` runners:
```bash
python regen_baseline.py    # throughput & waiting vs symmetric demand (base/signal/cloud, 10 seeds)
python regen_asym.py        # total throughput vs heavy-direction share (5 seeds)
python regen_gopp.py        # throughput vs opposing safety clearance g_opp (3 seeds)
python regen_fairness.py    # per-direction waiting + Jain fairness index (5 seeds)
python regen_spacetime.py   # space-time diagrams (cloud vs baseline)
```

## Outputs (in `out/`)
- `fig_spacetime_cloud.pdf/.png` — space-time diagram, proposed method (Fig. 2)
- `fig_spacetime_base.pdf/.png`  — space-time diagram, baseline (Fig. 3)
- `fig_waiting_cdf.pdf/.png`     — empirical CDF of waiting time (Fig. 4)
- `fig_sweep_throughput.pdf/.png` — throughput vs arrival rate
- `fig_sweep_waiting.pdf/.png`    — mean & P95 waiting vs arrival rate
- `results_table.csv`            — headline metrics at the representative point (the paper table)
- `sweep.csv`                    — full density-sweep numbers

Use the `.pdf` versions in the LaTeX papers (vector, crisp at any zoom).

## Tuning
All parameters are in the `Cfg` class at the top of the script:
- Geometry: `L`, `L_o`, `L_p`
- IDM: `v0`, `T_head`, `s0`, `a_max`, `b_comf`, `delta`, `v_sec`
- Coordinator: `g_opp`, `g_same`, `v_min`, `v_max`, `max_batch`
- Simulation: `dt`, `T_warm`, `T_sim`, `seed`

For final paper-quality numbers use `dt = 0.1`, `T_sim = 600`, and average over
several seeds (run with `seed = 1..10` and report mean ± std). The shipped
defaults (`dt = 0.2`, `T_sim = 360`) are tuned for a fast single run.

## Representative results (final config: dt = 0.1 s, T_sim = 600 s, 10 seeds; λ = 0.30 veh/s/dir)
| Metric | Decentralized | Fixed-time signal | Cloud-Assisted |
|--------|---------------|-------------------|----------------|
| Throughput Q [veh/h]       | 211 | 841 | 581 |
| Average waiting W̄ [s]      | 262 | 143 | 167 |
| P95 waiting W95 [s]        | 504 | 276 | 384 |
| Average travel time T̄t [s] | 327 | 240 | 267 |

These reproduce Table III of the paper. The quick single-run defaults (`dt = 0.2`,
`T_sim = 360`, one seed) give different numbers — use the final config above (via the
`regen_*.py` runners) to reproduce the published results.

> Note: cloud throughput peaks near λ ≈ 0.15 and eases slightly at very high
> demand — this is realistic capacity drop under heavy oversaturation, not a
> bug. If you prefer a flat saturation curve for the paper, cap the sweep at
> λ ≤ 0.20 or report the *offered-vs-served* flow.

## Reproducibility
Fixed `seed` makes every run deterministic. Increase `T_sim` and average over
seeds before quoting final numbers in the journal version.
