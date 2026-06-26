# Cloud-Assisted Obstacle-Overtaking Simulator

Microscopic traffic simulator that generates all the quantitative results
(figures + tables) for the associated IEEE Access paper.

## What it models
A one-lane two-way road with a localized obstacle. The obstacle blocks one
direction over a short stretch (the **shared section**), so vehicles from both
directions contend for the same pavement — only one direction may occupy the
section at a time (same-direction vehicles may follow as a *batch*).

Longitudinal motion uses the **Intelligent Driver Model (IDM)**. Four policies
are compared (in `het_sim.py`, the heterogeneous-fleet production simulator):

| Policy | Description |
|--------|-------------|
| `base`   | Decentralized one-lane-bridge rule: one vehicle at a time, right-of-way alternates between directions. A vehicle *claims* the section when granted, so opposing vehicles cannot enter during its approach (no head-on deadlock). |
| `signal` | Idealized fixed-time work-zone signal: green alternates on a fixed cycle (`sig_green`) with an all-red clearance (`sig_clear`); vehicles stop at red, no en-route speed shaping. |
| `actuated` | Idealized vehicle-actuated work-zone signal: green extends while the served direction has demand (up to `act_gmax`) and gaps out when its queue clears (min green `act_gmin`), same all-red clearance. Modelled with **no startup lost time** — a deliberately strong, best-case roadside comparator. |
| `cloud`  | **Proposed.** Centralized slot reservation + IDM-compliant target speed (vehicles are slowed *en route* to arrive exactly when their slot opens, avoiding full stops) + same-direction batching with a hard cap (`batch_cap = 10` consecutive cars, then a forced gap yielding to any waiting opposing traffic — "10 cars, gap, 10 cars") so neither direction starves. |

**Safety interlock.** All policies enforce strict directional mutual exclusion via a hard
section-ownership interlock: a vehicle may cross the section entry only when no opposing
vehicle is physically in the section or within its opposing safety clearance (`g_opp`). The
simulator is instrumented (`SAFETY_VIOL`) to flag any time step in which both directions
occupy the section; across all reported runs this count is **zero** (verified conflict-free).
The default fleet is **heterogeneous** (`het_drivers=True`: per-vehicle headway/accel/brake,
15% heavy vehicles via `truck_frac`). A `comm_delay`/`comm_loss` model and a `bursty` demand
mode are included for the robustness studies.

## Run
```bash
pip install numpy matplotlib
python cloud_overtaking_sim.py
```
Runtime is ~30 s. All outputs are written to `./out/`.

To reproduce the paper's figures and tables (all four policies, heterogeneous fleet,
realistic demand, final config `dt = 0.1`, `T_sim = 600`), use the heterogeneous runners:
```bash
python het_study.py    # symmetric density sweep, base/signal/actuated/cloud, 10 seeds -> out_real/het_sweep.csv
python het_asym.py     # total throughput + per-direction fairness vs heavy-direction share, 5 seeds -> out_real/het_asym.csv
python het_plots.py    # throughput/waiting sweep + asymmetric figures from the CSVs
```
Both runners are resumable and accept a chunk via env var (`LAMS=0.10,0.12 python het_study.py`,
`SHARES=0.80 python het_asym.py`).

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

## Representative results (heterogeneous fleet, dt = 0.1 s, T_sim = 600 s, 10 seeds; balanced λ = 0.10 veh/s/dir)
| Metric | Bridge | Fixed signal | Actuated | Cloud |
|--------|--------|--------------|----------|-------|
| Throughput Q [veh/h]       | 203 | 611 | 669 | 674 |
| Average waiting W̄ [s]      | 198 | 45  | 31  | 16  |
| P95 waiting W95 [s]        | 387 | 110 | 76  | 45  |
| Full stops per vehicle     | 5.0 | 1.8 | 1.1 | 0.7 |

The corridor is operated at **realistic demand** (λ ≤ 0.14, where traffic flows rather than
gridlocks), not at oversaturation. All four policies are conflict-free; the cloud matches the
idealized actuated signal on throughput while cutting waiting and stops, and leads it under
asymmetric demand (see `het_asym.py`).

## Reproducibility
Fixed `seed` makes every run deterministic. Increase `T_sim` and average over
seeds before quoting final numbers in the journal version.
