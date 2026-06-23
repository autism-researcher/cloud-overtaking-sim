# Second baseline + adaptiveness reframe — what changed in the IJAE paper

This summarizes the new baseline, the experiments, and the honest reframe that
was integrated into `IJAE_submission/IJAE_fullpaper_LaTeX/main.tex`.
All numbers below use the paper's production config (dt = 0.1 s, T = 600 s,
10 seeds; the asymmetry sweep uses 5 seeds).

## The second baseline: a fixed-time signal

A new `signal` policy was added to `cloud_overtaking_sim.py` (and a self-contained
`compare_baselines.py`). It emulates the portable work-zone traffic light actually
deployed at single-lane obstructions: a fixed green interval `G = 15 s` per
direction, separated by an all-red clearance `R = 6 s`. It batches same-direction
vehicles during green but cannot shape approach speed and is demand-blind. This is
a genuinely strong baseline, not a strawman.

## The key, honest finding

Under **balanced (symmetric) steady demand**, a well-tuned fixed signal matches or
beats the cloud method above low demand. At lambda = 0.30 veh/s/dir:

| Metric | Bridge | Fixed signal | Cloud (proposed) |
|---|---|---|---|
| Throughput Q [veh/h]       | 211 | **841** | 581 |
| Average waiting Wbar [s]   | 262 | **143** | 167 |
| P95 waiting W95 [s]        | 504 | **276** | 384 |
| Average travel Tt [s]      | 327 | **240** | 267 |

So the original "~2.8x throughput" headline only holds against the uncoordinated
bridge. Against a real signal, raw throughput is **not** where the method wins.

The method's genuine win is **adaptiveness**. A fixed signal's green split is set in
advance; real demand is usually unbalanced. Holding total demand fixed at 0.40
veh/s and increasing the imbalance, the signal's throughput falls (green wasted on
the light side) while the cloud's rises. The curves cross near a 70/30 split.

Representative asymmetric point (lambda+ = 0.40, lambda- = 0.08 veh/s):

| Metric | Bridge | Fixed signal | Cloud (proposed) |
|---|---|---|---|
| Throughput Q [veh/h]       | 209 | 698 | **863** |
| Average waiting Wbar [s]   | 227 | 100 | **69**  |
| P95 waiting W95 [s]        | 484 | 298 | **148** |

At a 95/5 split the cloud more than doubles the signal's throughput (1018 vs 490).

## The reframed contribution (now in the paper)

The method's value is **demand-adaptive, infrastructure-free, stop-free
coordination**: it beats a fixed signal under asymmetric and low demand, needs no
physical signal hardware, and never forces a full stop — while a well-tuned signal
remains competitive under balanced steady demand. This is honest and defensible,
and it pre-empts the most likely reviewer attack.

## What was edited in main.tex

- Abstract: reframed to the two-baseline comparison and adaptiveness (number-free).
- Contributions: added the fixed-signal benchmark and the asymmetric-demand finding.
- Section 4.2 "Baselines": added the fixed-time signal definition.
- New Section 5.3 "Comparison with a Fixed-Time Signal": Figs. fig_baseline_throughput,
  fig_baseline_waiting, Table (3-way at lambda = 0.30), honest prose.
- New Section 5.4 "Demand-Adaptive Behaviour under Asymmetric Demand":
  Fig. fig_asym_throughput, asymmetric table, crossover discussion.
- Discussion + Conclusion: reframed honestly.

Compiled cleanly to 10 pages, no undefined references.

## Reproduce

```bash
cd simulation
python compare_baselines.py     # 3-way comparison (fast config) -> out_baseline/
# production numbers in this brief come from run_multiseed-style runs (dt=0.1, T=600)
```

Signal knobs are `Cfg.sig_green`, `Cfg.sig_clear`. `simulate()` now accepts a dict
`{+1: lam_plus, -1: lam_minus}` for asymmetric demand.

## Optional next step

The one regime where the method loses is high *symmetric* demand. If you want to
win there too, upgrade the greedy reservation with adaptive batch sizing / slot
back-filling so it recovers saturation throughput. That is a real research step,
not just writing — happy to attempt it if you want.
