#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regenerate every data-driven figure in the paper from the fixed,
collision-free simulator, in one command.

    FIGOUT=../IEEE_Access_submission python3 regen_all_figures.py

Produces (into FIGOUT, default current dir):
    fig_spacetime_cloud.pdf / .png   (regen_spacetime.py)
    fig_spacetime_base.pdf  / .png
    fig_baseline_throughput.pdf/.png (regen_baseline.py)
    fig_baseline_waiting.pdf/.png
    fig_asym_throughput.pdf/.png     (regen_asym.py)

The communication figure (fig_comm) is produced by comm_delay_experiment.py,
whose simulator was already collision-free; rerun that script separately if you
want to refresh it (writes to out_comm/).

Note: the baseline and asymmetric sweeps take several minutes each.
"""
import os, sys, subprocess

FIGOUT = os.environ.get("FIGOUT", ".")
HERE = os.path.dirname(os.path.abspath(__file__))
env = dict(os.environ, FIGOUT=FIGOUT)


def run(script):
    print(f"\n==> {script}  (FIGOUT={FIGOUT})", flush=True)
    subprocess.run([sys.executable, os.path.join(HERE, script)], env=env, check=True)


if __name__ == "__main__":
    run("regen_spacetime.py")
    run("regen_baseline.py")
    run("regen_asym.py")
    print(f"\nAll space-time, baseline, and asymmetric figures written to {FIGOUT}.")
    print("To refresh the communication figure: python3 comm_delay_experiment.py")
