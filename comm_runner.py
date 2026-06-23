#!/usr/bin/env python3
import os, json, time
import numpy as np
from multiprocessing import Pool
from cloud_overtaking_sim import Cfg
import comm_delay_experiment as e
Cfg.dt=0.1; Cfg.T_warm=60.0; Cfg.T_sim=600.0
LAM=0.30; N_SEEDS=10; BASE=1
LAT=[0.0,0.05,0.10,0.20,0.50]; LOSS=[0.0,0.05,0.10,0.20]; LOSS_LAT=0.10
CKPT="ckpt.json"
def one(args):
    tau,loss,seed=args
    r=e.simulate_comm(LAM,tau,loss,rng=np.random.default_rng(seed))
    return (tau,loss,seed,r["Q"],r["wait"],r["min_gap"])
def load():
    return json.load(open(CKPT)) if os.path.exists(CKPT) else {}
def save(d): json.dump(d,open(CKPT,"w"))
def key(tau,loss): return f"{tau}|{loss}"
def main():
    pts=[("lat",tau,0.0) for tau in LAT]+[("loss",LOSS_LAT,p) for p in LOSS]
    done=load(); t0=time.time()
    for kind,tau,loss in pts:
        k=key(tau,loss)
        if k in done: continue
        if time.time()-t0>22:  # leave headroom under 45s
            print("TIME_BUDGET_EXIT pending"); break
        jobs=[(tau,loss,BASE+s) for s in range(N_SEEDS)]
        with Pool(min(N_SEEDS,os.cpu_count())) as pool:
            res=pool.map(one,jobs)
        Q=[r[3] for r in res]; W=[r[4] for r in res]; mg=min(r[5] for r in res)
        done[k]=dict(kind=kind,tau=tau,loss=loss,
                     Qm=float(np.mean(Q)),Qs=float(np.std(Q)),
                     Wm=float(np.mean(W)),Ws=float(np.std(W)),min_gap=float(mg))
        save(done)
        print(f"done {k}: Q={np.mean(Q):.1f}+-{np.std(Q):.1f} W={np.mean(W):.1f}+-{np.std(W):.1f} mg={mg:.3f}")
    rem=[key(tau,loss) for kind,tau,loss in pts if key(tau,loss) not in done]
    print("REMAINING", len(rem), rem)
if __name__=="__main__": main()
