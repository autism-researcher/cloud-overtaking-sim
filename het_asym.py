import os, csv, numpy as np, het_sim as H
H.Cfg.dt=0.1; H.Cfg.T_warm=60.0; H.Cfg.T_sim=600.0
H.Cfg.het_drivers=True; H.Cfg.truck_frac=0.15
TOTAL=0.16; SHARES=[0.50,0.60,0.70,0.80,0.90,0.95]; POLS=["base","signal","actuated","cloud"]; SEEDS=[1,2,3,4,5]
C="out_real/het_asym.csv"
def load():
    d={}
    if os.path.exists(C):
        for r in csv.DictReader(open(C)): d[(float(r["share"]),r["pol"],int(r["seed"]))]=r
    return d
def app(k,r):
    new=not os.path.exists(C)
    with open(C,"a",newline="") as f:
        w=csv.writer(f)
        if new: w.writerow(["share","pol","seed","Q","Wh","Wl","stops"])
        w.writerow([k[0],k[1],k[2],f"{r['Q']:.2f}",f"{r['wait_pos']:.2f}",f"{r['wait_neg']:.2f}",f"{r['stops_mean']:.3f}"])
have=load(); sel=os.environ.get("SHARES"); shs=[float(x) for x in sel.split(",")] if sel else SHARES
for sh in shs:
    lam={+1:TOTAL*sh,-1:TOTAL*(1-sh)}
    for p in POLS:
        for s in SEEDS:
            k=(sh,p,s)
            if k in have: continue
            r=H.simulate(lam=lam,policy=p,rng=np.random.default_rng(s))
            app(k,r); print(f"  {sh:.2f} {p:6s} s{s}: Q={r['Q']:.0f}",flush=True)
