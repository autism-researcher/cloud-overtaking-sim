# Extended communication-robustness: extreme latency/loss + Gilbert-Elliott bursty loss.
import importlib.util, numpy as np, csv, os, time
from cloud_overtaking_sim import Cfg, idm_accel, Cloud, in_section
Cfg.dt=0.1; Cfg.T_warm=60.0; Cfg.T_sim=600.0
LAM=0.30

class V:
    __slots__=("vid","d","length","v0i","s","v","t_in","t_out","wait","reserved","t_s","D","done","cmd_at","last_cmd")
    def __init__(s,vid,d,length,v0i,t_in):
        s.vid=vid;s.d=d;s.length=length;s.v0i=v0i;s.s=0.0;s.v=v0i;s.t_in=t_in;s.t_out=None;s.wait=0.0
        s.reserved=False;s.t_s=None;s.D=None;s.done=False;s.cmd_at=None;s.last_cmd=None

def simulate_comm(lam, comm_delay=0.0, loss=0.0, ge=None, rng=None):
    # ge=None -> i.i.d. loss; else ge=(p_gb, p_bg): Gilbert-Elliott, BAD state drops all updates
    if rng is None: rng=np.random.default_rng(0)
    dt=Cfg.dt; n=int((Cfg.T_warm+Cfg.T_sim)/dt)
    next_arr={+1:rng.exponential(1/lam),-1:rng.exponential(1/lam)}
    active={+1:[],-1:[]}; finished=[]; cloud=Cloud(); vid=0; min_gap=1e9
    bad=False  # GE channel state
    for k in range(n):
        t=k*dt
        # advance GE channel one step (correlated outage shared by all vehicles)
        if ge is not None:
            p_gb,p_bg=ge
            if bad: bad = (rng.random() >= p_bg)
            else:   bad = (rng.random() < p_gb)
        for d in (+1,-1):
            while t>=next_arr[d]:
                length=rng.uniform(Cfg.len_min,Cfg.len_max)
                v0i=float(np.clip(Cfg.v0+rng.uniform(-Cfg.v0_spread,Cfg.v0_spread),Cfg.v_min,Cfg.v_max))
                nv=V(vid,d,length,v0i,t); rear=min((vv.s for vv in active[d]),default=Cfg.L)
                nv.s=min(0.0,rear-(length+Cfg.s0)); active[d].append(nv); vid+=1
                next_arr[d]+=rng.exponential(1/lam)
        occ=set()
        for d in (+1,-1):
            for v in active[d]:
                if in_section(v): occ.add(d)
        owner=next(iter(occ)) if len(occ)==1 else (0 if not occ else 99)
        for d in (+1,-1):
            lane=sorted(active[d],key=lambda x:x.s,reverse=True)
            for idx,v in enumerate(lane):
                leader=lane[idx-1] if idx>0 else None
                if (not v.reserved) and Cfg.S_prec<=v.s<Cfg.S_entry:
                    r_i=t+comm_delay+(Cfg.S_entry-v.s)/max(v.v,Cfg.v_min)
                    v.D=(Cfg.L_o+v.length)/Cfg.v_sec+Cfg.g_same
                    v.t_s=cloud.reserve(d,r_i,v.D); v.reserved=True; v.cmd_at=t+comm_delay
                cmd_ok=v.reserved and v.cmd_at is not None and t>=v.cmd_at
                # packet loss: GE bad-state drop, else i.i.d.
                if cmd_ok:
                    if ge is not None:
                        if bad: cmd_ok=False
                    elif loss>0 and rng.random()<loss:
                        cmd_ok=False
                v0_eff=v.v0i
                if cmd_ok and v.s<Cfg.S_entry and v.t_s is not None:
                    dist=Cfg.S_entry-v.s; rem=v.t_s-t
                    v0_eff=float(np.clip(dist/rem,Cfg.v_min,Cfg.v_max)) if rem>0.1 else v.v0i
                    v.last_cmd=v0_eff
                elif v.reserved and (not cmd_ok) and v.last_cmd is not None and v.s<Cfg.S_entry:
                    v0_eff=v.last_cmd
                if Cfg.S_entry-5<=v.s<=Cfg.S_exit: v0_eff=min(v0_eff,Cfg.v_sec)
                if leader is not None: gap=leader.s-leader.length-v.s; dv=v.v-leader.v
                else: gap=1e6; dv=0.0
                allow=True
                if v.s<Cfg.S_entry:
                    opp_in=(len(occ)==1 and owner==-d)
                    granted=(v.reserved and v.cmd_at is not None and t>=v.cmd_at and t>=(v.t_s or t))
                    allow=granted and (not opp_in)
                if (not allow) and v.s<Cfg.S_entry:
                    slg=Cfg.S_entry-v.s
                    if slg<gap: gap=slg; dv=v.v
                a=idm_accel(v.v,v0_eff,gap,dv); v.v=max(0.0,v.v+a*dt); v.s+=v.v*dt
                if leader is not None:
                    rear=leader.s-leader.length
                    if v.s>rear: v.s=rear; v.v=min(v.v,leader.v)
                if t>=Cfg.T_warm and v.v<Cfg.v_wait_th and v.s<Cfg.S_exit: v.wait+=dt
                if v.s>=Cfg.L and not v.done: v.done=True; v.t_out=t
        for d in (+1,-1):
            ln=sorted(active[d],key=lambda x:x.s,reverse=True)
            for i in range(1,len(ln)):
                g=ln[i-1].s-ln[i-1].length-ln[i].s
                if g<min_gap: min_gap=g
        for d in (+1,-1):
            keep=[]
            for v in active[d]: (finished if v.done else keep).append(v)
            active[d]=keep
    T_total=Cfg.T_warm+Cfg.T_sim
    meas=[v for v in finished if v.t_out is not None and Cfg.T_warm<=v.t_out<=T_total]
    waits=np.array([v.wait for v in meas]) if meas else np.array([0.0])
    return dict(Q=len(meas)/Cfg.T_sim*3600.0, wait=float(np.mean(waits)), min_gap=min_gap)

def agg(delay,loss,ge,seeds=4):
    Q=[];W=[];mg=1e9
    for s in range(1,seeds+1):
        r=simulate_comm(LAM,delay,loss,ge,rng=np.random.default_rng(s))
        Q.append(r["Q"]);W.append(r["wait"]);mg=min(mg,r["min_gap"])
    return float(np.mean(Q)),float(np.std(Q)),float(np.mean(W)),mg

# experiment cells
cells=[]
for tau in [0.0,1.0,2.0,3.0]: cells.append((f"lat{int(tau*1000)}",tau,0.0,None))
for p in [0.4,0.8]:           cells.append((f"loss{int(p*100)}",0.10,p,None))
# Gilbert-Elliott bursty loss ~20% mean: p_gb=0.25*p_bg; avg burst = (1/p_bg)*dt seconds
cells.append(("ge_burst1s", 0.10, 0.0, (0.25/10, 1/10)))   # avg burst ~1.0 s
cells.append(("ge_burst5s", 0.10, 0.0, (0.25/50, 1/50)))   # avg burst ~5.0 s

OUT="comm_ext.csv"
done=set()
if os.path.exists(OUT):
    for r in csv.reader(open(OUT)):
        if r and r[0]!="cell": done.add(r[0])
else:
    open(OUT,"w").write("cell,latency_ms,loss_pct,ge,Q_mean,Q_std,W_mean,min_gap,collision\n")
t0=time.time()
for tag,tau,loss,ge in cells:
    if tag in done: continue
    if time.time()-t0>38: print("budget; resume",flush=True); break
    Qm,Qs,Wm,mg=agg(tau,loss,ge)
    coll="yes" if mg<-0.1 else "no"
    with open(OUT,"a",newline="") as f:
        csv.writer(f).writerow([tag,int(tau*1000),f"{loss*100:.0f}",("burst" if ge else "iid"),
            f"{Qm:.1f}",f"{Qs:.1f}",f"{Wm:.1f}",f"{mg:.2f}",coll])
    print(f"  {tag}: Q={Qm:.0f}±{Qs:.0f} W={Wm:.0f} min_gap={mg:.2f} coll={coll} ({time.time()-t0:.0f}s)",flush=True)
left=[t for (t,_,_,_) in cells if t not in {r[0] for r in csv.reader(open(OUT)) if r and r[0]!='cell'}]
print("REMAINING:",len(left),flush=True)
