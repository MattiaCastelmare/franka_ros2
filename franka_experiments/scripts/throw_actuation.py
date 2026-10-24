"""Actuation check on a live throw bag: commanded q̈_safe vs the q̈ the joints realised
(lag, gain per joint), accel-box saturation per pass, and velocity change achieved vs commanded.
Usage: python3 scripts/throw_actuation.py <live bag dir> <truth .npz>
"""
import sys,os; sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from ball_throw_eval import read_bag, _stamp, passes, DIST, QDD_SAFE, QDD_NOM
import numpy as np
bag, truth = sys.argv[1], sys.argv[2]
T=np.load(truth); tt,ok,P=T['t'],T['ok'],T['p']
rows=[]; safe=[]; nom=[]; js=[]
for tp,recv,m in read_bag(bag,{DIST,QDD_SAFE,QDD_NOM,'/NS_1/franka/joint_states'}):
    if tp==DIST: rows.append((_stamp(m),np.array([[l.closest_point_robot.x,l.closest_point_robot.y,l.closest_point_robot.z] for l in m.links if l.valid])))
    elif tp==QDD_SAFE: safe.append((recv,np.array(m.data[:7])))
    elif tp==QDD_NOM: nom.append((recv,np.array(m.data[:7])))
    else:
        n2v=dict(zip(m.name,m.velocity)); n2p=dict(zip(m.name,m.position)); K=[f'fr3_joint{i}' for i in range(1,8)]
        js.append((_stamp(m) if m.header.stamp.sec else recv, np.array([n2v[k] for k in K]), np.array([n2p[k] for k in K])))
ts=np.array([x[0] for x in safe]); Qs=np.array([x[1] for x in safe]); tn=np.array([x[0] for x in nom]); Qn=np.array([x[1] for x in nom])
tj=np.array([x[0] for x in js]); V=np.array([x[1] for x in js])
# realized accel: LS slope over +-10 ms
from numpy.lib.stride_tricks import sliding_window_view
w=21; tw=sliding_window_view(tj,w); Vw=sliding_window_view(V,(w,7))[:,0]
tc=tw.mean(1); A=np.array([np.polyfit(tw[i]-tc[i],Vw[i],1)[0] for i in range(0,len(tw),5)]); tA=tc[::5]
np.savez('/tmp/act.npz',tA=tA,A=A,ts=ts,Qs=Qs,tn=tn,Qn=Qn)
# lag: commanded (ZOH) vs realized, per joint, over whole run
lags=np.arange(0,0.101,0.002); res=[]
for j in range(7):
    best=None
    for L in lags:
        cmd=Qs[np.clip(np.searchsorted(ts,tA-L)-1,0,len(ts)-1),j]
        m=np.abs(cmd)>0.5
        if m.sum()<100: continue
        e=np.mean((A[m,j]-cmd[m])**2)
        g=np.dot(A[m,j],cmd[m])/np.dot(cmd[m],cmd[m])
        if best is None or e<best[1]: best=(L,e,g,np.sqrt(np.mean(cmd[m]**2)))
    res.append(best)
print('per joint: best lag commanded->realized, realized/commanded gain, rms cmd')
for j,b in enumerate(res):
    if b: print(f'  j{j+1}: lag {b[0]*1e3:4.0f} ms  gain {b[2]:.2f}  rms err {np.sqrt(b[1]):.2f}  rms cmd {b[3]:.2f} rad/s2')
box=np.array([6,2.585,3.5,4.0,10,5.5,10])
rt=np.array([r[0] for r in rows]); ps=passes(tt,ok,P,rt,[r[1] for r in rows],0.6,1.5)
lat=0.044
print('\nper pass (t relative to closest approach, receive clock):')
for n,p in enumerate(ps,1):
    tcl=p['tc']+lat
    w=(ts>tcl-0.35)&(ts<tcl+0.05)
    dev=np.linalg.norm(Qs[w]-Qn[np.clip(np.searchsorted(tn,ts[w])-1,0,None)],axis=1)
    sat=(np.abs(Qs[w])>=0.97*box).any(1)
    wa=(tA>tcl-0.35)&(tA<tcl+0.05)
    # realized deviation: realized accel minus nominal
    nomA=Qn[np.clip(np.searchsorted(tn,tA[wa])-1,0,None)]
    rdev=np.linalg.norm(A[wa]-nomA,axis=1)
    def first(t,x,thr): i=np.nonzero(x>thr)[0]; return (tcl-t[i[0]])*1e3 if len(i) else np.nan
    print(f' {n}: cmd dev>5 at {first(ts[w],dev,5):4.0f} ms before | realized dev>5 at {first(tA[wa],rdev,5):4.0f} ms before | ticks at accel box {sat.sum()}/{w.sum()} | joints saturated: {sorted(set(np.nonzero((np.abs(Qs[w])>=0.97*box))[1]+1))}')

print('\nvelocity change achieved vs commanded, from CBF onset to closest approach +50 ms  [rad/s]:')
tjv=tj
for n,p in enumerate(ps,1):
    tcl=p['tc']+lat
    w=(ts>tcl-0.35)&(ts<tcl+0.05)
    dev=np.linalg.norm(Qs[w]-Qn[np.clip(np.searchsorted(tn,ts[w])-1,0,None)],axis=1)
    i=np.nonzero(dev>5)[0]
    if not len(i): continue
    t0=ts[w][i[0]]; t1=tcl+0.05
    ww=(ts>=t0)&(ts<t1)
    dq_cmd=(Qs[ww]*0.01).sum(0)
    v0=V[np.searchsorted(tjv,t0)]; v1=V[min(np.searchsorted(tjv,t1),len(V)-1)]
    dq_real=v1-v0
    # nominal-only would have given:
    dq_nom=(Qn[np.clip(np.searchsorted(tn,ts[ww])-1,0,None)]*0.01).sum(0)
    print(f' {n} ({(t1-t0)*1e3:.0f} ms): cmd {np.round(dq_cmd,2)}\n{"":14s}real {np.round(dq_real,2)}\n{"":14s}nom  {np.round(dq_nom,2)}')
