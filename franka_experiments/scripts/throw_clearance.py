"""Usage: BAG=<bag name> python3 scripts/throw_clearance.py <replay or live bag dir> <original bag dir>   (truth: rosbag/<BAG>_truth.npz)

Clearance gain: min_t |c(t)+Δx(t) − b(t)| − min_t |c(t) − b(t)| at the CP nearest the ball.
c(t): the CP as a point fixed on its link, moved by the RECORDED joint states; Δx(t): double integral of
J(q)·(q̈_safe − q̈_nom) from tc−0.35 s (the replayed filter's own deviation); b(t): colour-tracked ball."""
import sys,os; sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
import numpy as np, pinocchio as pin
from ball_throw_eval import read_bag, _stamp, passes, DIST, QDD_SAFE, QDD_NOM
from franka_experiments.utils.kinematics import build_urdf_no_hand
rep, orig = sys.argv[1], sys.argv[2]
T=np.load(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'rosbag',os.environ['BAG']+'_truth.npz')); tt,ok,P=T['t'],T['ok'],T['p']
model=pin.buildModelFromUrdf(build_urdf_no_hand()); data=model.createData()
K=[f'fr3_joint{i}' for i in range(1,8)]
rows=[];safe=[];nom=[]
for tp,recv,m in read_bag(rep,{DIST,QDD_SAFE,QDD_NOM}):
    if tp==DIST: rows.append((_stamp(m),[(l.robot_link_name,np.array([l.closest_point_robot.x,l.closest_point_robot.y,l.closest_point_robot.z])) for l in m.links if l.valid and l.zone!='predicted']))
    elif tp==QDD_SAFE: safe.append((recv,np.array(m.data[:7])))
    else: nom.append((recv,np.array(m.data[:7])))
js=[]
for tp,recv,m in read_bag(orig,{'/NS_1/franka/joint_states','/NS_1/joint_states'}):
    n2p=dict(zip(m.name,m.position))
    if all(k in n2p for k in K): js.append((_stamp(m) or recv,np.array([n2p[k] for k in K])))
js.sort(key=lambda x:x[0]); tj=np.array([x[0] for x in js]); Qj=np.array([x[1] for x in js])
ts=np.array([x[0] for x in safe]); Qs=np.array([x[1] for x in safe]); tn=np.array([x[0] for x in nom]); Qn=np.array([x[1] for x in nom])
rt=np.array([r[0] for r in rows]); ps=passes(tt,ok,P,rt,[np.array([x[1] for x in r[1]]) if r[1] else np.zeros((0,3)) for r in rows],0.6,1.5)
lat=0.044; gains=[]
def fk(q,fid):
    pin.computeJointJacobians(model,data,q); pin.updateFramePlacements(model,data); return data.oMf[fid]
for n,p in enumerate(ps,1):
    tc=p['tc']; k=np.searchsorted(rt,tc); links=rows[min(k,len(rows)-1)][1]
    pb=P[np.argmin(abs(tt-tc))]; link,pr=min(links,key=lambda x: np.linalg.norm(x[1]-pb)); fid=model.getFrameId(link)
    M=fk(Qj[min(np.searchsorted(tj,tc),len(Qj)-1)],fid); p_loc=M.inverse().act(pr)
    t0=tc-0.35; grid=np.arange(t0,tc+0.15,0.01); X=np.zeros(3); V=np.zeros(3); d0=[]; d1=[]
    for t in grid:
        q=Qj[min(np.searchsorted(tj,t),len(Qj)-1)]; Mt=fk(q,fid); c=Mt.act(p_loc)
        J=pin.getFrameJacobian(model,data,fid,pin.LOCAL_WORLD_ALIGNED); r=c-Mt.translation; Jp=J[:3]+np.cross(J[3:].T,r).T
        i=np.searchsorted(ts,t+lat)-1
        if i>=0:
            qn=Qn[max(np.searchsorted(tn,ts[i])-1,0)]; V+=Jp@(Qs[i]-qn)*0.01
        X+=V*0.01
        j=np.argmin(abs(tt-t))
        if ok[j] and abs(tt[j]-t)<0.02: d0.append(np.linalg.norm(c-P[j])); d1.append(np.linalg.norm(c+X-P[j]))
    if d0:
        g=min(d1)-min(d0); gains.append(g)
        print(f' pass {n}: v={p["v"]:.1f}  {link:9s} clearance recorded {min(d0)*100:5.1f} cm -> with this filter\'s command {min(d1)*100:5.1f} cm  (gain {g*100:+5.1f})')
print(f'  mean clearance gain {np.mean(gains)*100:+.1f} cm   (min {np.min(gains)*100:+.1f})')
