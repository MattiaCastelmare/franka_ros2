import sys,numpy as np
sys.path.insert(0,'scripts')
import ball_closed_loop as B
bag,dist,truth=sys.argv[1:4]
rec=B.Recording(bag,dist); T=np.load(truth); tt,ok,P=T['t'],T['ok'],T['p']
ps=B.find_passes(rec,(tt,ok,P)); out=[]
for n,p in enumerate(ps,1):
    tc=p['tc']; fr=ft=None; dfirst=None
    for tcap,rv,m in rec.dist:
        if not (tc-1.5<tcap<tc+0.05): continue
        b=B._ball_at(tt,ok,P,tcap)
        if b is None: continue
        rows=[l for l in m.links if l.valid and np.linalg.norm([l.closest_point_human.x-b[0],l.closest_point_human.y-b[1],l.closest_point_human.z-b[2]])<0.15]
        if rows:
            if fr is None: fr=tcap; dfirst=min(l.distance for l in rows)
            if ft is None and any(l.frames_seen>=3 for l in rows): ft=tcap
    out.append((tc-fr if fr else np.nan, tc-ft if ft else np.nan, dfirst if dfirst else np.nan))
o=np.array(out,float)
print(f'{dist.split("/")[-1]:32s} first row lead median {np.nanmedian(o[:,0]):.2f}s  track>=3 {np.nanmedian(o[:,1]):.2f}s  d at first row {np.nanmedian(o[:,2]):.2f} m  per-pass track lead {np.round(o[:,1],2)}')
