import sys,numpy as np
sys.path.insert(0,'scripts')
import ball_closed_loop as B
from smoothness_report import load
bag,dist,truth=sys.argv[1:4]
rec=B.Recording(bag,dist); T=np.load(truth)
ps=B.find_passes(rec,(T['t'],T['ok'],T['p']))
safe,nom,js=load(bag)
ts=np.array([s[0] for s in safe]);Qs=np.array([s[1] for s in safe]);tn=np.array([s[0] for s in nom]);Qn=np.array([s[1] for s in nom])
inp=np.zeros(len(ts),bool)
for p in ps: inp|=(ts>p['tc']-0.8)&(ts<p['tc']+1.0)
big=np.abs(Qs).max(1)>=5.9
dev=np.linalg.norm(Qs-Qn[np.clip(np.searchsorted(tn,ts)-1,0,len(tn)-1)],axis=1)
print(bag,'passes',len(ps),'ticks in pass windows %.0f%%'%(100*inp.mean()))
print(' ticks with |q̈|>=5.9: in-pass %d (%.1f%% of in-pass ticks), out-of-pass %d (%.2f%% of out ticks)'%(big[inp].sum(),100*big[inp].mean(),big[~inp].sum(),100*big[~inp].mean()))
print(' active (dev>1) in-pass %.1f%%  out-of-pass %.1f%%'%(100*(dev[inp]>1).mean(),100*(dev[~inp]>1).mean()))
jerk=np.abs(np.diff(Qs,axis=0)).max(1)/0.01
print(' jerk>=300: in-pass %d, out-of-pass %d'%((jerk[inp[1:]]>=300).sum(),(jerk[~inp[1:]]>=300).sum()))
# out-of-pass saturated events: group into bursts
idx=np.nonzero(big&~inp)[0]
bursts=[];
for i in idx:
    if not bursts or ts[i]-bursts[-1][1]>0.3: bursts.append([ts[i],ts[i]])
    else: bursts[-1][1]=ts[i]
print(' out-of-pass saturation bursts:',len(bursts),'durations(s):',np.round([b[1]-b[0] for b in bursts],2)[:15])
