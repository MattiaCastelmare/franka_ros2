import yaml,glob,os
def flat(d,p=''):
    o={}
    for k,v in d.items():
        if isinstance(v,dict): o.update(flat(v,p+k+'.'))
        else: o[p+k]=v
    return o
cfgs={os.path.basename(os.path.dirname(f)):flat(yaml.safe_load(open(f))) for f in sorted(glob.glob('franka_sim/models/sac_*/config.yaml'))}
allk=set().union(*[c.keys() for c in cfgs.values()])
for k in sorted(allk):
    if k.startswith(('sac.','train','reward.w_')): continue
    vals=[str(cfgs[r].get(k,'-'))[:9] for r in cfgs]
    if len(set(vals))>1: print(f"{k:34s}"+' '.join(f"{v:>9s}" for v in vals))
print(' '*34+' '.join(f"{r[4:]:>9s}" for r in cfgs))
