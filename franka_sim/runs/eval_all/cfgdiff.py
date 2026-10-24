import sys, yaml
def flat(d, p=''):
    o = {}
    for k, v in d.items():
        if isinstance(v, dict): o.update(flat(v, p + k + '.'))
        else: o[p + k] = v
    return o
a = flat(yaml.safe_load(open(sys.argv[1]))); b = flat(yaml.safe_load(open(sys.argv[2])))
for k in sorted(set(a) | set(b)):
    if k.startswith(('shield_parity', 'randomization')): continue
    va, vb = a.get(k, '<absent>'), b.get(k, '<absent>')
    if va != vb: print(f'{k:38s} A={str(va)[:38]:40s} B={str(vb)[:38]}')
