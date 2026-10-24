"""Weight-averaged SAC policies (checkpoint averaging / model soup) -> models/sac_v11_avg/<name>.zip.

All sources are fine-tunes of the same parent (sac_v10c 2.5M), so their weights live in one basin and a
uniform average of the full policy state_dict (actor + critics) is meaningful. Only the actor is used at test time.
"""
import torch
from stable_baselines3 import SAC
M = 'franka_sim/models'
ck = lambda run, s: f'{M}/{run}/checkpoints/sac_{s}_steps.zip'
P = 'sac_v11_ft_prec'
SOUPS = {
    'prec_3.5-4.0_k3':   [ck(P, s) for s in (3500000, 3750000, 4000000)],
    'prec_3.25-4.0_k4':  [ck(P, s) for s in (3250000, 3500000, 3750000, 4000000)],
    'prec_3.5-4.0_k11':  [ck(P, s) for s in range(3500000, 4000001, 50000)],
    'prec_3.65-3.85_k5': [ck(P, s) for s in range(3650000, 3850001, 50000)],
    'soup3_3.75':        [ck(r, 3750000) for r in (P, 'sac_v11_ft_lr1e4', 'sac_v11_ft_lr1e4_s2')],
    'soup3_3.5-4.0_k9':  [ck(r, s) for r in (P, 'sac_v11_ft_lr1e4', 'sac_v11_ft_lr1e4_s2') for s in (3500000, 3750000, 4000000)],
    'lr1e4_3.5-4.0_k3':  [ck('sac_v11_ft_lr1e4', s) for s in (3500000, 3750000, 4000000)],
    'lr1e4s2_3.5-4.0_k3': [ck('sac_v11_ft_lr1e4_s2', s) for s in (3500000, 3750000, 4000000)],
}
import os, sys
torch.set_num_threads(1)
ONLY = sys.argv[1:]
for name, srcs in SOUPS.items():
    if ONLY and name not in ONLY: continue
    base = SAC.load(srcs[0], device='cpu')
    sds = [SAC.load(s, device='cpu').policy.state_dict() for s in srcs]
    avg = {k: (torch.stack([sd[k].float() for sd in sds]).mean(0).to(sds[0][k].dtype)
               if sds[0][k].is_floating_point() else sds[0][k]) for k in sds[0]}
    base.policy.load_state_dict(avg)
    base.save(f'{M}/sac_v11_avg/{name}.zip')
    print(name, len(srcs), 'sources')
