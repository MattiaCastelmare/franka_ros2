import yaml
for sc in ('static', 'dynamic'):
    c = yaml.safe_load(open('franka_sim/runs/eval_all/sac_v4_hard.yaml'))
    c['task'].update(blocking_fraction=0.6, target_clearance=0.26, target_free_always=True, reset_max_tries=3000)
    c['cbf'].update(floor_enable=True, soft_fallback=True)
    c['obstacle']['mode'] = 'static' if sc == 'static' else 'sinusoidal'
    yaml.safe_dump(c, open(f'franka_sim/runs/eval_all/sac_v4_{sc}.yaml', 'w'))
