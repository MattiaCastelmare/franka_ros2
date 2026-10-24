#!/bin/bash
# ens_soup9 on the held-out seeds whose episode changes with task.target_ik_check=true (traces/feas/changed_seeds.py)
cd /ros2_ws/src; export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all
one() { SEEDS=$1:1 python3 $T/test_traces.py ens sac_v11_ft_prec/checkpoints/sac_3500000_steps,sac_v11_ft_prec/checkpoints/sac_3750000_steps,sac_v11_ft_prec/checkpoints/sac_4000000_steps,sac_v11_ft_lr1e4/checkpoints/sac_3500000_steps,sac_v11_ft_lr1e4/checkpoints/sac_3750000_steps,sac_v11_ft_lr1e4/checkpoints/sac_4000000_steps,sac_v11_ft_lr1e4_s2/checkpoints/sac_3500000_steps,sac_v11_ft_lr1e4_s2/checkpoints/sac_3750000_steps,sac_v11_ft_lr1e4_s2/checkpoints/sac_4000000_steps franka_sim/models/sac_v10c/config.yaml $T/traces/feas_eval/ens_soup9_$2_$1.json obstacle.mode=$2 obstacle.static_fraction=0 task.target_ik_check=true 2>&1 | tail -1; }
for s in 3001 3015 4093 4119 4143 4226 4245; do one $s static & done; one 3094 sinusoidal & wait
