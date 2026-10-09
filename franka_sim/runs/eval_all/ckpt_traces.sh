#!/bin/bash
# Learning-stage test: each run's step checkpoints on the dynamic scenario, seeds 1000..1019 (test_traces.py).
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 SEEDS=1000:20
T=franka_sim/runs/eval_all
for r in sac_v10 sac_v10_obs51 sac_v10_g997; do for s in 250000 500000 750000 1000000 1250000 1500000 1750000 2000000; do
  echo "$r checkpoints/sac_${s}_steps franka_sim/models/$r/config.yaml $T/traces/ckpt/${r}__$s.json"; done; done |
xargs -P 12 -L 1 sh -c 'python3 franka_sim/runs/eval_all/test_traces.py $0 $1 $2 $3 obstacle.mode=sinusoidal obstacle.static_fraction=0 2>&1 | tail -1'
