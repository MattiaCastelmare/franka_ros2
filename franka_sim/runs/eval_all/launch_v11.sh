#!/bin/bash
# Launch the v11 batch (mk_v11.py configs) detached on the host: 7 fine-tunes from sac_v10c 2.5M + 1 from scratch.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
M=franka_sim/models/sac_v10c/checkpoints
RES="--resume $M/sac_2500000_steps.zip --resume-buffer $M/replay_buffer_latest.pkl --relabel-buffer --total-timesteps 1500000"
for v in v11_ft_lr1e4 v11_ft_lr1e4_s2 v11_ft_prec v11_ft_static v11_ft_obspot v11_ft_combo v11_ft_utd2 v11_scratch_prec; do
  case $v in v11_scratch*) A="--total-timesteps 2000000" ;; *) A="$RES" ;; esac
  setsid nohup systemd-inhibit --what=sleep:idle --who=franka_sim --why="sac_$v training" \
    docker exec franka_ros2 bash -lc "cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
      python3 -u -m franka_sim.train --config franka_sim/config_$v.yaml --exp-name sac_$v $A --checkpoint-every-episodes 0" \
    > runs/sac_${v}_train.log 2>&1 < /dev/null &
  sleep 2
done
