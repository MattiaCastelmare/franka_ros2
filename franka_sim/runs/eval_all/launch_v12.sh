#!/bin/bash
# Launch the v12 batch (mk_v12.py configs) detached on the host.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
V=franka_sim/models/sac_v10c/checkpoints; PR=franka_sim/models/sac_v11_ft_prec/checkpoints
FROM_V10C="--resume $V/sac_2500000_steps.zip --resume-buffer $V/replay_buffer_latest.pkl --relabel-buffer --total-timesteps 1250000"
FROM_PREC="--resume $PR/sac_3750000_steps.zip --policy-warmup 20000 --total-timesteps 1000000"
SAVE="--checkpoint-every-episodes 400 --save-replay-buffer --no-episode-onnx"
for v in ${RUNS:-v12_prec_s2 v12_prec_s3 v12_cont v12_hard v12_slack v12_hard_slack v12_lr3e5 v12_prec4}; do
  case $v in v12_prec_s*) A="$FROM_V10C" ;; *) A="$FROM_PREC" ;; esac
  setsid nohup systemd-inhibit --what=sleep:idle --who=franka_sim --why="sac_$v training" \
    docker exec franka_ros2 bash -lc "cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
      python3 -u -m franka_sim.train --config franka_sim/config_$v.yaml --exp-name sac_$v $A $SAVE" \
    > runs/sac_${v}_train.log 2>&1 < /dev/null &
  sleep 2
done
