#!/bin/bash
# Launch the c1 batch (mk_c1.py configs: lower SAC target entropy) detached on the host.
# Same resume as prec_s2/prec_s3 (ON, +1.25M) and b3_ft_v10c (OFF, +1.5M): v10c 2.5M + relabelled v10c buffer.
# 8 GPU runs (~39 fps each, ≈13.5 GiB of the 20 GiB container cap) → leave ≤3 parallel eval jobs.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
V=franka_sim/models/sac_v10c/checkpoints
SAVE="--checkpoint-every-episodes 400 --save-replay-buffer --no-episode-onnx"
RUNS=${RUNS:-"c1_ctrl c1_te14 c1_te14_s2 c1_te21 c1_te21_s2 c1_te28 c1_off_te21 c1_off_te14"}
for v in $RUNS; do
  case $v in c1_off_*) N=1500000 ;; *) N=1250000 ;; esac
  A="--resume $V/sac_2500000_steps.zip --resume-buffer $V/replay_buffer_latest.pkl --relabel-buffer --total-timesteps $N"
  setsid nohup systemd-inhibit --what=sleep:idle --who=franka_sim --why="sac_$v training" \
    docker exec franka_ros2 bash -lc "cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
      python3 -u -m franka_sim.train --config franka_sim/config_$v.yaml --exp-name sac_$v $A $SAVE" \
    > runs/sac_${v}_train.log 2>&1 < /dev/null &
  sleep 20
done
setsid nohup bash -c 'sleep 900; while docker exec franka_ros2 pgrep -f "exp-name sac_c1_" >/dev/null 2>&1; do sleep 300; done;
  powerprofilesctl set power-saver; echo "$(date) c1 batch finished, power profile -> power-saver"' \
  > runs/c1_power_restore.log 2>&1 < /dev/null &
