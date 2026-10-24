#!/bin/bash
# Launch the c2 batch (mk_c2.py configs) detached on the host.
# Same resume as prec_s2/prec_s3 (ON, +1.25M) and b3_ft_v10c (OFF, +1.5M): v10c 2.5M + relabelled v10c buffer.
# 8 GPU runs (~39 fps each, ≈13.5 GiB of the 20 GiB container cap) → leave ≤3 parallel eval jobs.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
V=franka_sim/models/sac_v10c/checkpoints
SAVE="--checkpoint-every-episodes 400 --save-replay-buffer --no-episode-onnx"
RUNS=${RUNS:-"c2_lr3e5 c2_bs2048 c2_off_s2 c2_off_s3"}
for v in $RUNS; do
  case $v in c2_off_*) N=1500000 ;; *) N=1250000 ;; esac
  A="--resume $V/sac_2500000_steps.zip --resume-buffer $V/replay_buffer_latest.pkl --relabel-buffer --total-timesteps $N"
  setsid nohup systemd-inhibit --what=sleep:idle --who=franka_sim --why="sac_$v training" \
    docker exec franka_ros2 bash -lc "cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
      python3 -u -m franka_sim.train --config franka_sim/config_$v.yaml --exp-name sac_$v $A $SAVE" \
    > runs/sac_${v}_train.log 2>&1 < /dev/null &
  sleep 20
done
setsid nohup bash -c 'sleep 900; while docker exec franka_ros2 pgrep -f "exp-name sac_c[0-9]_" >/dev/null 2>&1; do sleep 300; done;
  powerprofilesctl set power-saver; echo "$(date) c-batches finished, power profile -> power-saver"' \
  > runs/c2_power_restore.log 2>&1 < /dev/null &
