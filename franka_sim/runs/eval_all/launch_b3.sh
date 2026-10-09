#!/bin/bash
# Launch the b3 batch (mk_b3.py configs, end-to-end RL with the obstacle CBF OFF) detached on the host.
# Capacity (bench_b3.sh, 2026-10-05, power profile 'performance'): the GPU saturates at ~320 fps total;
# 10 GPU runs = ~32 fps each (2M steps ≈ 17-18 h), container RAM ≈ 13.8 GiB + 10 replay buffers (~2.5 GiB) of 20 GiB.
# Leave ≤3 parallel eval jobs (~1 GiB each) while these run. When every run has exited, the power profile goes
# back to power-saver (the user's setting before the batch).
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
V=franka_sim/models/sac_v10c/checkpoints
SAVE="--checkpoint-every-episodes 400 --save-replay-buffer --no-episode-onnx"
RUNS=${RUNS:-"b3_base b3_base_s2 b3_base_s3 b3_margin30 b3_wobs5 b3_coll200 b3_prec b3_obs51 b3_noterm b3_ft_v10c"}
for v in $RUNS; do
  case $v in
    b3_ft_v10c) A="--resume $V/sac_2500000_steps.zip --resume-buffer $V/replay_buffer_latest.pkl --relabel-buffer --total-timesteps 1500000" ;;
    *)          A="" ;;
  esac
  setsid nohup systemd-inhibit --what=sleep:idle --who=franka_sim --why="sac_$v training" \
    docker exec franka_ros2 bash -lc "cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
      python3 -u -m franka_sim.train --config franka_sim/config_$v.yaml --exp-name sac_$v $A $SAVE" \
    > runs/sac_${v}_train.log 2>&1 < /dev/null &
  sleep 5
done
# restore the power profile once no b3 training is left in the container
setsid nohup bash -c 'sleep 600; while docker exec franka_ros2 pgrep -f "exp-name sac_b3_" >/dev/null 2>&1; do sleep 300; done;
  powerprofilesctl set power-saver; echo "$(date) b3 batch finished, power profile -> power-saver"' \
  > runs/b3_power_restore.log 2>&1 < /dev/null &
