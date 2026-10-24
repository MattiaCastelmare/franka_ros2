#!/bin/bash
# Launch the d2 batch (mk_d2.py configs) detached on the host: 12 SAC fine-tunes sharing the GPU through MPS.
# Without MPS the GPU time-slices between processes: 8 runs gave 49 steps/s each (392 total); with MPS
# 142 each (1132), 12 runs 120 each (1444), 16 runs 93 each (1495)  [scripts/bench_sac_throughput.py].
# Each run is pinned to one E-core (8..19) at SCHED_IDLE so a robot session on the P-cores is not disturbed;
# cores 20-23 stay free for evaluation.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
M=franka_sim/models
SAVE="--checkpoint-every-episodes 400 --save-replay-buffer --no-episode-onnx"
MPS="CUDA_MPS_PIPE_DIRECTORY=/tmp/mps_d2/pipe CUDA_MPS_LOG_DIRECTORY=/tmp/mps_d2/log"
ENVX="cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 $MPS"
# MPS control daemon inside the container (private pipe dir: host processes never see it). Idempotent.
docker exec franka_ros2 bash -lc "export $MPS; mkdir -p /tmp/mps_d2/pipe /tmp/mps_d2/log;
  pgrep -x nvidia-cuda-mps-control > /dev/null || nvidia-cuda-mps-control -d"
launch() {  # name core command
  setsid nohup systemd-inhibit --what=sleep:idle --who=franka_sim --why="$1 training" \
    docker exec franka_ros2 bash -lc "$ENVX && exec taskset -c $2 chrt -i 0 $3" > runs/$1_train.log 2>&1 < /dev/null &
}
# run → parent (checkpoint dir, step)
parent() {
  case $1 in
    *_s1) echo sac_d1_a_s1 4000000 ;; *_s2) echo sac_d1_a_s2 4000000 ;; *_s3) echo sac_d1_a_s3 4000000 ;;
    *_s5) echo sac_d1_a_s5 4000000 ;; *_p1) echo sac_d1_a_p1 5000000 ;; *_p2) echo sac_d1_a_p2 5000000 ;;
    b_*)  echo sac_b3_ft_v10c 4000000 ;;
  esac
}
RUNS=${RUNS:-"p_s1 p_s5 p_s2 p_s3 p_p1 p_p2 p4_s1 p4_p1 b_1 b_2 c_s1 c_p1"}
core=8
for v in $RUNS; do
  read P STEP <<< "$(parent $v)"
  A="--resume $M/$P/checkpoints/sac_${STEP}_steps.zip --resume-buffer $M/$P/checkpoints/replay_buffer_latest.pkl"
  # P/P4/B add w_prec: relabel the buffer from the config it was collected with (C: nothing to relabel)
  case $v in c_*) ;; *) A="$A --relabel-buffer --relabel-from $M/$P/config.yaml" ;; esac
  launch sac_d2_$v $core "python3 -u -m franka_sim.train --config franka_sim/config_d2_$v.yaml --exp-name sac_d2_$v $A --total-timesteps 2000000 $SAVE"
  core=$((core + 1))
  sleep 20
done
