#!/bin/bash
# Launch the e2 batch (mk_e2.py configs) detached on the host: 12 SAC fine-tunes sharing the GPU through MPS,
# one E-core each (8..19) at SCHED_IDLE so a robot session on the P-cores is not disturbed; 20-23 stay for eval.
# Each run grafts its d2 parent (final checkpoint + its own replay buffer) onto the 51-D observation
# (--widen-obs: zero weights on the new inputs, buffer rebuilt at 51-D by train.widen_buffer); the S arm
# relabels the buffer rewards from the parent's config (per-step d_safe penalty instead of the potential).
#   RUNS="g_p_s5 s_p_s5" runs/eval_all/e2/launch_e2.sh      (default: all 12)
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
M=franka_sim/models
SAVE="--checkpoint-every-episodes 400 --save-replay-buffer --no-episode-onnx"
MPS="CUDA_MPS_PIPE_DIRECTORY=/tmp/mps_d2/pipe CUDA_MPS_LOG_DIRECTORY=/tmp/mps_d2/log"
ENVX="cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 $MPS"
docker exec franka_ros2 bash -lc "export $MPS; mkdir -p /tmp/mps_d2/pipe /tmp/mps_d2/log;
  pgrep -x nvidia-cuda-mps-control > /dev/null || nvidia-cuda-mps-control -d"
launch() {  # name core command
  setsid nohup systemd-inhibit --what=sleep:idle --who=franka_sim --why="$1 training" \
    docker exec franka_ros2 bash -lc "$ENVX && exec taskset -c $2 chrt -i 0 $3" > runs/$1_train.log 2>&1 < /dev/null &
}
step() {  # d2 parent → step of its final checkpoint (= the ens_d2k2000_all12 member)
  case $1 in p_p2|p4_p1) echo 7000000 ;; *) echo 6000000 ;; esac
}
RUNS=${RUNS:-"g_p_s5 s_p_s5 g_p4_s1 s_p4_s1 g_p_p2 s_p_p2 g_b_1 s_b_1 g_b_2 s_b_2 g_p4_p1 s_p4_p1"}
core=8
for v in $RUNS; do
  P=${v#?_}; STEP=$(step $P)
  A="--resume $M/sac_d2_$P/checkpoints/sac_${STEP}_steps.zip --resume-buffer $M/sac_d2_$P/checkpoints/replay_buffer_latest.pkl --widen-obs"
  case $v in s_*) A="$A --relabel-buffer --relabel-from $M/sac_d2_$P/config.yaml" ;; esac
  launch sac_e2_$v $core "python3 -u -m franka_sim.train --config franka_sim/config_e2_$v.yaml --exp-name sac_e2_$v $A --total-timesteps 2000000 $SAVE"
  core=$((core + 1))
  sleep 20
done
