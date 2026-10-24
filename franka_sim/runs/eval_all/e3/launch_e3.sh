#!/bin/bash
# Launch the e3 batch (mk_e3.py configs) detached on the host: the 12 e2 finals continue +1.5M with the d_safe
# penalty at w 4, each from its own 51-D replay buffer with the rewards relabelled from its e2 config.
# Same placement as e2: MPS, one E-core each (8..19) at SCHED_IDLE, 20-23 for eval.
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
start() { case ${1#?_} in p_p2|p4_p1) echo 9000000 ;; *) echo 8000000 ;; esac; }   # e2 final step
RUNS=${RUNS:-"g_p_s5 s_p_s5 g_p4_s1 s_p4_s1 g_p_p2 s_p_p2 g_b_1 s_b_1 g_b_2 s_b_2 g_p4_p1 s_p4_p1"}
core=8
for v in $RUNS; do
  A="--resume $M/sac_e2_$v/checkpoints/sac_$(start $v)_steps.zip --resume-buffer $M/sac_e2_$v/checkpoints/replay_buffer_latest.pkl"
  A="$A --relabel-buffer --relabel-from $M/sac_e2_$v/config.yaml"
  launch sac_e3_$v $core "python3 -u -m franka_sim.train --config franka_sim/config_e3_$v.yaml --exp-name sac_e3_$v $A --total-timesteps 1500000 $SAVE"
  core=$((core + 1))
  sleep 20
done
