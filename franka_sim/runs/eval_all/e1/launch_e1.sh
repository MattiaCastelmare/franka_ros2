#!/bin/bash
# Launch the e1 batch (mk_e1.py configs) detached on the host: 4 from-scratch SAC runs, 3 vectorised
# envs each (SubprocVecEnv), sharing the GPU through MPS. Each run is pinned to 3 E-cores (8-19),
# SCHED_IDLE, so a robot session on the P-cores is not disturbed; cores 20-23 stay free for evaluation.
#   runs/eval_all/e1/launch_e1.sh            # all runs
#   RUNS="a1 a2" runs/eval_all/e1/launch_e1.sh
# Logs: runs/sac_e1_<run>_train.log; models: models/sac_e1_<run>/checkpoints (every 500k steps,
# replay buffer every 2000 episodes).
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
SAVE="--checkpoint-every-episodes 2000 --save-replay-buffer --no-episode-onnx"
MPS="CUDA_MPS_PIPE_DIRECTORY=/tmp/mps_d2/pipe CUDA_MPS_LOG_DIRECTORY=/tmp/mps_d2/log"
ENVX="cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 $MPS"
docker exec franka_ros2 bash -lc "export $MPS; mkdir -p /tmp/mps_d2/pipe /tmp/mps_d2/log;
  pgrep -x nvidia-cuda-mps-control > /dev/null || nvidia-cuda-mps-control -d"
launch() {  # name cores command
  setsid nohup systemd-inhibit --what=sleep:idle --who=franka_sim --why="$1 training" \
    docker exec franka_ros2 bash -lc "$ENVX && exec taskset -c $2 chrt -i 0 $3" > runs/$1_train.log 2>&1 < /dev/null &
}
RUNS=${RUNS:-"a1 a2 a3 m1"}
core=8
for v in $RUNS; do
  launch sac_e1_$v $core-$((core + 2)) "python3 -u -m franka_sim.train --config franka_sim/config_e1_$v.yaml --exp-name sac_e1_$v $SAVE"
  echo "sac_e1_$v  cores $core-$((core + 2))"
  core=$((core + 3))
  sleep 20
done
