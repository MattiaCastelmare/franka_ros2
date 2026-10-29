#!/bin/bash
# Launch the d3 batch (mk_d3.py configs) detached on the host: 12 shield-in-the-loop SAC fine-tunes sharing
# the GPU through MPS (as d2: one E-core each, 8..19, SCHED_IDLE; cores 20-23 stay free for evaluation).
# Each run resumes a d2 network at +2.0M on a FRESH buffer: the d2 buffers were collected without the shield,
# so their transitions are wrong wherever it would have acted. --policy-warmup fills the new buffer with the
# parent's own behaviour behind the shield before the first gradient step.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_rl"
M=franka_rl/models
SAVE="--checkpoint-every-episodes 400 --save-replay-buffer --no-episode-onnx"
MPS="CUDA_MPS_PIPE_DIRECTORY=/tmp/mps_d2/pipe CUDA_MPS_LOG_DIRECTORY=/tmp/mps_d2/log"
ENVX="cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 $MPS"
docker exec franka_ros2 bash -lc "export $MPS; mkdir -p /tmp/mps_d2/pipe /tmp/mps_d2/log;
  pgrep -x nvidia-cuda-mps-control > /dev/null || nvidia-cuda-mps-control -d"
launch() {  # name core command
  setsid nohup systemd-inhibit --what=sleep:idle --who=franka_rl --why="$1 training" \
    docker exec franka_ros2 bash -lc "$ENVX && exec taskset -c $2 chrt -i 0 $3" > runs/$1_train.log 2>&1 < /dev/null &
}
# d3 run → d2 parent run (from mk_d3.py)
parent() {
  python3 -c "import sys; sys.path.insert(0, 'runs/eval_all/d3'); from mk_d3 import RUNS; print(RUNS['$1'][0])"
}
RUNS=${RUNS:-"d_s5 d_p2 d_b2 d_p4p1 d_s1 d_cp1 d1_p4s1 d1_s3 f_b1 f_s2 m_cs1 m_p1"}
core=8
for v in $RUNS; do
  P=$(parent $v)
  case $P in *_p1|*_p2) STEP=7000000 ;; *) STEP=6000000 ;; esac
  A="--resume $M/sac_d2_$P/checkpoints/sac_${STEP}_steps.zip --policy-warmup 50000"
  launch sac_d3_$v $core "python3 -u -m franka_rl.train --config franka_rl/config_d3_$v.yaml --exp-name sac_d3_$v $A --total-timesteps 2000000 $SAVE"
  echo "sac_d3_$v  core $core  parent sac_d2_$P @ $STEP"
  core=$((core + 1))
  sleep 20
done
