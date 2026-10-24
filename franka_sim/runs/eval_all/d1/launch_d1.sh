#!/bin/bash
# Launch the d1 batch (mk_d1.py configs) detached on the host: 8 SAC fine-tunes (GPU) + 1 PPO (14 CPU envs).
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
M=franka_sim/models
SAVE="--checkpoint-every-episodes 400 --save-replay-buffer --no-episode-onnx"
ENVX="cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1"
launch() {  # name command
  setsid nohup systemd-inhibit --what=sleep:idle --who=franka_sim --why="$1 training" \
    docker exec franka_ros2 bash -lc "$ENVX && $2" > runs/$1_train.log 2>&1 < /dev/null &
}
RUNS=${RUNS:-"a_s1 a_s2 a_s3 a_s4 a_s5 a_s6 a_p1 a_p2 ppo"}
for v in $RUNS; do
  case $v in
    a_s*) P=$M/sac_v10c/checkpoints
          A="--resume $P/sac_2500000_steps.zip --resume-buffer $P/replay_buffer_latest.pkl --relabel-buffer --total-timesteps 1500000"
          launch sac_d1_$v "python3 -u -m franka_sim.train --config franka_sim/config_d1_$v.yaml --exp-name sac_d1_$v $A $SAVE" ;;
    a_p*) P=$M/sac_v12_prec_s3/checkpoints
          A="--resume $P/sac_3750000_steps.zip --resume-buffer $P/replay_buffer_latest.pkl --relabel-buffer --relabel-from $M/sac_v12_prec_s3/config.yaml --total-timesteps 1250000"
          launch sac_d1_$v "python3 -u -m franka_sim.train --config franka_sim/config_d1_$v.yaml --exp-name sac_d1_$v $A $SAVE" ;;
    ppo)  launch ppo_d1 "python3 -u -m franka_sim.train_ppo --config franka_sim/config_d1_ppo.yaml --exp-name ppo_d1" ;;
  esac
  sleep 15
done
