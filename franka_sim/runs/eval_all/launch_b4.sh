#!/bin/bash
# Launch the b4 batch (mk_b4.py configs: shield-OFF fine-tunes for proper avoidance) detached on the host.
# Capacity (training_capacity_laptop): ≤10 GPU runs, ~32 fps each with the power profile on 'performance'
# → 1.5M steps ≈ 13 h. Container RAM ≈ 16.5 GiB of 20 → keep ≤3 parallel eval jobs while these run.
# Every run resumes a shield-trained checkpoint + ITS OWN replay buffer, relabelled from the config the buffer was
# collected with (--relabel-from). C3 = --buffer-mask (runs/eval_all/b4_masks, scripts/shield_buffer_mask.py),
# C4 = --widen-obs (config has obs.obstacle_velocity). When every run has exited, power profile → power-saver.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
M=franka_sim/models
K=franka_sim/runs/eval_all/b4_masks
SAVE="--checkpoint-every-episodes 400 --save-replay-buffer --no-episode-onnx"
RUNS=${RUNS:-"b4_clear b4_clear_filt b4_clear_mix b4_clear_vobs b4_full b4_full_ps3 b4_full_ps2 b4_full_lr3e5 b4_full_s2 b4_full_m25"}
src() {  # run name → "SOURCE_RUN CHECKPOINT_STEPS"
  case $1 in
    b4_full_ps3)   echo "v12_prec_s3 3750000" ;;
    b4_full_ps2)   echo "v12_prec_s2 3750000" ;;
    b4_full_lr3e5) echo "v12_lr3e5 4750000" ;;
    *)             echo "v10c 2500000" ;;
  esac
}
for v in $RUNS; do
  read S STEP <<< "$(src $v)"
  A="--resume $M/sac_$S/checkpoints/sac_${STEP}_steps.zip --resume-buffer $M/sac_$S/checkpoints/replay_buffer_latest.pkl"
  A="$A --relabel-buffer --relabel-from $M/sac_$S/config.yaml --total-timesteps 1500000"
  case $v in b4_clear_filt|b4_full*) A="$A --buffer-mask $K/$S.npz" ;; esac
  case $v in b4_clear_vobs|b4_full*) A="$A --widen-obs" ;; esac
  setsid nohup systemd-inhibit --what=sleep:idle --who=franka_sim --why="sac_$v training" \
    docker exec franka_ros2 bash -lc "cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
      python3 -u -m franka_sim.train --config franka_sim/config_$v.yaml --exp-name sac_$v $A $SAVE" \
    > runs/sac_${v}_train.log 2>&1 < /dev/null &
  sleep 20   # stagger the buffer loads (~0.5 GB transient each)
done
setsid nohup bash -c 'sleep 600; while docker exec franka_ros2 pgrep -f "exp-name sac_b4_" >/dev/null 2>&1; do sleep 300; done;
  powerprofilesctl set power-saver; echo "$(date) b4 batch finished, power profile -> power-saver"' \
  > runs/b4_power_restore.log 2>&1 < /dev/null &
