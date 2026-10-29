#!/bin/bash
# d1 scorecard videos (2026-10-12): every d1 model on the same seeds, policy alone, nominal dynamics,
# constraint monitor on (no termination) so the overlay counts violation ticks.  rec_d1.sh DYN_SEEDS STA_SEEDS
# Used: rec_d1.sh 3005,3000,3014,3008 3004,3005,3009,3030 (2 seeds big11 + all s-runs pass, 2 that split the s-runs 3/3).
cd /ros2_ws/src; export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' > /tmp/10_nvidia.json
export __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
M=franka_rl/models; O=franka_rl/runs/videos/d1; DYN=$1; STA=$2
rec() {  # name model config label mode seeds
  echo "python3 -m franka_rl.scripts.record_video --model $2 --config $3 --label '$4' --episode-s 5 --pad --held \
    --seeds $6 --set task.blocking_fraction=0.6 --set obstacle.mode=$5 --set obstacle.static_fraction=0 \
    --set randomization.enabled=false --set constraints.enabled=true --set constraints.terminate=false \
    --out $O/${1}_raw.mp4 > $O/${1}.log 2>&1"
}
{
for v in s1 s2 s3 s4 s5 s6 p1 p2; do
  rec ${v}_dyn $M/sac_d1_a_$v/best_model.zip $M/sac_d1_a_$v/config.yaml "sac_d1_a_$v best_model - moving obstacle" sinusoidal $DYN
  rec ${v}_sta $M/sac_d1_a_$v/best_model.zip $M/sac_d1_a_$v/config.yaml "sac_d1_a_$v best_model - static obstacle" static $STA
done
rec ppo_dyn $M/ppo_d1/final_model.zip $M/ppo_d1/config.yaml "ppo_d1 final - moving obstacle" sinusoidal $DYN
rec ppo_sta $M/ppo_d1/final_model.zip $M/ppo_d1/config.yaml "ppo_d1 final - static obstacle" static $STA
} | xargs -d '\n' -P ${P:-6} -I{} bash -c {}
echo ALL DONE
