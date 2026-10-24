#!/bin/bash
# Replay videos for the b4 page (2026-10-07). Shield OFF at test time, eval settings (blocking 0.6, 5 s, one scenario
# per video), FINAL 1.5M checkpoint of every run + ft_v10c 4.0M reference, seeds 4000-4007 (first 8 held-out seeds,
# fixed before looking at any video). env.cbf_obstacle_on_prob=null: mixing runs are replayed with the shield fixed OFF.
cd /ros2_ws/src; export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' > /tmp/10_nvidia.json
export __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
M=franka_sim/models; O=franka_sim/runs/videos/b4; SEEDS=4000,4001,4002,4003,4004,4005,4006,4007
rec() {  # name model config label
  for sc in static sinusoidal; do
    s=${sc/sinusoidal/dynamic}
    echo "python3 -m franka_sim.scripts.record_video --model $2 --config $3 --label '$4 - shield OFF' --episode-s 5 --pad --held \
      --seeds $SEEDS --set task.blocking_fraction=0.6 --set obstacle.mode=$sc --set obstacle.static_fraction=0 \
      --set env.cbf_obstacle_enabled=false --set env.cbf_obstacle_on_prob=null --out $O/${1}_${s}_raw.mp4 > $O/${1}_${s}.log 2>&1"
  done
}
{
rec ref_ftv10c "$M/sac_b3_ft_v10c/checkpoints/sac_4000000_steps.zip" "$M/sac_b3_ft_v10c/config.yaml" "ft_v10c 4.0M (b3 reference)"
for v in clear clear_filt clear_mix clear_vobs full full_s2 full_m25 full_ps3 full_ps2 full_lr3e5; do
  case $v in full_ps3|full_ps2) s=5250000;; full_lr3e5) s=6250000;; *) s=4000000;; esac
  rec $v "$M/sac_b4_$v/checkpoints/sac_${s}_steps.zip" "$M/sac_b4_$v/config.yaml" "b4 $v 1.5M"
done
} | xargs -d '\n' -P 8 -I{} bash -c {}
echo ALL DONE
