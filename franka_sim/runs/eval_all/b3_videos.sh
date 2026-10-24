#!/bin/bash
# Comparison videos for the b3 page (2026-10-06): benchmark episodes replayed with the held-out eval settings.
# OFF = obstacle CBF rows off (env.cbf_obstacle_enabled=false), ON = shield on. Each model with its own frozen config.
cd /ros2_ws/src; export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' > /tmp/10_nvidia.json
export __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
M=franka_sim/models; O=franka_sim/runs/videos/b3
FT="$M/sac_b3_ft_v10c/checkpoints/sac_4000000_steps.zip $M/sac_b3_ft_v10c/config.yaml"
V10C="$M/sac_v10c/checkpoints/sac_2500000_steps.zip $M/sac_v10c/config.yaml"
BASE="$M/sac_b3_base/checkpoints/sac_2000000_steps.zip $M/sac_b3_base/config.yaml"
ENS="$M/sac_v11_ens9/ens_soup9.onnx $M/sac_v10c/config.yaml"
rec() {  # out "model config" label mode seeds shield(true|false)
  set -- "$1" $2 "$3" "$4" "$5" "$6"
  python3 -m franka_sim.scripts.record_video --model $2 --config $3 --label "$4" \
    --episode-s 5 --pad --held --seeds $6 --set task.blocking_fraction=0.6 --set obstacle.mode=$5 \
    --set obstacle.static_fraction=0 --set env.cbf_obstacle_enabled=$7 --out $O/$1_raw.mp4 > $O/$1.log 2>&1
}
rec a_sta_ft   "$FT"   "ft_v10c 4.0M - shield OFF"          static     3003,3023,3028,3047 false &
rec a_sta_v10c "$V10C" "v10c 2.5M (start) - shield OFF"     static     3003,3023,3028,3047 false &
rec a_dyn_ft   "$FT"   "ft_v10c 4.0M - shield OFF"          sinusoidal 3028,3030,3068,3078 false &
rec a_dyn_v10c "$V10C" "v10c 2.5M (start) - shield OFF"     sinusoidal 3028,3030,3068,3078 false &
rec b_ft       "$FT"   "ft_v10c 4.0M (fine-tune) - shield OFF" static  3002,3004,3005,3006 false &
rec b_base     "$BASE" "b3_base 2.0M (from scratch) - shield OFF" static 3002,3004,3005,3006 false &
wait
rec c_off      "$FT"   "ft_v10c 4.0M - shield OFF"          sinusoidal 4126,4176,4213,4233 false &
rec c_on       "$FT"   "ft_v10c 4.0M - shield ON"           sinusoidal 4126,4176,4213,4233 true &
# rec d_ens      "$ENS"  "ens_soup9 - shield ON"              static     3002,3006,3007,3008 true &
# rec d_ft       "$FT"   "ft_v10c 4.0M - shield ON"           static     3002,3006,3007,3008 true &
wait
echo ALL DONE
