#!/bin/bash
# Moving-obstacle comparison videos for the b3 page (2026-10-06, second batch): same settings as b3_videos.sh.
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
rec b_dyn_ft   "$FT"   "ft_v10c 4.0M (fine-tune) - shield OFF"    sinusoidal 3001,3003,3005,3007 false &
rec b_dyn_base "$BASE" "b3_base 2.0M (from scratch) - shield OFF" sinusoidal 3001,3003,3005,3007 false &
rec d_dyn_ens  "$ENS"  "ens_soup9 - shield ON"                    sinusoidal 3013,3016,3018,3023 true &
rec d_dyn_ft   "$FT"   "ft_v10c 4.0M - shield ON"                 sinusoidal 3013,3016,3018,3023 true &
rec e_dyn_ft   "$FT"   "ft_v10c 4.0M - shield OFF"                sinusoidal 4126,4176,3085,3094 false &
rec e_dyn_ens  "$ENS"  "ens_soup9 - shield OFF"                   sinusoidal 4126,4176,3085,3094 false &
wait
echo ALL DONE
