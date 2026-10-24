#!/bin/bash
# Slide video (2026-10-06): ens_soup9 (trained WITH shield) vs ft_v10c 4.0M (fine-tuned WITHOUT obstacle shield),
# both run with the obstacle CBF OFF, 10 s episodes, obstacle on the direct route. Seeds from b3_slide_screen.py
# (outcome-blind, 5 per mode, ordered shallow -> deepest hand_clearance).
cd /ros2_ws/src; export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' > /tmp/10_nvidia.json
export __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
M=franka_sim/models; O=franka_sim/runs/videos/b3_slide
ENS="$M/sac_v11_ens9/ens_soup9.onnx $M/sac_v10c/config.yaml"
FT="$M/sac_b3_ft_v10c/checkpoints/sac_4000000_steps.zip $M/sac_b3_ft_v10c/config.yaml"
STA=6901,6585,6458,6091,6416; DYN=6126,6705,6033,6885,6712
rec() {  # out "model config" label mode seeds intro
  set -- "$1" $2 "$3" "$4" "$5" "$6"
  python3 -m franka_sim.scripts.record_video --model $2 --config $3 --label "$4" --episode-s 10 --pad --held \
    --seeds $6 --set task.blocking_fraction=1.0 --set obstacle.mode=$5 --set obstacle.static_fraction=0 \
    --set env.cbf_obstacle_enabled=false --intro "$7" --out $O/$1_raw.mp4 > $O/$1.log 2>&1
}
I_S="STATIC obstacle on the direct route - 5 episodes, increasing difficulty (direct route cuts 1 -> 7 cm into it)"
I_D="MOVING obstacle sweeping across the route - 5 episodes, increasing difficulty (1 -> 19 cm)"
rec ens_static "$ENS" "ens_soup9 - trained WITH shield - test: shield OFF"    static     $STA "$I_S" &
rec ft_static  "$FT"  "ft_v10c - trained WITHOUT shield - test: shield OFF"   static     $STA "$I_S" &
rec ens_dyn    "$ENS" "ens_soup9 - trained WITH shield - test: shield OFF"    sinusoidal $DYN "$I_D" &
rec ft_dyn     "$FT"  "ft_v10c - trained WITHOUT shield - test: shield OFF"   sinusoidal $DYN "$I_D" &
wait
echo ALL DONE
