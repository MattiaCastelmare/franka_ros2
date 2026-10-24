#!/bin/bash
# Slide video v3 (2026-10-06): "which model wins, and when" - 3 chapters, ens_soup9 (top) vs ft_v10c 4.0M (bottom),
# 10 s episodes, obstacle on the direct route (blocking seeds 6000-6999, b3_slide_screen.py). Episodes = scenes where the
# two models DISAGREE in the chapter's dominant direction (b3_slide/stats + stats_on), spread shallow -> deep.
cd /ros2_ws/src; export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' > /tmp/10_nvidia.json
export __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
M=franka_sim/models; O=franka_sim/runs/videos/b3_slide
ENS="$M/sac_v11_ens9/ens_soup9.onnx $M/sac_v10c/config.yaml"
FT="$M/sac_b3_ft_v10c/checkpoints/sac_4000000_steps.zip $M/sac_b3_ft_v10c/config.yaml"
rec() {  # out "model config" label mode seeds shield intro1 intro2 intro3
  set -- "$1" $2 "$3" "$4" "$5" "$6" "$7" "$8" "$9" "${10}"
  python3 -m franka_sim.scripts.record_video --model $2 --config $3 --label "$4" --episode-s 10 --pad --held \
    --seeds $6 --set task.blocking_fraction=1.0 --set obstacle.mode=$5 --set obstacle.static_fraction=0 \
    --set env.cbf_obstacle_enabled=$7 --intro "$8" --intro "$9" --intro "${10}" --out $O/$1_raw.mp4 > $O/$1.log 2>&1
}
LE="ens_soup9 (9 networks, trained WITH shield)"; LF="ft_v10c (1 network, trained WITHOUT shield)"
A1="1/3  SHIELD ON (deployment)  -  static obstacle on the path"
A2="Winner: ens_soup9.  All 249 hard static scenes: held 218 vs 126, 0 collisions for both"
A3="Shown: 2 of the 99 scenes where only ens_soup9 holds the target (the reverse happens in 7)"
B1="1/3  SHIELD ON (deployment)  -  moving obstacle across the path"
B2="Winner: ens_soup9.  All 199 hard moving scenes: held 180 vs 143, 0 collisions for both"
B3="Shown: 2 of the 44 scenes where only ens_soup9 holds the target (the reverse happens in 7)"
C1="2/3  SHIELD OFF  -  static obstacle on the path"
C2="Winner: ft_v10c.  All 249 hard static scenes: collisions 84 (ens_soup9) vs 23 (ft_v10c)"
C3="Shown: 3 scenes where ens_soup9 collides and ft_v10c holds (63 scenes collide only with ens_soup9, 2 only with ft_v10c)"
D1="3/3  SHIELD OFF  -  moving obstacle across the path"
D2="Winner: ens_soup9.  All 199 hard moving scenes: held 173 vs 134, collisions 20 vs 26"
D3="Shown: 3 of the 45 scenes where only ens_soup9 holds the target (the reverse happens in 6)"
rec ch1s_ens "$ENS" "$LE" static     6683,6416      true  "$A1" "$A2" "$A3" &
rec ch1s_ft  "$FT"  "$LF" static     6683,6416      true  "$A1" "$A2" "$A3" &
rec ch1d_ens "$ENS" "$LE" sinusoidal 6285,6052      true  "$B1" "$B2" "$B3" &
rec ch1d_ft  "$FT"  "$LF" sinusoidal 6285,6052      true  "$B1" "$B2" "$B3" &
wait
rec ch2_ens  "$ENS" "$LE" static     6183,6295,6487 false "$C1" "$C2" "$C3" &
rec ch2_ft   "$FT"  "$LF" static     6183,6295,6487 false "$C1" "$C2" "$C3" &
rec ch3_ens  "$ENS" "$LE" sinusoidal 6922,6315,6972 false "$D1" "$D2" "$D3" &
rec ch3_ft   "$FT"  "$LF" sinusoidal 6922,6315,6972 false "$D1" "$D2" "$D3" &
wait
echo ALL DONE
