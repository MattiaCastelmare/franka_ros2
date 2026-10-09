#!/bin/bash
# Slide video v4 (2026-10-06): 3 cases x 4 episodes, ens_soup9 (top) vs ft_v10c 4.0M (bottom), 10 s, obstacle on the direct
# route; no record_video title/summary cards (b3_cards.py makes big plain ones). Seeds = disagreements in the case's
# dominant direction (b3_slide/stats, stats_on), spread shallow -> deep.
cd /ros2_ws/src; export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' > /tmp/10_nvidia.json
export __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
M=franka_sim/models; O=franka_sim/runs/videos/b3_slide/v4; mkdir -p $O
ENS="$M/sac_v11_ens9/ens_soup9.onnx $M/sac_v10c/config.yaml"
FT="$M/sac_b3_ft_v10c/checkpoints/sac_4000000_steps.zip $M/sac_b3_ft_v10c/config.yaml"
rec() {  # out "model config" label mode seeds shield
  set -- "$1" $2 "$3" "$4" "$5" "$6" "$7"
  python3 -m franka_sim.scripts.record_video --model $2 --config $3 --label "$4" --episode-s 10 --pad --held \
    --title-s 0 --summary-s 0 --seeds $6 --set task.blocking_fraction=1.0 --set obstacle.mode=$5 \
    --set obstacle.static_fraction=0 --set env.cbf_obstacle_enabled=$7 --out $O/$1_raw.mp4 > $O/$1.log 2>&1
}
LE="ens_soup9 (9 networks, trained WITH shield)"; LF="ft_v10c (1 network, v10c fine-tuned WITHOUT shield)"
rec c1s_ens "$ENS" "$LE" static     6683,6416           true &
rec c1s_ft  "$FT"  "$LF" static     6683,6416           true &
rec c1d_ens "$ENS" "$LE" sinusoidal 6285,6052           true &
rec c1d_ft  "$FT"  "$LF" sinusoidal 6285,6052           true &
wait
rec c2_ens  "$ENS" "$LE" static     6183,6255,6886,6487 false &
rec c2_ft   "$FT"  "$LF" static     6183,6255,6886,6487 false &
rec c3_ens  "$ENS" "$LE" sinusoidal 6922,6256,6837,6972 false &
rec c3_ft   "$FT"  "$LF" sinusoidal 6922,6256,6837,6972 false &
wait
python3 franka_sim/runs/eval_all/b3_cards.py $O/cards
echo ALL DONE
