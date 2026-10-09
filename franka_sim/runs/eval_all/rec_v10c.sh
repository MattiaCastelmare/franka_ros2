#!/bin/bash
# Videos of sac_v10c (checkpoint given as $1, e.g. checkpoints/sac_2500000_steps), same seeds/configs as rec_v10.sh.   rec_v10c.sh MODEL TAG "dyn numbers" "static numbers"
cd /ros2_ws/src; printf "{\"file_format_version\":\"1.0.0\",\"ICD\":{\"library_path\":\"libEGL_nvidia.so.0\"}}\n" > /tmp/10_nvidia.json
export OMP_NUM_THREADS=1 PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
rec() {  # scenario, intro line, test numbers
  python3 -m franka_sim.scripts.record_video --model franka_sim/models/sac_v10c/$M.zip \
    --config franka_sim/runs/eval_all/sac_v10_$1.yaml --label "sac_v10c ($TAG)" --episodes 10 --episode-s 10 \
    --intro "$2" \
    --intro "sac_v10 resumed for +2M steps (seed 1); this is the $TAG checkpoint" \
    --intro "shield: HOCBF obstacle rows + joint box + EE workspace + FLOOR rows + BASE keep-out" \
    --intro "$3" \
    --out franka_sim/runs/videos/sac_v10c_${TAG}_$1_raw.mp4 > franka_sim/runs/videos/sac_v10c_${TAG}_$1.log 2>&1
}
M=$1; TAG=$2
rec static  "STATIC obstacle" "$4" &
rec dynamic "MOVING obstacle (0.2 Hz, +/-0.2 m sweep)" "$3" &
wait
