#!/bin/bash
# Videos of a v11 checkpoint, same seeds/configs as rec_v10c.sh.   rec_v11.sh RUN MODEL TAG "what" "dyn numbers" "static numbers"
cd /ros2_ws/src; printf "{\"file_format_version\":\"1.0.0\",\"ICD\":{\"library_path\":\"libEGL_nvidia.so.0\"}}\n" > /tmp/10_nvidia.json
export OMP_NUM_THREADS=1 PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
R=$1; M=$2; TAG=$3; WHAT=$4
rec() {  # scenario, intro line, test numbers
  python3 -m franka_sim.scripts.record_video --model franka_sim/models/$R/$M.zip \
    --config franka_sim/runs/eval_all/sac_v10_$1.yaml --label "$R ($TAG)" --episodes 10 --episode-s 10 \
    --intro "$2" --intro "$WHAT" \
    --intro "shield: HOCBF obstacle rows + joint box + EE workspace + FLOOR rows + BASE keep-out" \
    --intro "$3" \
    --out franka_sim/runs/videos/${R}_${TAG}_$1_raw.mp4 > franka_sim/runs/videos/${R}_${TAG}_$1.log 2>&1
}
rec static  "STATIC obstacle" "$6" &
rec dynamic "MOVING obstacle (0.2 Hz, +/-0.2 m sweep)" "$5" &
wait
