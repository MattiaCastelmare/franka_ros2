#!/bin/bash
# Videos of sac_v10/best_model in the static and dynamic scenarios (same configs as the 40-seed hold test).
cd /ros2_ws/src; printf "{\"file_format_version\":\"1.0.0\",\"ICD\":{\"library_path\":\"libEGL_nvidia.so.0\"}}\n" > /tmp/10_nvidia.json
export OMP_NUM_THREADS=1 PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
rec() {  # scenario, intro line, 40-seed numbers
  python3 -m franka_sim.scripts.record_video --model franka_sim/models/sac_v10/best_model.zip \
    --config franka_sim/runs/eval_all/sac_v10_$1.yaml --label "sac_v10 (best)" --episodes 10 --episode-s 10 \
    --intro "$2" \
    --intro "trained to HOLD the target (no terminate_on_success); obstacle ON the path in ~60% of episodes" \
    --intro "shield: HOCBF obstacle rows + joint box + EE workspace + FLOOR rows + BASE keep-out" \
    --intro "$3" \
    --out franka_sim/runs/videos/sac_v10_$1_raw.mp4 > franka_sim/runs/videos/sac_v10_$1.log 2>&1
}
rec static  "STATIC obstacle" "40-seed test: target reached 25/40, held 17/40, collisions 0/40, floor contacts 0/40" &
rec dynamic "MOVING obstacle (0.2 Hz, +/-0.2 m sweep)" "40-seed test: target reached 37/40, held 25/40, collisions 0/40, floor contacts 0/40" &
wait
