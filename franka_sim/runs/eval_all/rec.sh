#!/bin/bash
cd /ros2_ws/src; printf "{\"file_format_version\":\"1.0.0\",\"ICD\":{\"library_path\":\"libEGL_nvidia.so.0\"}}\n" > /tmp/10_nvidia.json
export OMP_NUM_THREADS=1 PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
rec() {  # scenario, intro line
  python3 -m franka_sim.scripts.record_video --model franka_sim/models/sac_v4/final_model.zip \
    --config franka_sim/runs/eval_all/sac_v4_$1.yaml --label "sac_v4 (final)" --episodes 10 --episode-s 10 \
    --intro "$2" \
    --intro "every target reachable (>= 0.26 m from the obstacle); obstacle ON the path in ~60% of episodes" \
    --intro "shield: HOCBF obstacle rows + joint box + EE workspace + FLOOR rows + BASE keep-out" \
    --intro "$3" \
    --out franka_sim/runs/videos/best_sac_v4_$1_raw.mp4 > franka_sim/runs/videos/best_sac_v4_$1.log 2>&1
}
rec static  "STATIC obstacle" "40-seed test: target reached 19/40, held 9/40, collisions 0/40, floor contacts 0/40" &
rec dynamic "MOVING obstacle (0.2 Hz, +/-0.2 m sweep)" "40-seed test: target reached 39/40, held 20/40, collisions 0/40, floor contacts 0/40" &
wait
