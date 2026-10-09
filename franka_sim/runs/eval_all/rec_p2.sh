#!/bin/bash
# Videos of sac_v10_obs51 / sac_v10_g997 best_model, static + dynamic, same seeds (12345..) and settings as rec_v10.sh.
cd /ros2_ws/src; printf "{\"file_format_version\":\"1.0.0\",\"ICD\":{\"library_path\":\"libEGL_nvidia.so.0\"}}\n" > /tmp/10_nvidia.json
export OMP_NUM_THREADS=1 PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
T=franka_sim/runs/eval_all
# Scenario configs = the run's own config with the obstacle forced static / sinusoidal (as sac_v10_{static,dynamic}.yaml).
python3 - <<'EOF'
import yaml
for r in ['sac_v10_obs51', 'sac_v10_g997']:
    c = yaml.safe_load(open(f'franka_sim/models/{r}/config.yaml'))
    for sc, mode in [('static', 'static'), ('dynamic', 'sinusoidal')]:
        c['obstacle']['mode'] = mode; c['obstacle']['static_fraction'] = 0
        yaml.safe_dump(c, open(f'franka_sim/runs/eval_all/{r}_{sc}.yaml', 'w'))
EOF
rec() {  # run, scenario, label, intro line, 40-seed numbers, the one change
  python3 -m franka_sim.scripts.record_video --model franka_sim/models/$1/best_model.zip \
    --config $T/$1_$2.yaml --label "$3" --episodes 10 --episode-s 10 \
    --intro "$4" --intro "$5" \
    --intro "same recipe as sac_v10 except ONE change: $6" \
    --out franka_sim/runs/videos/$1_$2_raw.mp4 > franka_sim/runs/videos/$1_$2.log 2>&1
}
rec sac_v10_obs51 static  "sac_v10_obs51 (best)" "STATIC obstacle" "40-seed test: reached 13/40, held 6/40, collisions 0/40" "51-D observation (v_obs + per-CP d, n)" &
rec sac_v10_obs51 dynamic "sac_v10_obs51 (best)" "MOVING obstacle (0.2 Hz, +/-0.2 m sweep)" "40-seed test: reached 10/40, held 2/40, collisions 0/40" "51-D observation (v_obs + per-CP d, n)" &
rec sac_v10_g997  static  "sac_v10_g997 (best)" "STATIC obstacle" "40-seed test: reached 4/40, held 0/40, collisions 0/40" "gamma 0.997 instead of 0.99" &
rec sac_v10_g997  dynamic "sac_v10_g997 (best)" "MOVING obstacle (0.2 Hz, +/-0.2 m sweep)" "40-seed test: reached 9/40, held 3/40, collisions 0/40" "gamma 0.997 instead of 0.99" &
wait
