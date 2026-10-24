#!/bin/bash
# Comparison videos for the v12 page (2026-10-02): benchmark episodes replayed with the held-out eval settings.
cd /ros2_ws/src; export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' > /tmp/10_nvidia.json
export __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
M=franka_sim/models; O=franka_sim/runs/videos/v12
ENS=$M/sac_v11_ens9/ens_soup9.onnx; P375=$M/sac_v11_ft_prec/checkpoints/sac_3750000_steps.zip
P400=$M/sac_v11_ft_prec/checkpoints/sac_4000000_steps.zip; C400=$M/sac_v12_cont/checkpoints/sac_4000000_steps.zip
rec() {  # out model label mode seeds [extra --set]
  python3 -m franka_sim.scripts.record_video --model $2 --label "$3" --config $M/sac_v10c/config.yaml \
    --episode-s 5 --held --seeds $5 --set task.blocking_fraction=0.6 --set obstacle.mode=$4 \
    --set obstacle.static_fraction=0 ${6:+--set $6} --out $O/$1_raw.mp4 > $O/$1.log 2>&1
}
rec a_dyn_ens   $ENS  "ens_soup9 (9-model ensemble)" sinusoidal 3013,3044,3006,3068 &
rec a_dyn_prec  $P375 "ft_prec 3.75M (single model)" sinusoidal 3013,3044,3006,3068 &
rec a_sta_ens   $ENS  "ens_soup9 (9-model ensemble)" static 3003,3008,3030,3022 &
rec a_sta_prec  $P375 "ft_prec 3.75M (single model)" static 3003,3008,3030,3022 &
rec b_prec      $P375 "ft_prec 3.75M (start of v12)" static 3021,3020,3007,3006 &
rec b_cont      $C400 "v12_cont 4.0M (resumed without buffer)" static 3021,3020,3007,3006 &
wait
rec c_off       $ENS  "ens_soup9 - original episode (infeasible)" static 3001,4226 &
rec c_on        $ENS  "ens_soup9 - same seed, IK check on" static 3001,4226 task.target_ik_check=true &
rec d_ens       $ENS  "ens_soup9 (9-model ensemble)" static 3059,4086,4089,4284 &
rec d_p400      $P400 "ft_prec 4.0M (single model)" static 3059,4086,4089,4284 &
wait
echo ALL DONE
