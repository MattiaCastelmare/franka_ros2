#!/bin/bash
# d2 scorecard videos (2026-10-13): the 12 d2 networks at +2.0M and the 12-net ensemble on the d1 video seeds,
# policy alone, nominal dynamics, constraint monitor on (no termination) so the overlay counts violation ticks.
#   docker exec franka_ros2 taskset -c 8-23 chrt -i 0 bash /ros2_ws/src/franka_sim/runs/eval_all/d2/rec_d2.sh \
#       3005,3000,3014,3008 3004,3005,3009,3030 [EXTRA_NAME EXTRA_SCENARIO EXTRA_SEEDS]
# The optional extra job records the ensemble alone on its own failure seeds.
cd /ros2_ws/src; export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 CUDA_VISIBLE_DEVICES=
printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' > /tmp/10_nvidia.json
export __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json
M=franka_sim/models; O=franka_sim/runs/videos/d2; DYN=$1; STA=$2; mkdir -p $O
rec() {  # name model config label mode seeds
  [ -s $O/${1}_raw.mp4 ] && grep -q " frames " $O/${1}.log 2>/dev/null && return
  echo "python3 -m franka_sim.scripts.record_video --model $2 --config $3 --label '$4' --episode-s 5 --pad --held \
    --seeds $6 --set task.blocking_fraction=0.6 --set obstacle.mode=$5 --set obstacle.static_fraction=0 \
    --set randomization.enabled=false --set constraints.enabled=true --set constraints.terminate=false \
    --out $O/${1}_raw.mp4 > $O/${1}.log 2>&1"
}
E=$M/sac_d2_ens/ens_d2k2000_all12.onnx; EC=$M/sac_d2_ens/config.yaml
{
for r in p_s1 p_s5 p_s2 p_s3 p_p1 p_p2 p4_s1 p4_p1 b_1 b_2 c_s1 c_p1; do
  case $r in *_p1|*_p2) st=7000000 ;; *) st=6000000 ;; esac
  ck=$M/sac_d2_$r/checkpoints/sac_${st}_steps.zip; cf=$M/sac_d2_$r/config.yaml
  rec ${r}_dyn $ck $cf "sac_d2_$r +2.0M - moving obstacle" sinusoidal $DYN
  rec ${r}_sta $ck $cf "sac_d2_$r +2.0M - static obstacle" static $STA
done
rec ens_dyn $E $EC "ens_d2k2000_all12 (12 nets) - moving obstacle" sinusoidal $DYN
rec ens_sta $E $EC "ens_d2k2000_all12 (12 nets) - static obstacle" static $STA
[ -n "$5" ] && rec $3 $E $EC "ens_d2k2000_all12 (12 nets) - its failures" $4 $5
} | xargs -d '\n' -P ${P:-13} -I{} bash -c {}
echo ALL DONE
