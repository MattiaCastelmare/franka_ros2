#!/bin/bash
# e1 evaluation. Two protocols, controllers from stdin as in d2/eval_d2.sh (TAG|zip:PATH or onnx:PATH|CONFIG):
#   SCENE=one   (default) the d1/d2/d3 held-out protocol: ONE sphere, 400 static + 400 moving seeds,
#               randomisation off → numbers comparable with traces/d2 (ens_d1b8_d2k2000_full 750,
#               big11clip_full 731; alone: ens_d2k2000_all12 794, big11clip 739). Traces → traces/d2/.
#   SCENE=multi 2-3 spheres per episode (obstacle.count_range=[2,3]), same seeds → traces/e1_multi/.
#   MODE=full (deployed shield, default) | alone.   CONDS=nom (default) | "nom rnd".
# Tags get the suffix _full automatically when MODE=full and the tag lacks it.
#   printf 'e1_a1_5000k|zip:franka_sim/models/sac_e1_a1/checkpoints/sac_5000000_steps.zip|franka_sim/models/sac_e1_a1/config.yaml\n' \
#     | docker exec -i -e SCENE=multi franka_ros2 taskset -c 20-23 chrt -i 0 bash /ros2_ws/src/franka_sim/runs/eval_all/e1/eval_e1.sh 4
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 CUDA_VISIBLE_DEVICES=
P=${1:-4}; MODE=${MODE:-full}; SCENE=${SCENE:-one}
O=franka_sim/runs/eval_all/traces/d2; EXTRA=""
if [ "$SCENE" = multi ]; then O=franka_sim/runs/eval_all/traces/e1_multi; EXTRA="--set obstacle.count_range=[2,3]"; fi
CH="3000:25 3025:25 3050:25 3075:25 4000:25 4025:25 4050:25 4075:25 4100:25 4125:25 4150:25 4175:25 4200:25 4225:25 4250:25 4275:25"
mkdir -p $O
while IFS='|' read tag spec cfg; do
  [ -z "$tag" ] && continue
  if [ "$MODE" = full ]; then case $tag in *_full) ;; *) tag=${tag}_full;; esac; fi
  for cond in ${CONDS:-nom}; do
    [ $cond = nom ] && X="--set randomization.enabled=false" || X=""
    for sc in static dynamic; do for ch in $CH; do
      out=$O/${tag}_${cond}_${sc}_${ch%%:*}.json
      [ -s $out ] || echo "python3 -m franka_sim.scripts.eval_constraints --controller $spec --config $cfg --mode $MODE --scenario $sc --seeds $ch $X $EXTRA --out $out"
    done; done
  done
done | xargs -d '\n' -P $P -I{} bash -c '{} 2>&1 | tail -1'
echo ALL DONE
