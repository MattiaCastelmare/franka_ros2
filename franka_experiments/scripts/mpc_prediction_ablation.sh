#!/usr/bin/env bash
# MPC ablation in Gazebo: the same bag replayed with mpc_human_mode:=static and :=prediction,
# then scripts/mpc_prediction_ablation.py on the runs. Inside the container, from /ros2_ws
# (human_logging writes experiment_logs/ relative to the working directory):
#
#   bash src/franka_experiments/scripts/mpc_prediction_ablation.sh [control_mode ...]
#
# control_mode: nominal = the MPC alone (default), cbf = shielded MPC (reactive CBF).
# Environment: BAG (/bags/cbf_final), DURATION_S (180: one pass of cbf_final after startup),
# REPEATS (1), DOMAIN (77: own ROS domain, nothing reaches a real robot's nodes).
set -u
BAG=${BAG:-/bags/cbf_final}
DURATION_S=${DURATION_S:-180}
REPEATS=${REPEATS:-1}
export ROS_DOMAIN_ID=${DOMAIN:-77}
MODES=("$@")
[ ${#MODES[@]} -eq 0 ] && MODES=(nominal)
STAMP=$(date +%Y%m%d_%H%M%S)
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
runs=()

for control_mode in "${MODES[@]}"; do
  controller=mpc; [ "$control_mode" = cbf ] && controller=mpc_cbf
  for repeat in $(seq 1 "$REPEATS"); do
    for human_mode in static prediction; do
      name=ablation_${STAMP}_${human_mode}_${repeat}
      echo "=== $controller $human_mode #$repeat -> experiment_logs/$controller/$name (${DURATION_S} s)"
      timeout -s INT -k 30 "$DURATION_S" ros2 launch franka_experiments easy_torque.launch.py \
        real:=false commander:=mpc control_mode:="$control_mode" human:=true human_bag:="$BAG" \
        mpc_human_mode:="$human_mode" run_name:="$name" > "/tmp/${controller}_$name.launch.log" 2>&1
      # Gazebo and the bag player sometimes outlive the launch
      pkill -INT -f 'gz sim|ign gazebo|ros2 bag play' 2>/dev/null; sleep 5
      pkill -KILL -f 'gz sim|ign gazebo|ros2 bag play' 2>/dev/null; sleep 2
      runs+=("experiment_logs/$controller/$name")
    done
  done
done

python3 "$SCRIPT_DIR/mpc_prediction_ablation.py" "${runs[@]}" --csv "experiment_logs/ablation_${STAMP}.csv"
