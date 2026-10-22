#!/usr/bin/env bash
# Phase 0 reference contract: torque_control_stack on FAKE hardware, captured.
# Run inside the franka_ros2 container:
#   bash /ros2_ws/src/franka_simulation/docs/sim_reference_contract/run_capture.sh
# camera off (no D455 grab), GUIs off / offscreen, logger output to /tmp, isolated DDS domain.
set +u
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
export QT_QPA_PLATFORM=offscreen
export FRANKA_LOGS_DIR=/tmp/contract_capture_logs
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:-$HERE/fake_hw}
LOG=/tmp/contract_capture_launch.log
rm -rf "$OUT"; mkdir -p "$OUT"

setsid ros2 launch franka_experiments torque_control_stack.launch.py \
  use_fake_hardware:=true enable_camera:=false "rt_pin_cpu:=''" \
  trajectory_overlay_window:=false > "$LOG" 2>&1 &
LPID=$!
ros2 daemon stop >/dev/null 2>&1
sleep "${WAIT_S:-40}"

python3 "$HERE/capture_contract.py" "$OUT" --wait 5 \
  --param-node /cbf_safety_filter \
  --param-node /qddot_to_torque \
  --param-node /real_time_distance \
  --param-node /NS_1/pentagon_qddot_commander \
  --param-node /experiment_logger \
  --param-node /NS_1/rt_torque_controller \
  --param-node /NS_1/joint_state_broadcaster \
  --param-node /NS_1/controller_manager \
  --param-node /NS_1/robot_state_publisher \
  --param-node /NS_1/joint_state_publisher \
  --param-node /trajectory_overlay
ros2 control list_controllers -c /NS_1/controller_manager > "$OUT/controllers.txt" 2>&1
ros2 control list_hardware_interfaces -c /NS_1/controller_manager > "$OUT/hardware_interfaces.txt" 2>&1
for t in /NS_1/franka/joint_states /NS_1/joint_states /NS_1/qddot_nom /NS_1/qddot_safe \
         /NS_1/torque_cmd /NS_1/cbf_status; do
  echo "== $t" >> "$OUT/rates.txt"
  timeout 6 ros2 topic hz "$t" --window 200 2>&1 | tail -3 >> "$OUT/rates.txt"
done

kill -INT -- -"$LPID" 2>/dev/null
sleep 8
kill -KILL -- -"$LPID" 2>/dev/null
cp "$LOG" "$OUT/launch.log"
ros2 daemon stop >/dev/null 2>&1
echo "done -> $OUT"
