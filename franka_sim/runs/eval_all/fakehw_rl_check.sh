#!/bin/bash
# Fake-HW check of an ONNX policy through the canonical torque stack (motion_source:=rl).   fakehw_rl_check.sh MODEL.onnx
source /ros2_ws/install/setup.bash
M=$1; L=/tmp/rlfake.log
timeout 150 ros2 launch franka_experiments test_rl_fake.launch.py rl_onnx_model:=$M > $L 2>&1 &
for i in $(seq 1 60); do grep -q "activated.*rt_torque_controller" $L && break; grep -q "Caught exception in launch" $L && break; sleep 1; done
grep -E "Caught exception|process has died" $L && { echo "LAUNCH FAILED"; tail -20 $L; exit 1; }
sleep 10
echo "== controllers"; ros2 control list_controllers -c /NS_1/controller_manager 2>&1 | head
echo "== nodes"; ros2 node list 2>/dev/null | sort
for t in /NS_1/joint_states /NS_1/qddot_nom /NS_1/qddot_safe /NS_1/torque_cmd /NS_1/rl_status; do
  echo "== hz $t"; timeout 6 ros2 topic hz $t 2>&1 | grep -m1 "average rate"; done
echo "== rl_status [infer_ms, tick_ms, d_min, dist, target_idx, gate]"; timeout 5 ros2 topic echo --once /NS_1/rl_status 2>&1 | head -12
echo "== qddot_nom"; timeout 5 ros2 topic echo --once /NS_1/qddot_nom 2>&1 | head -12
echo "== qddot_safe"; timeout 5 ros2 topic echo --once /NS_1/qddot_safe 2>&1 | head -12
echo "== torque_cmd"; timeout 5 ros2 topic echo --once /NS_1/torque_cmd 2>&1 | head -12
echo "== check_topics.sh rl"; cd /ros2_ws/src/franka_experiments && timeout 120 ./test/scripts/check_topics.sh rl 2>&1 | tail -30
echo "== commander log"; grep -E "rl_policy_commander\]" $L | tail -8
echo "== errors in launch log"; grep -iE "error|exception|died" $L | head -10
pkill -f "ros2 launch franka_experiments test_rl_fake" ; sleep 3; echo DONE
