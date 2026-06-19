#!/bin/bash
# End-to-end check of the joint-space pentagon_qddot_commander:
#   fake joint states + the node (NO move_group — the node does its own
#   offline Pinocchio IK and needs no planning services).
# It also exercises the CBF virtual-time freeze by publishing /NS_1/cbf_status.
# Run inside the franka_ros2 container:  bash e2e_pentagon_moveit.sh
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

LOGDIR=$(mktemp -d /tmp/pentagon_e2e.XXXX)
echo "logs: $LOGDIR"

cleanup() { kill $(jobs -p) 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT

# 1) fake joint states at the FR3 ready pose
ros2 topic pub -r 50 /joint_states sensor_msgs/msg/JointState "{
  name: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
  position: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
  velocity: [0, 0, 0, 0, 0, 0, 0]}" \
    > /dev/null 2>&1 &
sleep 2

# 2) the joint-space node (small pentagon near the ready-pose EE position)
ros2 run franka_experiments pentagon_qddot_commander --ros-args \
    -p joint_state_topic:=/joint_states \
    -p center_xyz:="[0.4, 0.0, 0.5]" -p radius:=0.15 -p cycle_time:=7.0 \
    -p warmup_s:=2.0 -p approach_time:=2.0 -p timing_law:=linear \
    > "$LOGDIR/node.log" 2>&1 &
sleep 9

echo "──── node log (start + phase progression) ────"
grep -E "Trajectory start|WARMUP|APPROACH|TRACK|exceeds" "$LOGDIR/node.log" | head -12
echo "──── qddot_nom sample ────"
timeout 5 ros2 topic echo --once /NS_1/qddot_nom 2>/dev/null
echo "──── q_des_state sample ────"
timeout 5 ros2 topic echo --once /NS_1/q_des_state 2>/dev/null | head -18
echo "──── publish CBF active (n_c=2) for 3 s — virtual time should freeze ────"
ros2 topic pub -r 20 /NS_1/cbf_status std_msgs/msg/Float64MultiArray "{data: [2.0, 0.01]}" \
    > /dev/null 2>&1 &
CBF_PID=$!
sleep 3
kill $CBF_PID 2>/dev/null
sleep 3
echo "──── TRACK timeline (τ freezes while CBF active, then resumes) ────"
grep "TRACK" "$LOGDIR/node.log" | tail -10
echo "──── rates ────"
timeout 6 ros2 topic hz --window 200 /NS_1/qddot_nom 2>/dev/null | head -2
