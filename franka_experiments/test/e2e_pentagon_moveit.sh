#!/bin/bash
# End-to-end check of the MoveIt-based pentagon_qddot_commander:
#   move_group (franka_fr3_moveit_config) + fake joint states + the node.
# Run inside the franka_ros2 container:  bash e2e_pentagon_moveit.sh
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

LOGDIR=$(mktemp -d /tmp/pentagon_e2e.XXXX)
echo "logs: $LOGDIR"

cleanup() { kill $(jobs -p) 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT

# 1) move_group (URDF/SRDF from franka_description, group fr3_arm)
ros2 launch franka_fr3_moveit_config move_group.launch.py \
    robot_ip:=dont-care use_fake_hardware:=true fake_sensor_commands:=false \
    load_gripper:=true namespace:=/ \
    > "$LOGDIR/move_group.log" 2>&1 &
sleep 8

# 1b) robot_state_publisher: TF base→fr3_link0→…→fr3_hand_tcp (FK answers in
#     fr3_link0 need this transform; the real bringup always provides it)
xacro /ros2_ws/install/franka_description/share/franka_description/robots/fr3/fr3.urdf.xacro \
    ros2_control:=false hand:=true robot_ip:=dont-care use_fake_hardware:=true \
    > "$LOGDIR/fr3.urdf"
python3 -c "
import yaml
urdf = open('$LOGDIR/fr3.urdf').read()
yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': urdf}}},
               open('$LOGDIR/rsp_params.yaml', 'w'))
"
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args --params-file "$LOGDIR/rsp_params.yaml" \
    > "$LOGDIR/rsp.log" 2>&1 &

# 2) fake joint states at the FR3 ready pose
ros2 topic pub -r 50 /joint_states sensor_msgs/msg/JointState "{
  name: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
  position: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]}" \
    > /dev/null 2>&1 &
sleep 2

# 3) the refactored node (small pentagon near the ready-pose EE position)
ros2 run franka_experiments pentagon_qddot_commander --ros-args \
    -p joint_state_topic:=/joint_states \
    -p center_xyz:="[0.45, 0.0, 0.45]" -p radius:=0.08 -p cycle_time:=6.0 \
    > "$LOGDIR/node.log" 2>&1 &
sleep 15

echo "──── node log ────"
cat "$LOGDIR/node.log"
echo "──── qddot_nom sample ────"
timeout 5 ros2 topic echo --once /NS_1/qddot_nom 2>/dev/null
echo "──── q_des_state sample ────"
timeout 5 ros2 topic echo --once /NS_1/q_des_state 2>/dev/null | head -25
echo "──── planned trajectory points ────"
timeout 5 ros2 topic echo --once --field points /pentagon_qddot_commander/planned_trajectory 2>/dev/null | head -5
echo "──── rates ────"
timeout 6 ros2 topic hz --window 200 /NS_1/qddot_nom 2>/dev/null | head -2
