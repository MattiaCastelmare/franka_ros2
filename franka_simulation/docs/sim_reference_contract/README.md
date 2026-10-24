# Reference contract — pipeline A on fake hardware (Phase 0)

Captured 2026-10-21 on branch `sim-gz-cbf-experiments` (= `humble-mattia` @ `94cd7b0`, no code changes),
inside the `franka_ros2` container, `ROS_DOMAIN_ID=77`, with:

    ros2 launch franka_experiments torque_control_stack.launch.py \
        use_fake_hardware:=true enable_camera:=false rt_pin_cpu:='' trajectory_overlay_window:=false

Everything else at the launch defaults (`launch_defaults.yaml` + `franka_bringup/config/franka.config.yaml`,
which wins for `namespace: NS_1` and `load_gripper: "false"`). Camera off so no D455 is grabbed; the
OpenCV windows run with `QT_QPA_PLATFORM=offscreen`; `experiment_logger` wrote to `/tmp` (`FRANKA_LOGS_DIR`).

Reproduce: `bash run_capture.sh [out_dir]` (inside the container). `capture_contract.py` is the capture
tool; Phase 1's `launch_testing` diff reuses it.

| file | content |
|---|---|
| `fake_hw/graph.json` | 17 nodes, 45 topics: per-node pub/sub/services, per-topic types and endpoint QoS |
| `fake_hw/tf.json` | 22 frames on `/tf` + `/tf_static` with parent, static flag, transform |
| `fake_hw/params/*.yaml` | `ros2 param dump` of the 11 pipeline-A nodes (+ controller_manager, RSP, JSP) |
| `fake_hw/controllers.txt`, `hardware_interfaces.txt` | `ros2 control list_*` |
| `fake_hw/rates.txt` | `ros2 topic hz` of the safety-path topics |
| `fake_hw/launch.log` | the launch output |

**QoS depth is not discoverable** (rmw_fastrtps reports 0 for every endpoint); reliability and durability
are. The depths below come from the code.

## Safety path (what Phase 1's diff is restricted to)

| topic | type | publishers (reliability) | subscribers (reliability) | rate fake-HW [Hz] |
|---|---|---|---|---|
| `/NS_1/franka/joint_states` | `sensor_msgs/msg/JointState` | /NS_1/joint_state_broadcaster (R) | /NS_1/joint_state_publisher (R), /NS_1/pentagon_qddot_commander (R), /cbf_safety_filter (R), /qddot_to_torque (R) | 100 |
| `/NS_1/joint_states` | `sensor_msgs/msg/JointState` | /NS_1/joint_state_publisher (R), /_ros2cli_295006 (R) | /NS_1/move_group_private_104362302338624 (B), /NS_1/pentagon_qddot_commander (R), /NS_1/robot_state_publisher (B), /experiment_logger (R) | 40 |
| `/NS_1/qddot_nom` | `std_msgs/msg/Float64MultiArray` | /NS_1/pentagon_qddot_commander (R) | /cbf_safety_filter (R), /experiment_logger (R) | 100 |
| `/NS_1/qddot_safe` | `std_msgs/msg/Float64MultiArray` | /cbf_safety_filter (R) | /NS_1/rt_torque_controller (B), /experiment_logger (R), /qddot_to_torque (R) | 300 |
| `/NS_1/torque_cmd` | `std_msgs/msg/Float64MultiArray` | /qddot_to_torque (R) | /NS_1/rt_torque_controller (B), /experiment_logger (R) | 300 |
| `/NS_1/cbf_status` | `std_msgs/msg/Float64MultiArray` | /cbf_safety_filter (R) | /NS_1/pentagon_qddot_commander (R), /experiment_logger (R) | 300 |
| `/cbf/per_link_distances` | `franka_msgs/msg/MultiLinkDistance` | /real_time_distance (B) | /cbf_safety_filter (B), /experiment_logger (B) | — |
| `/NS_1/iso_safety` | `std_msgs/msg/Float64MultiArray` | — | /NS_1/pentagon_qddot_commander (R), /cbf_safety_filter (R), /experiment_logger (R) | — |
| `/NS_1/torque_saturation` | `std_msgs/msg/Float64MultiArray` | /qddot_to_torque (R) | /experiment_logger (R) | — |
| `/NS_1/ee_actual` | `geometry_msgs/msg/PointStamped` | /NS_1/pentagon_qddot_commander (R) | /trajectory_overlay (R) | — |
| `/NS_1/ee_desired` | `geometry_msgs/msg/PointStamped` | /NS_1/pentagon_qddot_commander (R) | /trajectory_overlay (R) | — |
| `/tf` | `tf2_msgs/msg/TFMessage` | /NS_1/robot_state_publisher (R) | /NS_1/transform_listener_impl_5eeabdf58310 (R), /real_time_distance (R) | — |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | /NS_1/robot_state_publisher (R), /world_to_robot_base_tf (R) | /NS_1/transform_listener_impl_5eeabdf58310 (R), /real_time_distance (R) | — |

(R = RELIABLE, B = BEST_EFFORT.) Depths from the code: `cbf_safety_filter` subscribes everything with
depth 1 (`cbf_safety_filter.py:345-354`), publishes `qddot_safe` depth 10; `RtTorqueController` uses
`SensorDataQoS().keep_last(1)` on both inputs (`rt_torque_controller.cpp:474,502`).

**Rates are fake-hardware rates.** On the real FR3 the controller_manager runs at 1000 Hz, so
`/NS_1/franka/joint_states` is 1 kHz; fake hardware runs at 100 Hz (`launch_support.py:278`). Gazebo must
reproduce the **hardware** number (1 kHz, `update_rate` from the plugin's YAML).

`/NS_1/joint_states` has two publishers: the bringup's `joint_state_publisher` (30 Hz republisher, measured
40 Hz here) and the `finger_state_publisher` (`ros2 topic pub`, 10 Hz, fingers only, for MoveIt).

## TF tree (accelerometer frames omitted; all are static children of their link)

| frame | parent | static | xyz |
|---|---|---|---|
| `base` | `world` | yes | [0.0, 0.0, 0.0] |
| `fr3_link0` | `base` | yes | [0.0, 0.0, 0.0] |
| `fr3_link1` | `fr3_link0` | no | [0.0, 0.0, 0.333] |
| `fr3_link2` | `fr3_link1` | no | [0.0, 0.0, 0.0] |
| `fr3_link3` | `fr3_link2` | no | [0.0, -0.316, 0.0] |
| `fr3_link4` | `fr3_link3` | no | [0.0825, 0.0, 0.0] |
| `fr3_link5` | `fr3_link4` | no | [-0.0825, 0.384, 0.0] |
| `fr3_link6` | `fr3_link5` | no | [0.0, 0.0, 0.0] |
| `fr3_link7` | `fr3_link6` | no | [0.088, 0.0, 0.0] |
| `fr3_link8` | `fr3_link7` | yes | [0.0, 0.0, 0.107] |

TF is on the **absolute** `/tf` and `/tf_static`, not `/NS_1/tf`: the comment at
`torque_control_stack.launch.py:536-540` ("publishes TF on /<ns>/tf") is wrong. No hand frames:
`load_gripper` resolves to `"false"` from `franka.config.yaml`, so the bringup URDF has no hand even though
`launch_defaults.yaml` says `true` (`_DEFAULTS = {**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}`,
`torque_control_stack.launch.py:104`). With the camera on, the stack adds `base → camera_link` from
`camera_link_extrinsics.yaml` and the RealSense driver adds its own subtree; neither is read by
`real_time_distance`, which takes `base → camera_color_optical_frame` from `camera_extrinsics.yaml`.

## Controllers

`joint_state_broadcaster` (active) and `rt_torque_controller` (active), all 7 `fr3_jointN/effort`
command interfaces claimed. No `franka_robot_state_broadcaster` (not spawned on fake hardware).
