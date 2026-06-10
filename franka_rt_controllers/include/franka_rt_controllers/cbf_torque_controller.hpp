#pragma once
#include <array>
#include <atomic>
#include <string>
#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace franka_rt_controllers {

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CBFTorqueController : public controller_interface::ControllerInterface {
 public:
  using Vec7 = Eigen::Matrix<double, 7, 1>;

  // Single setpoint frame from Python.
  struct Setpoint {
    Vec7 q_d   = Vec7::Zero();
    Vec7 dq_d  = Vec7::Zero();
    Vec7 qddot = Vec7::Zero();
  };

  // FOH interpolation state written by the subscriber callback, read by update().
  // Stores both the previous and the latest setpoint so update() can lerp between them.
  struct InterpState {
    Setpoint prev;           // setpoint active before the last message
    Setpoint next;           // latest received setpoint
    double   arrival_s{0.0}; // wall-clock time of the last message
    double   period_s{0.01}; // EMA-estimated inter-message period (default 100 Hz)
  };

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration()   const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
  CallbackReturn on_init()      override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate  (const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

 private:
  void read_joint_states();

  std::string arm_id_;
  static constexpr int kNJ = 7;

  Vec7 q_{Vec7::Zero()};
  Vec7 dq_{Vec7::Zero()};
  Vec7 kd_{Vec7::Constant(20.0)};
  Vec7 kp_local_{Vec7::Zero()};
  Vec7 kd_local_{Vec7::Zero()};
  Vec7 last_q_d_{Vec7::Zero()};

  // Optional 1st-order IIR low-pass on qddot_ff (applied after FOH interpolation).
  // alpha=0 → no filtering; alpha∈(0,1) → smoothing pole at -(1-alpha)/dt Hz.
  double lpf_alpha_{0.0};
  Vec7   qddot_lpf_{Vec7::Zero()};

  double qddot_timeout_s_{0.1};
  std::string qddot_safe_topic_;
  std::string q_des_topic_;

  // Pinocchio
  pinocchio::Model     pin_model_;
  pinocchio::Data      pin_data_;
  Eigen::VectorXd      q_pin_;
  Eigen::VectorXd      dq_pin_;
  Eigen::VectorXd      qddot_pin_;
  std::array<int, kNJ> arm_q_idx_{};
  std::array<int, kNJ> arm_v_idx_{};

  // ── Legacy qddot_safe (Float64MultiArray) ───────────────────────────────
  using MsgT = std_msgs::msg::Float64MultiArray;
  realtime_tools::RealtimeBuffer<Vec7>  qddot_buf_;
  // Written by the (non-RT) subscriber thread, read by the 1 kHz update():
  // atomic to avoid a data race (lock-free for double on x86_64).
  std::atomic<double>                   last_msg_stamp_s_{0.0};
  rclcpp::Subscription<MsgT>::SharedPtr sub_;

  // ── JointState setpoint with FOH interpolation ───────────────────────────
  realtime_tools::RealtimeBuffer<InterpState>                    interp_buf_;
  std::atomic<double>                                            sp_stamp_s_{0.0};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  sp_sub_;
};

}  // namespace franka_rt_controllers
