#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_rt_controllers {

class RtTorqueController : public controller_interface::ControllerInterface {
 public:
  static constexpr size_t kNumJoints = 7;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  struct TorqueInput {
    std::array<double, kNumJoints> tau{};
    bool received{false};
  };

  // q̈_safe transport — parallel, lock-free, gemello di TorqueInput. Porta il
  // riferimento di accelerazione (uscita CBF) per il feedback di velocità a
  // 1 kHz. stamp = istante di ricezione [s] (node clock), usato dal check di
  // freschezza in update().
  struct AccelInput {
    std::array<double, kNumJoints> qddot{};
    bool   received{false};
    double stamp{0.0};
  };

  void updateJointStates();
  void commandCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void accelCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::string arm_id_;
  std::vector<std::string> joint_names_;
  std::string command_topic_;
  std::string accel_topic_;
  bool is_gazebo_{false};

  // ── Feedback di velocità (vedi on_configure per la semantica) ────────────
  std::array<double, kNumJoints> d_gains_{};   // Kd per giunto [N·m/(rad/s)]
  double e_max_{1.0};                           // clamp errore velocità [rad/s]
  double accel_timeout_{0.1};                   // finestra freschezza q̈_safe [s]

  // ── Stato RT-only (aggiornato dentro update(); mai condiviso tra thread) ──
  std::array<double, kNumJoints> q_{};          // posizione misurata
  std::array<double, kNumJoints> dq_{};         // velocità misurata
  std::array<double, kNumJoints> qdot_des_{};   // riferimento velocità integrato

  realtime_tools::RealtimeBuffer<TorqueInput> command_buf_;
  realtime_tools::RealtimeBuffer<AccelInput>  accel_buf_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr accel_sub_;
};

}  // namespace franka_rt_controllers
