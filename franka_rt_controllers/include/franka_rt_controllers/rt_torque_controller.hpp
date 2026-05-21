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

  std::string arm_id_;
  std::vector<std::string> joint_names_;
  std::string command_topic_;
  double lpf_alpha_{1.0};
  bool is_gazebo_{false};

  // Per-joint effort limits [N·m], scaled from {87,87,87,87,12,12,12}
  std::array<double, kNumJoints> tau_max_{};

  realtime_tools::RealtimeBuffer<TorqueInput> command_buf_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;

  // RT-only state: filtered command (LPF state, touched only in update())
  std::array<double, kNumJoints> tau_filtered_{};

  void commandCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace franka_rt_controllers
