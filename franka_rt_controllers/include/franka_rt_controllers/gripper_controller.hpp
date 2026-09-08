#pragma once

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_rt_controllers {

/**
 * Minimal gripper controller — open / close, nothing else.
 *
 * Same idea as franka_example_controllers::GripperExampleController, but
 * instead of toggling forever on its own it just exposes the two motions and
 * lets the rest of the system decide when to use them:
 *
 *   openGripper()   →  <ns>/franka_gripper/move   (open to open_width)
 *   closeGripper()  →  <ns>/franka_gripper/grasp  (close with grasp_force)
 *
 * Callable from anywhere without duplicating the gripper logic:
 *   - from C++, on the controller instance (the two public methods);
 *   - from anywhere else, through the service ~/set_gripper
 *     (std_srvs/SetBool: data=true → close, data=false → open).
 *
 * It claims NO hardware interface and does nothing in update(), so it can be
 * loaded next to an arm controller without interfering with it.  All action
 * calls are asynchronous and happen outside the real-time path.
 */
class GripperController : public controller_interface::ControllerInterface {
 public:
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

  /// Open the fingers to `open_width`. Returns false if the goal was not sent.
  bool openGripper();

  /// Close the fingers on the object with `grasp_force`. False if not sent.
  bool closeGripper();

 private:
  void setGripperCb(const std_srvs::srv::SetBool::Request::SharedPtr request,
                    std_srvs::srv::SetBool::Response::SharedPtr response);

  rclcpp_action::Client<franka_msgs::action::Move>::SharedPtr move_client_;
  rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr grasp_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_gripper_service_;

  // Goal callbacks live as long as the controller, so the options structs are
  // members (same reasoning as GripperExampleController).
  rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions move_options_;
  rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions grasp_options_;

  double open_width_{0.08};
  double open_speed_{0.1};
  double grasp_width_{0.0};
  double grasp_speed_{0.05};
  double grasp_force_{40.0};
  double grasp_epsilon_inner_{0.005};
  double grasp_epsilon_outer_{0.08};
  double server_wait_s_{5.0};
};

}  // namespace franka_rt_controllers
