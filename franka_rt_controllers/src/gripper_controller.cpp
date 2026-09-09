#include <franka_rt_controllers/gripper_controller.hpp>

#include <chrono>
#include <cstdio>
#include <exception>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace franka_rt_controllers {

// ═══════════════════════════════════════════════════════════════════════════
//  Interfaces — none: this controller only talks to the gripper action server
// ═══════════════════════════════════════════════════════════════════════════

controller_interface::InterfaceConfiguration
GripperController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
GripperController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type GripperController::update(const rclcpp::Time&,
                                                            const rclcpp::Duration&) {
  return controller_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn GripperController::on_init() {
  try {
    auto_declare<double>("open_width", open_width_);
    auto_declare<double>("open_speed", open_speed_);
    auto_declare<double>("grasp_width", grasp_width_);
    auto_declare<double>("grasp_speed", grasp_speed_);
    auto_declare<double>("grasp_force", grasp_force_);
    auto_declare<double>("grasp_epsilon_inner", grasp_epsilon_inner_);
    auto_declare<double>("grasp_epsilon_outer", grasp_epsilon_outer_);
    auto_declare<double>("server_wait_s", server_wait_s_);
  } catch (const std::exception& e) {
    fprintf(stderr, "GripperController: exception in on_init(): %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperController::on_configure(const rclcpp_lifecycle::State&) {
  auto node = get_node();

  open_width_           = node->get_parameter("open_width").as_double();
  open_speed_           = node->get_parameter("open_speed").as_double();
  grasp_width_          = node->get_parameter("grasp_width").as_double();
  grasp_speed_          = node->get_parameter("grasp_speed").as_double();
  grasp_force_          = node->get_parameter("grasp_force").as_double();
  grasp_epsilon_inner_  = node->get_parameter("grasp_epsilon_inner").as_double();
  grasp_epsilon_outer_  = node->get_parameter("grasp_epsilon_outer").as_double();
  server_wait_s_        = node->get_parameter("server_wait_s").as_double();

  // franka_gripper lives in the same namespace as the controller manager.
  // LifecycleNode has no get_fully_qualified_name() in Humble, so the node path
  // is composed from namespace + name.
  const std::string ns = node->get_namespace();
  const std::string ns_prefix = (ns == "/" ? "" : ns);
  const std::string prefix = ns_prefix + "/franka_gripper";
  const std::string node_path = ns_prefix + "/" + node->get_name();

  move_client_  = rclcpp_action::create_client<franka_msgs::action::Move>(
      node, prefix + "/move");
  grasp_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
      node, prefix + "/grasp");
  if (!move_client_ || !grasp_client_) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create the gripper action clients.");
    return CallbackReturn::ERROR;
  }

  // The single entry point for the rest of the system: true = close, false = open.
  set_gripper_service_ = node->create_service<std_srvs::srv::SetBool>(
      "~/set_gripper",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
             std_srvs::srv::SetBool::Response::SharedPtr response) {
        setGripperCb(request, response);
      });

  // Log the result of every goal; that is all the feedback we need here.
  move_options_.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult&
                 result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
          RCLCPP_INFO(get_node()->get_logger(), "Gripper opened.");
        } else {
          RCLCPP_ERROR(get_node()->get_logger(), "Open failed: %s",
                       result.result ? result.result->error.c_str() : "no result");
        }
      };
  grasp_options_.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult&
                 result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
          RCLCPP_INFO(get_node()->get_logger(), "Gripper closed on the object.");
        } else {
          RCLCPP_ERROR(get_node()->get_logger(), "Close failed: %s",
                       result.result ? result.result->error.c_str() : "no result");
        }
      };

  RCLCPP_INFO(node->get_logger(),
              "GripperController configured: actions %s/{move,grasp}, service "
              "%s/set_gripper (true = close, false = open)",
              prefix.c_str(), node_path.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperController::on_activate(const rclcpp_lifecycle::State&) {
  const auto wait = std::chrono::duration<double>(server_wait_s_);
  if (!move_client_->wait_for_action_server(
          std::chrono::duration_cast<std::chrono::nanoseconds>(wait)) ||
      !grasp_client_->wait_for_action_server(
          std::chrono::duration_cast<std::chrono::nanoseconds>(wait))) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "franka_gripper move/grasp action servers not available after %.1fs. "
                 "Is the gripper node running (load_gripper:=true, real hardware)?",
                 server_wait_s_);
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperController::on_deactivate(const rclcpp_lifecycle::State&) {
  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Open / close
// ═══════════════════════════════════════════════════════════════════════════

bool GripperController::openGripper() {
  if (!move_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot open: move action server not ready.");
    return false;
  }
  franka_msgs::action::Move::Goal goal;
  goal.width = open_width_;
  goal.speed = open_speed_;
  RCLCPP_INFO(get_node()->get_logger(), "Opening gripper to %.3f m", goal.width);
  return move_client_->async_send_goal(goal, move_options_).valid();
}

bool GripperController::closeGripper() {
  if (!grasp_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot close: grasp action server not ready.");
    return false;
  }
  franka_msgs::action::Grasp::Goal goal;
  goal.width = grasp_width_;
  goal.speed = grasp_speed_;
  goal.force = grasp_force_;
  goal.epsilon.inner = grasp_epsilon_inner_;
  goal.epsilon.outer = grasp_epsilon_outer_;
  RCLCPP_INFO(get_node()->get_logger(), "Closing gripper on %.3f m with %.1f N",
              goal.width, goal.force);
  return grasp_client_->async_send_goal(goal, grasp_options_).valid();
}

void GripperController::setGripperCb(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response) {
  const bool sent = request->data ? closeGripper() : openGripper();
  response->success = sent;
  response->message = sent ? (request->data ? "close goal sent" : "open goal sent")
                           : "goal not sent (gripper action server not ready)";
}

}  // namespace franka_rt_controllers

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_rt_controllers::GripperController,
                       controller_interface::ControllerInterface)
