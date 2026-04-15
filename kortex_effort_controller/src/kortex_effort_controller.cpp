// Copyright 2024, Your Name
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kortex_effort_controller/kortex_effort_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace kortex_effort_controller
{

// Default joint names matching the Gen3 7-DOF URDF
static const std::vector<std::string> DEFAULT_JOINT_NAMES = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};

// Conservative torque limits (Nm): joints 1-4 large actuators, 5-7 small
static const std::vector<double> DEFAULT_TORQUE_LIMITS = {
  39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0};

KortexEffortController::KortexEffortController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn KortexEffortController::on_init()
{
  try {
    // Declare parameters
    auto_declare<std::vector<std::string>>("joints", DEFAULT_JOINT_NAMES);
    auto_declare<std::vector<double>>("torque_limits", DEFAULT_TORQUE_LIMITS);
    auto_declare<std::string>("command_topic", "~/joint_torque_command");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KortexEffortController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_   = get_node()->get_parameter("joints").as_string_array();
  torque_limits_ = get_node()->get_parameter("torque_limits").as_double_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (torque_limits_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'torque_limits' size (%zu) must match 'joints' size (%zu).",
      torque_limits_.size(), joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  desired_torques_.assign(joint_names_.size(), 0.0);
  has_command_ = false;

  // Subscribe to torque commands
  auto topic = get_node()->get_parameter("command_topic").as_string();
  torque_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    topic, rclcpp::SystemDefaultsQoS(),
    std::bind(
      &KortexEffortController::torque_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "KortexEffortController configured for %zu joints, subscribed to %s.",
    joint_names_.size(), topic.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
KortexEffortController::command_interface_configuration() const
{
  // Claim the effort command interface for every joint
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
KortexEffortController::state_interface_configuration() const
{
  // Read position and effort state for all joints (useful for logging/safety)
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/effort");
  }
  return config;
}

controller_interface::CallbackReturn KortexEffortController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Zero commands on activation — don't move until a command arrives
  std::lock_guard<std::mutex> lock(torque_mutex_);
  desired_torques_.assign(joint_names_.size(), 0.0);
  has_command_ = false;

  // Write zeros to command interfaces immediately
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces_[i].set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "KortexEffortController activated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KortexEffortController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Zero all commands on deactivation for safety
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces_[i].set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "KortexEffortController deactivated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type KortexEffortController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<double> torques;
  {
    std::lock_guard<std::mutex> lock(torque_mutex_);
    torques = desired_torques_;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces_[i].set_value(torques[i]);
  }

  return controller_interface::return_type::OK;
}

void KortexEffortController::torque_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != joint_names_.size()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Expected %zu torque values, got %zu. Ignoring.",
      joint_names_.size(), msg->data.size());
    return;
  }

  std::vector<double> clamped(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    double limit = torque_limits_[i];
    double tau   = msg->data[i];
    double safe  = std::max(-limit, std::min(limit, tau));
    if (std::abs(safe) < std::abs(tau)) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Joint %s torque %.2f Nm clamped to %.2f Nm", joint_names_[i].c_str(), tau, safe);
    }
    clamped[i] = safe;
  }

  std::lock_guard<std::mutex> lock(torque_mutex_);
  desired_torques_ = clamped;
  has_command_ = true;
}

}  // namespace kortex_effort_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  kortex_effort_controller::KortexEffortController,
  controller_interface::ControllerInterface)
