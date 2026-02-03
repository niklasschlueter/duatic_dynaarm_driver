/*
 * Copyright 2026 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <duatic_dynaarm_controllers/safety_monitor_controller.hpp>
#include <duatic_dynaarm_controllers/ros2_control_compat.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace duatic_dynaarm_controllers
{

SafetyMonitorController::SafetyMonitorController() : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration SafetyMonitorController::command_interface_configuration() const
{
  // This controller does NOT claim any command interfaces
  // It monitors state and triggers freeze via service calls
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration SafetyMonitorController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;

  // Always claim position interfaces
  for (const auto& joint : joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }

  // Claim velocity interfaces if velocity checking or external torque monitoring is enabled
  if (params_.check_velocity || params_.check_external_torque) {
    for (const auto& joint : joints) {
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
  }

  // Claim effort and acceleration interfaces if external torque monitoring is enabled
  if (params_.check_external_torque) {
    for (const auto& joint : joints) {
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
      config.names.emplace_back(joint + "/acceleration_commanded");
    }
  }

  return config;
}

controller_interface::CallbackReturn SafetyMonitorController::on_init()
{
  try {
    param_listener_ = std::make_unique<safety_monitor_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyMonitorController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Refresh parameters
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  // Validate parameters
  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  const size_t num_joints = params_.joints.size();

  if (params_.check_velocity && params_.max_joint_velocities.size() != num_joints) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "check_velocity is enabled but max_joint_velocities size (%zu) does not match joints size (%zu)",
                 params_.max_joint_velocities.size(), num_joints);
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Validate external torque parameters
  // Treat single-element [0.0] array as "use single threshold" (same as empty)
  const bool use_per_joint_thresholds = params_.check_external_torque &&
                                         params_.external_torque_thresholds.size() > 1;
  if (use_per_joint_thresholds && params_.external_torque_thresholds.size() != num_joints) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "check_external_torque is enabled but external_torque_thresholds size (%zu) does not match joints size (%zu)",
                 params_.external_torque_thresholds.size(), num_joints);
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Load Pinocchio model to read joint limits from URDF
  if (params_.check_joint_limits || params_.check_workspace || params_.check_external_torque) {
    try {
      pinocchio::urdf::buildModelFromXML(get_robot_description(), pinocchio_model_);
      pinocchio_data_ = pinocchio::Data(pinocchio_model_);

      // Extract joint limits from URDF with tolerance
      joint_lower_limits_.clear();
      joint_upper_limits_.clear();

      for (const auto& joint_name : params_.joints) {
        auto joint_id = pinocchio_model_.getJointId(joint_name);
        if (joint_id == 0) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Joint '%s' not found in URDF model", joint_name.c_str());
          return controller_interface::CallbackReturn::FAILURE;
        }

        const auto& joint = pinocchio_model_.joints[joint_id];
        const int idx = joint.idx_q();

        // Apply tolerance for safety margin
        double lower_limit = pinocchio_model_.lowerPositionLimit[idx] + params_.joint_limit_tolerance;
        double upper_limit = pinocchio_model_.upperPositionLimit[idx] - params_.joint_limit_tolerance;

        joint_lower_limits_.push_back(lower_limit);
        joint_upper_limits_.push_back(upper_limit);

        RCLCPP_INFO(get_node()->get_logger(),
                    "Joint '%s': limits [%.3f, %.3f] rad (with %.3f rad tolerance)",
                    joint_name.c_str(), lower_limit, upper_limit, params_.joint_limit_tolerance);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to load robot model: %s", e.what());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  // Create service client for controller switching
  switch_controller_client_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

  // Create status publisher
  status_pub_ = get_node()->create_publisher<std_msgs::msg::String>("~/status", 10);
  status_pub_rt_ = std::make_unique<StatusPublisher>(status_pub_);

  // Get end-effector frame ID if workspace checking is enabled
  if (params_.check_workspace) {
    if (!pinocchio_model_.existFrame(params_.end_effector_frame)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "End-effector frame '%s' not found in robot model. Available frames:",
                   params_.end_effector_frame.c_str());
      for (const auto& frame : pinocchio_model_.frames) {
        RCLCPP_ERROR(get_node()->get_logger(), "  - %s", frame.name.c_str());
      }
      return controller_interface::CallbackReturn::FAILURE;
    }

    ee_frame_id_ = pinocchio_model_.getFrameId(params_.end_effector_frame);
    RCLCPP_INFO(get_node()->get_logger(),
                "Workspace checking enabled for frame '%s' (ID: %d)",
                params_.end_effector_frame.c_str(), ee_frame_id_);
  }

  // Initialize torque residual filtering buffers
  if (params_.check_external_torque) {
    torque_residual_history_.resize(num_joints);
    filtered_torque_residuals_.resize(num_joints, 0.0);
    for (auto& history : torque_residual_history_) {
      history.clear();
    }
    RCLCPP_INFO(get_node()->get_logger(),
                "External torque monitoring enabled with filter window size: %d",
                params_.torque_residual_filter_window);
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Safety Monitor configured for %zu joints. Freeze controller: '%s'",
              num_joints, params_.freeze_controller_name.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyMonitorController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Clear interface vectors
  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();
  joint_effort_state_interfaces_.clear();
  joint_acceleration_state_interfaces_.clear();

  // Get ordered position interfaces
  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, hardware_interface::HW_IF_POSITION,
          joint_position_state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not get ordered position state interfaces");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Get ordered velocity interfaces if checking is enabled
  if (params_.check_velocity || params_.check_external_torque) {
    if (!controller_interface::get_ordered_interfaces(
            state_interfaces_, params_.joints, hardware_interface::HW_IF_VELOCITY,
            joint_velocity_state_interfaces_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not get ordered velocity state interfaces");
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  // Get ordered effort and acceleration interfaces if external torque monitoring is enabled
  if (params_.check_external_torque) {
    if (!controller_interface::get_ordered_interfaces(
            state_interfaces_, params_.joints, hardware_interface::HW_IF_EFFORT,
            joint_effort_state_interfaces_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not get ordered effort state interfaces");
      return controller_interface::CallbackReturn::FAILURE;
    }

    if (!controller_interface::get_ordered_interfaces(
            state_interfaces_, params_.joints, "acceleration_commanded",
            joint_acceleration_state_interfaces_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not get ordered acceleration state interfaces");
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  // Reset state
  freeze_triggered_ = false;
  service_call_in_progress_ = false;
  last_violation_reason_.clear();
  last_violation_time_ = get_node()->now();
  last_status_publish_time_ = get_node()->now();

  RCLCPP_INFO(get_node()->get_logger(), "Safety Monitor activated and monitoring");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyMonitorController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Safety Monitor deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SafetyMonitorController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& /*period*/)
{
  // Check safety conditions
  bool position_violation = check_position_limits();
  bool velocity_violation = params_.check_velocity ? check_velocity_limits() : false;
  bool workspace_violation = params_.check_workspace ? check_workspace_limits() : false;
  bool external_torque_violation = params_.check_external_torque ? check_external_torque() : false;

  // If we have any violation, trigger freeze
  // We check freeze_triggered_ and service_call_in_progress_ to avoid spamming service calls
  if ((position_violation || velocity_violation || workspace_violation || external_torque_violation) &&
      !freeze_triggered_ && !service_call_in_progress_) {
    trigger_freeze(last_violation_reason_);
  }

  // If no violation and freeze was triggered by us, allow retriggering
  // This allows the safety monitor to retrigger after user manually unfreezes
  if (!position_violation && !velocity_violation && !workspace_violation && !external_torque_violation && freeze_triggered_) {
    freeze_triggered_ = false;
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                        "Safety conditions now satisfied - ready to monitor again");
  }

  // Publish status periodically
  publish_status(time);

  return controller_interface::return_type::OK;
}

bool SafetyMonitorController::check_position_limits()
{
  if (!params_.check_joint_limits || joint_lower_limits_.empty()) {
    return false;
  }

  const size_t num_joints = joint_position_state_interfaces_.size();

  for (size_t i = 0; i < num_joints; ++i) {
    double position;
    try {
      position = duatic_dynaarm_controllers::compat::require_value(joint_position_state_interfaces_[i].get());
    } catch (const duatic_dynaarm_controllers::exceptions::MissingInterfaceValue& e) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Failed to read position for joint '%s': %s",
                           params_.joints[i].c_str(), e.what());
      return false;
    }

    // Check upper limit (from URDF with tolerance)
    if (position > joint_upper_limits_[i]) {
      last_violation_reason_ = "Joint " + std::to_string(i) + " (" + params_.joints[i] +
                              ") position " + std::to_string(position) +
                              " rad exceeds max limit " + std::to_string(joint_upper_limits_[i]) + " rad";
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "SAFETY VIOLATION: %s", last_violation_reason_.c_str());
      return true;
    }

    // Check lower limit (from URDF with tolerance)
    if (position < joint_lower_limits_[i]) {
      last_violation_reason_ = "Joint " + std::to_string(i) + " (" + params_.joints[i] +
                              ") position " + std::to_string(position) +
                              " rad below min limit " + std::to_string(joint_lower_limits_[i]) + " rad";
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "SAFETY VIOLATION: %s", last_violation_reason_.c_str());
      return true;
    }
  }

  return false;
}

bool SafetyMonitorController::check_velocity_limits()
{
  if (!params_.check_velocity || joint_velocity_state_interfaces_.empty()) {
    return false;
  }

  const size_t num_joints = joint_velocity_state_interfaces_.size();

  for (size_t i = 0; i < num_joints; ++i) {
    double velocity;
    try {
      velocity = duatic_dynaarm_controllers::compat::require_value(joint_velocity_state_interfaces_[i].get());
    } catch (const duatic_dynaarm_controllers::exceptions::MissingInterfaceValue& e) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Failed to read velocity for joint '%s': %s",
                           params_.joints[i].c_str(), e.what());
      return false;
    }

    // Check velocity magnitude
    if (std::abs(velocity) > params_.max_joint_velocities[i]) {
      last_violation_reason_ = "Joint " + std::to_string(i) + " (" + params_.joints[i] +
                              ") velocity " + std::to_string(velocity) +
                              " rad/s exceeds max limit " + std::to_string(params_.max_joint_velocities[i]) + " rad/s";
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "SAFETY VIOLATION: %s", last_violation_reason_.c_str());
      return true;
    }
  }

  return false;
}

bool SafetyMonitorController::check_workspace_limits()
{
  if (!params_.check_workspace) {
    return false;
  }

  const size_t num_joints = joint_position_state_interfaces_.size();

  // Build joint position vector for FK
  Eigen::VectorXd q = Eigen::VectorXd::Zero(pinocchio_model_.nq);

  for (size_t i = 0; i < num_joints; ++i) {
    double position;
    try {
      position = duatic_dynaarm_controllers::compat::require_value(joint_position_state_interfaces_[i].get());
    } catch (const duatic_dynaarm_controllers::exceptions::MissingInterfaceValue& e) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Failed to read position for joint '%s': %s",
                           params_.joints[i].c_str(), e.what());
      return false;
    }

    // Map joint to pinocchio model index
    const std::string& joint_name = params_.joints[i];
    auto idx = pinocchio_model_.getJointId(joint_name);
    if (idx == 0) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
      return false;
    }
    q[pinocchio_model_.joints[idx].idx_q()] = position;
  }

  // Compute forward kinematics
  pinocchio::forwardKinematics(pinocchio_model_, pinocchio_data_, q);
  pinocchio::updateFramePlacements(pinocchio_model_, pinocchio_data_);

  // Get end-effector position
  const auto& ee_transform = pinocchio_data_.oMf[ee_frame_id_];
  const Eigen::Vector3d ee_position = ee_transform.translation();

  // Check workspace limits
  if (ee_position.x() < params_.workspace_x_min) {
    last_violation_reason_ = "End-effector X position " + std::to_string(ee_position.x()) +
                            " m below min limit " + std::to_string(params_.workspace_x_min) + " m";
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "WORKSPACE VIOLATION: %s", last_violation_reason_.c_str());
    return true;
  }

  if (ee_position.x() > params_.workspace_x_max) {
    last_violation_reason_ = "End-effector X position " + std::to_string(ee_position.x()) +
                            " m exceeds max limit " + std::to_string(params_.workspace_x_max) + " m";
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "WORKSPACE VIOLATION: %s", last_violation_reason_.c_str());
    return true;
  }

  if (ee_position.y() < params_.workspace_y_min) {
    last_violation_reason_ = "End-effector Y position " + std::to_string(ee_position.y()) +
                            " m below min limit " + std::to_string(params_.workspace_y_min) + " m";
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "WORKSPACE VIOLATION: %s", last_violation_reason_.c_str());
    return true;
  }

  if (ee_position.y() > params_.workspace_y_max) {
    last_violation_reason_ = "End-effector Y position " + std::to_string(ee_position.y()) +
                            " m exceeds max limit " + std::to_string(params_.workspace_y_max) + " m";
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "WORKSPACE VIOLATION: %s", last_violation_reason_.c_str());
    return true;
  }

  if (ee_position.z() < params_.workspace_z_min) {
    last_violation_reason_ = "End-effector Z position " + std::to_string(ee_position.z()) +
                            " m below min limit " + std::to_string(params_.workspace_z_min) + " m (table height)";
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "WORKSPACE VIOLATION: %s", last_violation_reason_.c_str());
    return true;
  }

  if (ee_position.z() > params_.workspace_z_max) {
    last_violation_reason_ = "End-effector Z position " + std::to_string(ee_position.z()) +
                            " m exceeds max limit " + std::to_string(params_.workspace_z_max) + " m";
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "WORKSPACE VIOLATION: %s", last_violation_reason_.c_str());
    return true;
  }

  return false;
}

bool SafetyMonitorController::check_external_torque()
{
  if (!params_.check_external_torque) {
    return false;
  }

  const size_t num_joints = joint_position_state_interfaces_.size();

  // Build q, v, a vectors for Pinocchio RNEA
  Eigen::VectorXd q = Eigen::VectorXd::Zero(pinocchio_model_.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(pinocchio_model_.nv);

  for (size_t i = 0; i < num_joints; ++i) {
    double position, velocity, acceleration;
    try {
      position = duatic_dynaarm_controllers::compat::require_value(joint_position_state_interfaces_[i].get());
      velocity = duatic_dynaarm_controllers::compat::require_value(joint_velocity_state_interfaces_[i].get());
      acceleration = duatic_dynaarm_controllers::compat::require_value(joint_acceleration_state_interfaces_[i].get());
    } catch (const duatic_dynaarm_controllers::exceptions::MissingInterfaceValue& e) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Failed to read state for joint '%s': %s",
                           params_.joints[i].c_str(), e.what());
      return false;
    }

    // Map joint to pinocchio model index
    const std::string& joint_name = params_.joints[i];
    auto joint_id = pinocchio_model_.getJointId(joint_name);
    if (joint_id == 0) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
      return false;
    }

    const int idx_q = pinocchio_model_.joints[joint_id].idx_q();
    const int idx_v = pinocchio_model_.joints[joint_id].idx_v();

    q[idx_q] = position;
    v[idx_v] = velocity;
    a[idx_v] = acceleration;
  }

  // Compute expected torque using RNEA: tau = M(q)a + C(q,v)v + g(q)
  pinocchio::forwardKinematics(pinocchio_model_, pinocchio_data_, q, v, a);
  const Eigen::VectorXd tau_expected = pinocchio::rnea(pinocchio_model_, pinocchio_data_, q, v, a);

  // Check residuals for each joint
  for (size_t i = 0; i < num_joints; ++i) {
    double tau_measured;
    try {
      tau_measured = duatic_dynaarm_controllers::compat::require_value(joint_effort_state_interfaces_[i].get());
    } catch (const duatic_dynaarm_controllers::exceptions::MissingInterfaceValue& e) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Failed to read effort for joint '%s': %s",
                           params_.joints[i].c_str(), e.what());
      return false;
    }

    // Get expected torque for this joint
    const std::string& joint_name = params_.joints[i];
    auto joint_id = pinocchio_model_.getJointId(joint_name);
    const int idx_v = pinocchio_model_.joints[joint_id].idx_v();
    double tau_exp = tau_expected[idx_v];

    // Compute residual
    double residual = std::abs(tau_measured - tau_exp);

    // Apply moving average filter
    torque_residual_history_[i].push_back(residual);
    if (static_cast<int>(torque_residual_history_[i].size()) > params_.torque_residual_filter_window) {
      torque_residual_history_[i].pop_front();
    }

    // Compute filtered residual (average)
    double sum = 0.0;
    for (const auto& val : torque_residual_history_[i]) {
      sum += val;
    }
    double filtered_residual = torque_residual_history_[i].empty() ? 0.0 : sum / torque_residual_history_[i].size();
    filtered_torque_residuals_[i] = filtered_residual;

    // Get threshold for this joint
    // Use single threshold if array has <= 1 elements (including default [0.0])
    double threshold = (params_.external_torque_thresholds.size() > 1)
                        ? params_.external_torque_thresholds[i]
                        : params_.external_torque_threshold_single;

    // Debug logging: print torque values periodically
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                         "Joint %zu (%s): τ_measured=%.3f Nm, τ_expected=%.3f Nm, residual=%.3f Nm (filtered=%.3f Nm), threshold=%.3f Nm",
                         i, params_.joints[i].c_str(), tau_measured, tau_exp, residual, filtered_residual, threshold);

    // Check if collision detected
    if (filtered_residual > threshold) {
      last_violation_reason_ = "External torque detected on joint " + std::to_string(i) +
                              " (" + params_.joints[i] + "): residual " +
                              std::to_string(filtered_residual) + " Nm exceeds threshold " +
                              std::to_string(threshold) + " Nm (measured: " +
                              std::to_string(tau_measured) + " Nm, expected: " +
                              std::to_string(tau_exp) + " Nm)";
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "COLLISION DETECTED: %s", last_violation_reason_.c_str());
      return true;
    }
  }

  return false;
}

void SafetyMonitorController::trigger_freeze(const std::string& reason)
{
  // Prevent multiple simultaneous service calls
  if (service_call_in_progress_) {
    return;
  }

  RCLCPP_ERROR(get_node()->get_logger(),
               "🚨 TRIGGERING FREEZE CONTROLLER 🚨");
  RCLCPP_ERROR(get_node()->get_logger(), "Reason: %s", reason.c_str());

  // Create service request
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers.push_back(params_.freeze_controller_name);
  request->deactivate_controllers.clear();
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  request->timeout = rclcpp::Duration(0, 0);  // No timeout

  // Send async request
  service_call_in_progress_ = true;

  auto response_callback = [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
    try {
      auto response = future.get();
      if (response->ok) {
        RCLCPP_INFO(get_node()->get_logger(), "Freeze controller activated successfully");
        freeze_triggered_ = true;
      } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate freeze controller!");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Service call exception: %s", e.what());
    }
    service_call_in_progress_ = false;
  };

  switch_controller_client_->async_send_request(request, response_callback);
}

void SafetyMonitorController::publish_status(const rclcpp::Time& time)
{
  // Publish status at configured rate
  const double dt = (time - last_status_publish_time_).seconds();
  const double publish_period = 1.0 / params_.status_publish_rate;

  if (dt < publish_period) {
    return;
  }

  std_msgs::msg::String status_msg;

  if (freeze_triggered_) {
    status_msg.data = "FROZEN - Safety violation: " + last_violation_reason_;
  } else {
    status_msg.data = "OK - Monitoring active";
  }

  if (status_pub_rt_->trylock()) {
    status_pub_rt_->msg_ = status_msg;
    status_pub_rt_->unlockAndPublish();
  }

  last_status_publish_time_ = time;
}

}  // namespace duatic_dynaarm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(duatic_dynaarm_controllers::SafetyMonitorController,
                       controller_interface::ControllerInterface)
