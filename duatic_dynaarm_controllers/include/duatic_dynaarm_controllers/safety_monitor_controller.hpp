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

#pragma once

// std
#include <atomic>
#include <memory>
#include <string>
#include <vector>

// ROS2
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/logging.hpp>
#include <realtime_tools/realtime_publisher.hpp>

// Messages & Services
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_msgs/msg/string.hpp>

// Project
#include <duatic_dynaarm_controllers/safety_monitor_controller_parameters.hpp>
#include <duatic_dynaarm_controllers/interface_utils.hpp>

// Pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

// std
#include <deque>

namespace duatic_dynaarm_controllers
{

class SafetyMonitorController : public controller_interface::ControllerInterface
{
public:
  SafetyMonitorController();
  ~SafetyMonitorController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // Parameters
  std::unique_ptr<safety_monitor_controller::ParamListener> param_listener_;
  safety_monitor_controller::Params params_;

  // State interfaces
  StateInterfaceReferences joint_position_state_interfaces_;
  StateInterfaceReferences joint_velocity_state_interfaces_;
  StateInterfaceReferences joint_effort_state_interfaces_;        // Measured torque
  StateInterfaceReferences joint_acceleration_state_interfaces_;  // For RNEA

  // Service client for activating freeze controller
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;

  // State tracking
  std::atomic<bool> freeze_triggered_{false};
  std::atomic<bool> service_call_in_progress_{false};
  rclcpp::Time last_violation_time_;
  std::string last_violation_reason_;

  // Status publishing
  using StatusPublisher = realtime_tools::RealtimePublisher<std_msgs::msg::String>;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::unique_ptr<StatusPublisher> status_pub_rt_;
  rclcpp::Time last_status_publish_time_;

  // Pinocchio model for forward kinematics and joint limits
  pinocchio::Model pinocchio_model_;
  pinocchio::Data pinocchio_data_;
  pinocchio::FrameIndex ee_frame_id_;

  // Joint limits from URDF (with tolerance applied)
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;

  // External torque monitoring (collision detection)
  std::vector<std::deque<double>> torque_residual_history_;  // Moving average filter
  std::vector<double> filtered_torque_residuals_;

  // Helper methods
  bool check_position_limits();
  bool check_velocity_limits();
  bool check_workspace_limits();
  bool check_external_torque();
  void trigger_freeze(const std::string& reason);
  void publish_status(const rclcpp::Time& time);
};

}  // namespace duatic_dynaarm_controllers
