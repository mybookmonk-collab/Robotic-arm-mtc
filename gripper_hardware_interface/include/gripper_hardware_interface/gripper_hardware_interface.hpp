// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef GRIPPER_HARDWARE_INTERFACE__GRIPPER_HARDWARE_INTERFACE_HPP_
#define GRIPPER_HARDWARE_INTERFACE__GRIPPER_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <optional>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node_interfaces/node_parameters.hpp"
#include "gripper_hardware_interface/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/client.hpp"
#include "elfin_robot_msgs/srv/gripper_read_data.hpp"
#include "elfin_robot_msgs/srv/gripper_write_data.hpp"


namespace gripper_hardware_interface
{
class GripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware interface data
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // 节点
  std::shared_ptr<rclcpp::Node> node_;
  // Executor + 线程
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_thread_;

  // Service clients
  rclcpp::Client<elfin_robot_msgs::srv::GripperReadData>::SharedPtr read_client_;
  rclcpp::Client<elfin_robot_msgs::srv::GripperWriteData>::SharedPtr write_client_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  
  // Gripper control parameters
  int32_t gripper_slave_id_ = 0;
  int32_t gripper_function_code_ = 0;
  double gripper_speed_ = 0.0;
  double gripper_force_ = 0.0;
  
  // Gripper state
  double current_position_ = 0.0;
  double target_position_ = 0.0;
  bool position_reached_ = true;
  
  // Configuration parameters
  std::string ethernet_interface_;
  double position_tolerance_ = 0.0;
  double max_position_ = 0.0;
  double min_position_ = 0.0;
  
  // Gripper control methods
  bool initializeEtherCAT();
  bool readGripperPosition();
  bool writeGripperPosition(double position);
  bool isPositionReached();
  
  // Helper utilities for invoking CLI-based ROS 2 service calls
  std::optional<std::string> runCommandAndCaptureOutput(const std::string & command) const;
  static std::optional<int32_t> parsePositionFromServiceOutput(const std::string & output);
  static std::optional<bool> parseSuccessFromServiceOutput(const std::string & output);
};

}  // namespace gripper_hardware_interface

#endif  // GRIPPER_HARDWARE_INTERFACE__GRIPPER_HARDWARE_INTERFACE_HPP_
