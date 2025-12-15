// Copyright ...
// (保持原有版权说明)

#include <limits>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <sstream>
#include <cctype>
#include <algorithm>
#include <bitset>



#include "gripper_hardware_interface/gripper_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "elfin_robot_msgs/srv/gripper_read_data.hpp"
#include "elfin_robot_msgs/srv/gripper_write_data.hpp"


namespace gripper_hardware_interface
{

hardware_interface::CallbackReturn GripperHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // 创建独立节点
  node_ = std::make_shared<rclcpp::Node>("gripper_hw_interface_node");

//  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
//  executor_->add_node(node_);
//  spinner_thread_ = std::thread([this]() {
//    executor_->spin();
//  });

  // 创建客户端
  read_client_  = node_->create_client<elfin_robot_msgs::srv::GripperReadData>("/read_claw");
  write_client_ = node_->create_client<elfin_robot_msgs::srv::GripperWriteData>("/write_claw");

  // 等待 service available，避免刚启动时超时
  if (!read_client_->wait_for_service(std::chrono::seconds(20))) {
    RCLCPP_ERROR(rclcpp::get_logger("GripperHardwareInterface"), "Service /read_claw not available in on_init");
    return CallbackReturn::ERROR;
  }
  if (!write_client_->wait_for_service(std::chrono::seconds(20))) {
    RCLCPP_ERROR(rclcpp::get_logger("GripperHardwareInterface"), "Service /write_claw not available in on_init");
    return CallbackReturn::ERROR;
  }

  // 初始化数据
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  if (info_.joints.size() != 1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("GripperHardwareInterface"),
                "Expected 1 joint, but got %zu joints", info_.joints.size());
    return CallbackReturn::ERROR;
  }

  if (info_.joints[0].name != "Left_1_Joint")
  {
    RCLCPP_ERROR(rclcpp::get_logger("GripperHardwareInterface"),
                "Expected joint name 'Left_1_Joint', but got '%s'", info_.joints[0].name.c_str());
    return CallbackReturn::ERROR;
  }

  // 初始化状态
  current_position_ = 0.0;
  target_position_ = -1.0;
  position_reached_ = true;

  // 读取参数
  for (const auto& param : info_.hardware_parameters)
  {
    if (param.first == "gripper_slave_id") gripper_slave_id_ = std::stoi(param.second);
    else if (param.first == "gripper_function_code") gripper_function_code_ = std::stoi(param.second);
    else if (param.first == "gripper_speed") gripper_speed_ = std::stod(param.second);
    else if (param.first == "gripper_force") gripper_force_ = std::stod(param.second);
    else if (param.first == "ethernet_interface") ethernet_interface_ = param.second;
    else if (param.first == "position_tolerance") position_tolerance_ = std::stod(param.second);
    else if (param.first == "max_position") max_position_ = std::stod(param.second);
    else if (param.first == "min_position") min_position_ = std::stod(param.second);
  }

  // 设置默认值
  if (gripper_slave_id_ == 0) gripper_slave_id_ = 1;
  if (gripper_function_code_ == 0) gripper_function_code_ = 6;
  if (gripper_speed_ == 0.0) gripper_speed_ = 100.0;
  if (gripper_force_ == 0.0) gripper_force_ = 50.0;
  if (ethernet_interface_.empty()) ethernet_interface_ = "enp3s0";
  if (position_tolerance_ == 0.000) position_tolerance_ = 0.01;
  if (max_position_ == 0.0) max_position_ = 0.0;
  if (min_position_ == -1.0) min_position_ = -1.0;

  RCLCPP_INFO(rclcpp::get_logger("GripperHardwareInterface"), "Gripper hardware interface initialized");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("GripperHardwareInterface"), "Configuring...");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GripperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GripperHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("GripperHardwareInterface"), "Activating...");

  if (!readGripperPosition()) {
    RCLCPP_ERROR(rclcpp::get_logger("GripperHardwareInterface"), "Failed to read initial position");
    current_position_ = 0.0;
  }
  usleep(100000);
  for (size_t i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = current_position_;
    hw_commands_[i] = current_position_;
  }

  RCLCPP_INFO(rclcpp::get_logger("GripperHardwareInterface"), "Activated successfully!");
  return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn GripperHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("GripperHardwareInterface"), "Deactivating...");
//  if (executor_) {
//    executor_->cancel();
//  }
//  if (spinner_thread_.joinable()) {
//    spinner_thread_.join();
//  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type GripperHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  rclcpp::spin_some(node_);
  if (readGripperPosition())
  {
    for (size_t i = 0; i < hw_states_.size(); ++i) {
      hw_states_[i] = current_position_;
    }
  }
//  for (size_t i = 0; i < hw_states_.size(); ++i) {
//        hw_states_[i] = current_position_;
//        hw_commands_[i] = current_position_;
//  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GripperHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &) {
    rclcpp::spin_some(node_);

    for (size_t i = 0; i < hw_commands_.size(); ++i) {
        double target = hw_commands_[i];
        if (target == target_position_) {
            continue;
        }
        if (std::isfinite(target) &&
            std::abs(target - current_position_) > position_tolerance_)
        {
//            readGripperPosition(); // 更新 current_position_
            RCLCPP_INFO(node_->get_logger(),
            "目标位置=%f → 当前位置=%f", target, current_position_);
            //target_position_ = std::clamp(target, min_position_, max_position_);
//            RCLCPP_INFO(node_->get_logger(), "target_position_: %f", target_position_);
            if (!writeGripperPosition(target)) {
//                RCLCPP_INFO(node_->get_logger(),
//                "没有达到目标位置");
                position_reached_ = false; // 标记未到达
            } else {
//                readGripperPosition(); // 更新 current_position_
                target_position_ = target;
            }
            break;
        }
    }

    return hardware_interface::return_type::OK;
}


bool GripperHardwareInterface::readGripperPosition() {
    auto request = std::make_shared<elfin_robot_msgs::srv::GripperReadData::Request>();
    request->slave_id = gripper_slave_id_;
    request->function_code = 3;

    read_client_->async_send_request(
        request,
        [this](rclcpp::Client<elfin_robot_msgs::srv::GripperReadData>::SharedFuture future) {
            auto response = future.get();
            //RCLCPP_INFO(node_->get_logger(), "原始位置值: %ld", static_cast<long>(response->position));

            // 关键修改：0~1033 → -1~0 线性映射
            const double max_position = 1106.0; // 完全闭合时的原始值
            current_position_ = static_cast<double>(response->position) / max_position - 1.0;

            // 确保结果在[-1,0]范围内
            current_position_ = std::clamp(current_position_, -1.0, 0.0);
//            RCLCPP_INFO(node_->get_logger(), "转换后位置: %f", current_position_);
        }
    );
    return true;
}


bool GripperHardwareInterface::writeGripperPosition(double position) {
    // 输入范围保护：确保position在[-1,0]内
    position = std::clamp(position, -1.0, 0.0);

    // 核心修改：逆向映射 -1~0 → 0~9000
    const double max_position = 9000.0;
    int32_t steps = static_cast<int32_t>((position + 1.0) * max_position);

    // 日志调试：输出映射前后的值
    RCLCPP_INFO(node_->get_logger(),
        "写入映射: position=%f → steps=%d", position, steps);

    auto request = std::make_shared<elfin_robot_msgs::srv::GripperWriteData::Request>();
    request->slave_id = gripper_slave_id_;
    request->function_code = 6;
    request->addrss = 0;
    request->value = steps;          // 使用映射后的值
    request->speed = static_cast<int>(gripper_speed_);
    request->force = static_cast<int>(gripper_force_);

    write_client_->async_send_request(request,[this](rclcpp::Client<elfin_robot_msgs::srv::GripperWriteData>::SharedFuture future){
        auto response = future.get();
        if (!response->success) {
            RCLCPP_ERROR(node_->get_logger(), "写入无效响应");
            return false;
        }
        return true;
    }
    );
    return true;
}

bool GripperHardwareInterface::isPositionReached()
{
  if (std::abs(current_position_ - target_position_) <= position_tolerance_) {
    if (!position_reached_) {
      RCLCPP_INFO(rclcpp::get_logger("GripperHardwareInterface"),
                 "Reached pos: %.3f", current_position_);
      position_reached_ = true;
    }
    return true;
  }
  return false;
}

}  // namespace gripper_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  gripper_hardware_interface::GripperHardwareInterface, hardware_interface::SystemInterface)

