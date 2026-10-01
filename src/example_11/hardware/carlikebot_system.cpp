// Copyright 2021 ros2_control Development Team
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

#include "ros2_control_demo_example_11/carlikebot_system.hpp"
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ros2_control_demo_example_11
{
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.CarlikeBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      get_logger(),
      "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.rear_wheel_joint = info_.hardware_parameters["rear_wheel_joint"];
  cfg_.steering_wheel_joint = info_.hardware_parameters["steering_wheel_joint"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]); 
  
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a steering joint.", joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a drive joint.", joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }
  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production
  // code
  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  hw_interfaces_["steering"] = Joint(cfg_.steering_wheel_joint);

  hw_interfaces_["traction"] = Joint(cfg_.rear_wheel_joint);

  // ROS2 node for Hardware Interface
  node_ = rclcpp::Node::make_shared("carlikebot_hw_node");
  executor_.add_node(node_);
  // Publisher topic for sending commands to the ESP
  comms_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("esp_commands", 10);
  // Subscriber node for receiving state updates from the ESP
  state_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/esp_state", 10, std::bind(&CarlikeBotSystemHardware::state_callback, this, std::placeholders::_1));

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces() //registers the state interfaces of the hardware component, which are then used by the controller to read the state of the hardware
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    const std::string & name = joint.second.joint_name;
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

    if (name == cfg_.rear_wheel_joint) //traction
    {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          name, hardware_interface::HW_IF_VELOCITY,
          &joint.second.state.velocity));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CarlikeBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    const std::string & name = joint.second.joint_name;
    if (name == cfg_.steering_wheel_joint) //steering
    {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.second.joint_name, hardware_interface::HW_IF_POSITION,
          &joint.second.command.position));
    }
    else if (name == cfg_.rear_wheel_joint) //traction
    {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
          &joint.second.command.velocity));
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarlikeBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  executor_.spin_some();

  for (auto & joint:hw_interfaces_){
    if (joint.second.joint_name == cfg_.rear_wheel_joint){
      joint.second.state.velocity = last_throttle_cmd_;
    }
    else if (joint.second.joint_name == cfg_.steering_wheel_joint){
      joint.second.state.position = last_steering_cmd_;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_11 ::CarlikeBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double throttle_cmd = 0.0;
  double steering_cmd = 0.0;

  for(auto & joint:hw_interfaces_){
    const std::string & name = joint.second.joint_name;
    if (name == cfg_.rear_wheel_joint){
      throttle_cmd = joint.second.command.velocity;
    }
    else if (name == cfg_.steering_wheel_joint){
      steering_cmd = joint.second.command.position;
    }
  }
  // Packing messages into a Float32MultiArray to send to the ESP
  std_msgs::msg::Float32MultiArray msg;

  float max_velocity = 0.5;          // tune this
  float max_steering_angle = 0.6;    // radians (~30°)

  float norm_throttle = throttle_cmd / max_velocity;
  float norm_steering = steering_cmd / max_steering_angle;

  norm_throttle = std::clamp(norm_throttle, -1.0f, 1.0f);
  norm_steering = std::clamp(norm_steering, -1.0f, 1.0f);

  msg.data.push_back(norm_throttle);
  msg.data.push_back(norm_steering);
  comms_pub_->publish(msg);

  RCLCPP_INFO_THROTTLE(
  node_->get_logger(), *clock_, 1000,
  "WRITE CALLED: throttle=%.2f steering=%.2f",
  throttle_cmd, steering_cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_11

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_11::CarlikeBotSystemHardware, hardware_interface::SystemInterface)