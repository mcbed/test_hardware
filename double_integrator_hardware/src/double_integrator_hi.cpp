// Copyright 2022, ICube Laboratory, University of Strasbourg
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

#include "double_integrator_hardware/double_integrator_hi.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <iterator>
#include <iostream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace double_integrator_hardware
{
  // ------------------------------------------------------------------------------------------
CallbackReturn DoubleIntegratorHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_states_position_ = std::numeric_limits<double>::quiet_NaN();
  hw_states_velocity_ = std::numeric_limits<double>::quiet_NaN();
  hw_states_acceleration_ = std::numeric_limits<double>::quiet_NaN();
  hw_commands_acceleration_ = std::numeric_limits<double>::quiet_NaN();
  hw_states_ext_effort_ = std::numeric_limits<double>::quiet_NaN();

  // double integrator has currently exactly 3 states and 1 command interface on each joint
  if (info_.joints[0].command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DoubleIntegratorHardwareInterface"),
      "Joint '%s' has %ld command interfaces found. 1 expected.", info_.joints[0].name.c_str(),
      info_.joints[0].command_interfaces.size());
    return CallbackReturn::ERROR;
  }

  if (info_.joints[0].command_interfaces[0].name != hardware_interface::HW_IF_ACCELERATION)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DoubleIntegratorHardwareInterface"),
      "Joint '%s' have %s command interfaces found. '%s' expected.", info_.joints[0].name.c_str(),
      info_.joints[0].command_interfaces[0].name.c_str(), hardware_interface::HW_IF_ACCELERATION);
    return CallbackReturn::ERROR;
  }

  if (info_.joints[0].state_interfaces.size() != 4)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DoubleIntegratorHardwareInterface"),
      "Joint '%s' has %ld state interface. 4 expected.", info_.joints[0].name.c_str(),
      info_.joints[0].state_interfaces.size());
    return CallbackReturn::ERROR;
  }

  if (info_.joints[0].state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DoubleIntegratorHardwareInterface"),
      "Joint '%s' have %s state interface. '%s' expected.", info_.joints[0].name.c_str(),
      info_.joints[0].state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    return CallbackReturn::ERROR;
  }
  if (info_.joints[0].state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DoubleIntegratorHardwareInterface"),
      "Joint '%s' have %s state interface. '%s' expected.", info_.joints[0].name.c_str(),
      info_.joints[0].state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
    return CallbackReturn::ERROR;
  }
  if (info_.joints[0].state_interfaces[2].name != hardware_interface::HW_IF_ACCELERATION)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DoubleIntegratorHardwareInterface"),
      "Joint '%s' have %s state interface. '%s' expected.", info_.joints[0].name.c_str(),
      info_.joints[0].state_interfaces[0].name.c_str(), hardware_interface::HW_IF_ACCELERATION);
    return CallbackReturn::ERROR;
  }
  if (info_.joints[0].state_interfaces[3].name != "external_effort")
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DoubleIntegratorHardwareInterface"),
      "Joint '%s' have %s state interface. '%s' expected.", info_.joints[0].name.c_str(),
      info_.joints[0].state_interfaces[0].name.c_str(), "external_effort");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}
  // ------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
DoubleIntegratorHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_states_position_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_ACCELERATION, &hw_states_acceleration_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, "external_effort", &hw_states_ext_effort_));

  return state_interfaces;
}
  // ------------------------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
DoubleIntegratorHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_ACCELERATION, &hw_commands_acceleration_));

  return command_interfaces;
}
  // ------------------------------------------------------------------------------------------
CallbackReturn DoubleIntegratorHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DoubleIntegratorHardwareInterface"), "Starting ...please wait...");

  hw_states_velocity_ = 0;
  hw_states_position_ = 0;
  hw_states_acceleration_ = 0;
  hw_states_ext_effort_ = 0;

  RCLCPP_INFO(
    rclcpp::get_logger("DoubleIntegratorHardwareInterface"), "System Successfully started!");

  return CallbackReturn::SUCCESS;
}
  // ------------------------------------------------------------------------------------------
CallbackReturn DoubleIntegratorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DoubleIntegratorHardwareInterface"), "Stopping ...please wait...");

  RCLCPP_INFO(
    rclcpp::get_logger("DoubleIntegratorHardwareInterface"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}
  // ------------------------------------------------------------------------------------------
hardware_interface::return_type DoubleIntegratorHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Process input data and populate hw_states_position_, hw_states_velocity_, hw_states_acceleration_

  hw_states_velocity_ = hw_states_velocity_ + hw_states_acceleration_*period.nanoseconds()*1e-9;
  hw_states_position_ = hw_states_position_ + hw_states_velocity_*period.nanoseconds()*1e-9;

  return hardware_interface::return_type::OK;
}
  // ------------------------------------------------------------------------------------------
hardware_interface::return_type DoubleIntegratorHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if(std::isnan(hw_commands_acceleration_))
    hw_states_acceleration_ = hw_commands_acceleration_;
  else
    hw_states_acceleration_ = 0;

  return hardware_interface::return_type::OK;
}

}  // namespace double_integrator_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  double_integrator_hardware::DoubleIntegratorHardwareInterface, hardware_interface::ActuatorInterface)
