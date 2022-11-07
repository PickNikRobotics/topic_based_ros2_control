// Copyright 2022 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Jafar Abdi */
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include <issac_ros2_control/issac_system.hpp>
#include <rclcpp/executors.hpp>

namespace issac_ros2_control
{

CallbackReturn IssacSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize storage for all joints' standard interfaces, regardless of their existence and set all values without
  // initial values to 0
  joint_commands_.resize(standard_interfaces_.size());
  joint_states_.resize(standard_interfaces_.size());
  for (auto i = 0u; i < standard_interfaces_.size(); i++)
  {
    joint_commands_[i].resize(info_.joints.size(), 0.0);
    joint_states_[i].resize(info_.joints.size(), 0.0);
  }

  const auto get_hardware_parameter = [this](const std::string& parameter_name, const std::string& default_value) {
    if (auto it = info_.hardware_parameters.find(parameter_name); it != info_.hardware_parameters.end())
    {
      return it->second;
    }
    return default_value;
  };

  node_ = rclcpp::Node::make_shared("issac_ros2_control");
  issac_joint_commands_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_commands_topic", "/joint_command"), rclcpp::QoS(1));
  issac_joint_states_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_states_topic", "/issac_joint_states"), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr joint_state) { latest_joint_state_ = *joint_state; });

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> IssacSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joints' state interfaces
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const auto& joint = info_.joints[i];
    for (const auto& interface : joint.state_interfaces)
    {
      // Add interface: if not in the standard list then use "other" interface list
      if (!getInterface(joint.name, interface.name, i, joint_states_, state_interfaces))
      {
        throw std::runtime_error("Interface is not found in the standard list.");
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> IssacSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Joints' state interfaces
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const auto& joint = info_.joints[i];
    for (const auto& interface : joint.command_interfaces)
    {
      if (!getInterface(joint.name, interface.name, i, joint_commands_, command_interfaces))
      {
        throw std::runtime_error("Interface is not found in the standard list.");
      }
    }
  }

  return command_interfaces;
}

hardware_interface::return_type IssacSystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  rclcpp::spin_some(node_);
  for (std::size_t i = 0; i < latest_joint_state_.name.size(); ++i)
  {
    const auto& joints = info_.joints;
    auto it = std::find_if(joints.begin(), joints.end(),
                           [&joint_name = std::as_const(latest_joint_state_.name[i])](
                               const hardware_interface::ComponentInfo& info) { return joint_name == info.name; });
    if (it != joints.end())
    {
      auto j = static_cast<std::size_t>(std::distance(joints.begin(), it));
      joint_states_[0][j] = latest_joint_state_.position[i];
      joint_states_[1][j] = latest_joint_state_.velocity[i];
      joint_states_[3][j] = latest_joint_state_.effort[i];
    }
  }

  return hardware_interface::return_type::OK;
}

template <typename HandleType>
bool IssacSystem::getInterface(const std::string& name, const std::string& interface_name, const size_t vector_index,
                               std::vector<std::vector<double>>& values, std::vector<HandleType>& interfaces)
{
  auto it = std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface_name);
  if (it != standard_interfaces_.end())
  {
    auto j = static_cast<std::size_t>(std::distance(standard_interfaces_.begin(), it));
    interfaces.emplace_back(name, *it, &values[j][vector_index]);
    return true;
  }
  return false;
}

hardware_interface::return_type IssacSystem::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (std::all_of(joint_commands_[0].cbegin(), joint_commands_[0].cend(),
                  [](const double joint_command) { return std::isnan(joint_command); }))
  {
    return hardware_interface::return_type::OK;
  }

  sensor_msgs::msg::JointState joint_state;
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    joint_state.name.push_back(info_.joints[i].name);
    joint_state.header.stamp = node_->now();
    joint_state.position.push_back(joint_commands_[0][i]);
    joint_state.velocity.push_back(joint_commands_[1][i]);
    joint_state.effort.push_back(joint_commands_[3][i]);
  }
  issac_joint_commands_publisher_->publish(joint_state);

  return hardware_interface::return_type::OK;
}
}  // end namespace issac_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(issac_ros2_control::IssacSystem, hardware_interface::SystemInterface)
