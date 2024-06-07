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

#include <angles/angles.h>
#include <rclcpp/executors.hpp>
#include <topic_based_ros2_control/topic_based_system.hpp>

namespace
{
/** @brief Sums the total rotation for joint states that wrap from 2*pi to -2*pi
when rotating in the positive direction */
void sumRotationFromMinus2PiTo2Pi(const double current_wrapped_rad, double& total_rotation)
{
  double delta = 0;
  angles::shortest_angular_distance_with_large_limits(total_rotation, current_wrapped_rad, 2 * M_PI, -2 * M_PI, delta);

  // Add the corrected delta to the total rotation
  total_rotation += delta;
}
}  // namespace

namespace topic_based_ros2_control
{

static constexpr std::size_t POSITION_INTERFACE_INDEX = 0;
static constexpr std::size_t VELOCITY_INTERFACE_INDEX = 1;
// JointState doesn't contain an acceleration field, so right now it's not used
static constexpr std::size_t EFFORT_INTERFACE_INDEX = 3;

CallbackReturn TopicBasedSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize storage for all joints' standard interfaces, regardless of their existence and set all values to nan
  joint_commands_.resize(standard_interfaces_.size());
  joint_states_.resize(standard_interfaces_.size());
  for (auto i = 0u; i < standard_interfaces_.size(); i++)
  {
    joint_commands_[i].resize(info_.joints.size(), 0.0);
    joint_states_[i].resize(info_.joints.size(), 0.0);
  }

  // Initial command values
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const auto& component = info_.joints[i];
    for (const auto& interface : component.state_interfaces)
    {
      auto it = std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name);
      // If interface name is found in the interfaces list
      if (it != standard_interfaces_.end())
      {
        auto index = static_cast<std::size_t>(std::distance(standard_interfaces_.begin(), it));
        // Check the initial_value param is used
        if (!interface.initial_value.empty())
        {
          joint_commands_[index][i] = std::stod(interface.initial_value);
        }
      }
    }
  }
  ready_to_send_cmds_ = true;

  // Search for mimic joints
  for (auto i = 0u; i < info_.joints.size(); ++i)
  {
    const auto& joint = info_.joints.at(i);
    if (joint.parameters.find("mimic") != joint.parameters.cend())
    {
      const auto mimicked_joint_it = std::find_if(
          info_.joints.begin(), info_.joints.end(),
          [&mimicked_joint = joint.parameters.at("mimic")](const hardware_interface::ComponentInfo& joint_info) {
            return joint_info.name == mimicked_joint;
          });
      if (mimicked_joint_it == info_.joints.cend())
      {
        throw std::runtime_error(std::string("Mimicked joint '") + joint.parameters.at("mimic") + "' not found");
      }
      MimicJoint mimic_joint;
      mimic_joint.joint_index = i;
      mimic_joint.mimicked_joint_index =
          static_cast<std::size_t>(std::distance(info_.joints.begin(), mimicked_joint_it));
      auto param_it = joint.parameters.find("multiplier");
      if (param_it != joint.parameters.end())
      {
        mimic_joint.multiplier = std::stod(joint.parameters.at("multiplier"));
      }
      mimic_joints_.push_back(mimic_joint);
    }
  }

  const auto get_hardware_parameter = [this](const std::string& parameter_name, const std::string& default_value) {
    if (auto it = info_.hardware_parameters.find(parameter_name); it != info_.hardware_parameters.end())
    {
      return it->second;
    }
    return default_value;
  };

  // Add random ID to prevent warnings about multiple publishers within the same node
  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=topic_based_ros2_control_" + info_.name });

  node_ = rclcpp::Node::make_shared("_", options);

  if (auto it = info_.hardware_parameters.find("trigger_joint_command_threshold"); it != info_.hardware_parameters.end())
  {
    trigger_joint_command_threshold_ = std::stod(it->second);
  }

  topic_based_joint_commands_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_commands_topic", "/robot_joint_commands"), rclcpp::QoS(1));
  topic_based_joint_states_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_states_topic", "/robot_joint_states"), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr joint_state) { latest_joint_state_ = *joint_state; });

  // if the values on the `joint_states_topic` are wrapped between -2*pi and 2*pi (like they are in Isaac Sim)
  // sum the total joint rotation returned on the `joint_states_` interface
  if (get_hardware_parameter("sum_wrapped_joint_states", "false") == "true")
  {
    sum_wrapped_joint_states_ = true;
  }
  if (get_hardware_parameter("use_initial_states_as_initial_commands", "false") == "true")
  {
    initial_states_as_initial_cmd_ = true;
    ready_to_send_cmds_ = false;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TopicBasedSystem::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> TopicBasedSystem::export_command_interfaces()
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

hardware_interface::return_type TopicBasedSystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }

  for (std::size_t i = 0; i < latest_joint_state_.name.size(); ++i)
  {
    const auto& joints = info_.joints;
    auto it = std::find_if(joints.begin(), joints.end(),
                           [&joint_name = std::as_const(latest_joint_state_.name[i])](
                               const hardware_interface::ComponentInfo& info) { return joint_name == info.name; });
    if (it != joints.end())
    {
      auto j = static_cast<std::size_t>(std::distance(joints.begin(), it));
      if (sum_wrapped_joint_states_)
      {
        sumRotationFromMinus2PiTo2Pi(latest_joint_state_.position[i], joint_states_[POSITION_INTERFACE_INDEX][j]);
      }
      else
      {
        joint_states_[POSITION_INTERFACE_INDEX][j] = latest_joint_state_.position[i];
      }
      if (!latest_joint_state_.velocity.empty())
      {
        joint_states_[VELOCITY_INTERFACE_INDEX][j] = latest_joint_state_.velocity[i];
      }
      if (!latest_joint_state_.effort.empty())
      {
        joint_states_[EFFORT_INTERFACE_INDEX][j] = latest_joint_state_.effort[i];
      }
    }
  }

  for (const auto& mimic_joint : mimic_joints_)
  {
    for (auto& joint_state : joint_states_)
    {
      joint_state[mimic_joint.joint_index] = mimic_joint.multiplier * joint_state[mimic_joint.mimicked_joint_index];
    }
  }

  if (!ready_to_send_cmds_ && initial_states_as_initial_cmd_)
  {
    for (std::size_t i = 0; i < joint_states_.size(); ++i)
    {
      for (std::size_t j = 0; j < joint_states_[i].size(); ++j)
      {
        joint_commands_[i][j] = joint_states_[i][j];
      }
    }
    ready_to_send_cmds_ = true;
  }

  return hardware_interface::return_type::OK;
}

template <typename HandleType>
bool TopicBasedSystem::getInterface(const std::string& name, const std::string& interface_name,
                                    const size_t vector_index, std::vector<std::vector<double>>& values,
                                    std::vector<HandleType>& interfaces)
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

hardware_interface::return_type TopicBasedSystem::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (!ready_to_send_cmds_)
  {
    return hardware_interface::return_type::ERROR;
  }
  // To avoid spamming TopicBased's joint command topic we check the difference between the joint states and
  // the current joint commands, if it's smaller than a threshold we don't publish it.
  const auto diff = std::transform_reduce(
      joint_states_[POSITION_INTERFACE_INDEX].cbegin(), joint_states_[POSITION_INTERFACE_INDEX].cend(),
      joint_commands_[POSITION_INTERFACE_INDEX].cbegin(), 0.0,
      [](const auto d1, const auto d2) { return std::abs(d1) + std::abs(d2); }, std::minus<double>{});
  if (diff <= trigger_joint_command_threshold_)
  {
    return hardware_interface::return_type::OK;
  }

  sensor_msgs::msg::JointState joint_state;
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    joint_state.name.push_back(info_.joints[i].name);
    joint_state.header.stamp = node_->now();
    // only send commands to the interfaces that are defined for this joint
    for (const auto& interface : info_.joints[i].command_interfaces)
    {
      if (interface.name == hardware_interface::HW_IF_POSITION)
      {
        joint_state.position.push_back(joint_commands_[POSITION_INTERFACE_INDEX][i]);
      }
      else if (interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        joint_state.velocity.push_back(joint_commands_[VELOCITY_INTERFACE_INDEX][i]);
      }
      else if (interface.name == hardware_interface::HW_IF_EFFORT)
      {
        joint_state.effort.push_back(joint_commands_[EFFORT_INTERFACE_INDEX][i]);
      }
      else
      {
        RCLCPP_WARN_ONCE(node_->get_logger(), "Joint '%s' has unsupported command interfaces found: %s.",
                         info_.joints[i].name.c_str(), interface.name.c_str());
      }
    }
  }

  for (const auto& mimic_joint : mimic_joints_)
  {
    for (const auto& interface : info_.joints[mimic_joint.mimicked_joint_index].command_interfaces)
    {
      if (interface.name == hardware_interface::HW_IF_POSITION)
      {
        joint_state.position[mimic_joint.joint_index] =
            mimic_joint.multiplier * joint_state.position[mimic_joint.mimicked_joint_index];
      }
      else if (interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        joint_state.velocity[mimic_joint.joint_index] =
            mimic_joint.multiplier * joint_state.velocity[mimic_joint.mimicked_joint_index];
      }
      else if (interface.name == hardware_interface::HW_IF_EFFORT)
      {
        joint_state.effort[mimic_joint.joint_index] =
            mimic_joint.multiplier * joint_state.effort[mimic_joint.mimicked_joint_index];
      }
    }
  }

  if (rclcpp::ok())
  {
    topic_based_joint_commands_publisher_->publish(joint_state);
  }

  return hardware_interface::return_type::OK;
}
}  // end namespace topic_based_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(topic_based_ros2_control::TopicBasedSystem, hardware_interface::SystemInterface)
