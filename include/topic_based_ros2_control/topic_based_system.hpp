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

/* Author: Jafar Abdi
   Desc: ros2_control system interface for topic based sim
*/

#pragma once

// C++
#include <memory>
#include <string>

// ROS
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

namespace topic_based_ros2_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TopicBasedSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_states_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_commands_publisher_;
  rclcpp::Node::SharedPtr node_;
  sensor_msgs::msg::JointState latest_joint_state_;

  /// Use standard interfaces for joints because they are relevant for dynamic behavior
  std::array<std::string, 4> standard_interfaces_ = { hardware_interface::HW_IF_POSITION,
                                                      hardware_interface::HW_IF_VELOCITY,
                                                      hardware_interface::HW_IF_ACCELERATION,
                                                      hardware_interface::HW_IF_EFFORT };

  struct MimicJoint
  {
    std::size_t joint_index;
    std::size_t mimicked_joint_index;
    double multiplier = 1.0;
  };
  std::vector<MimicJoint> mimic_joints_;

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> joint_commands_;
  std::vector<double> last_position_command_;
  std::vector<std::vector<double>> joint_states_;

  template <typename HandleType>
  bool getInterface(const std::string& name, const std::string& interface_name, const size_t vector_index,
                    std::vector<std::vector<double>>& values, std::vector<HandleType>& interfaces);
};

}  // namespace topic_based_ros2_control
