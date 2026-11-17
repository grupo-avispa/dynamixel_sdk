// Copyright 2021 ROBOTIS CO., LTD.
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

#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


class ReadWriteNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;
  using JointState = sensor_msgs::msg::JointState;

  ReadWriteNode();
  virtual ~ReadWriteNode();

  /// Opens the serial port and starts the servomotor setup in the background.
  bool initialize();

  /// Sends the final pose and disables the torque.
  void shutdown();

private:
  /// Retries the initial pose until the servomotor answers.
  void runSetup(uint8_t id, uint32_t initial_pose);

  /// Moves the servomotor to a goal position (ticks), using 2 or 4 byte
  /// registers depending on the two_byte_registers parameter.
  void writeGoalPosition(uint8_t id, uint32_t position);

  /// Reads the present position (ticks), using 2 or 4 byte registers.
  uint32_t readPresentPosition(uint8_t id);

  /// Bridge from /joint_states (angle in radians) to the servomotor.
  void jointStateCallback(const JointState::SharedPtr msg);

  /// Converts an angle in radians to a position in ticks (clamped to the
  /// [min_position, max_position] range).
  uint32_t angleToPosition(double angle) const;

  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Subscription<JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;

  std::thread setup_thread_;

  // Joint bridge configuration (angle <-> ticks conversion)
  std::string joint_name_;
  double min_angle_;
  double max_angle_;
  int min_position_;
  int max_position_;
  bool two_byte_registers_;
  int moving_speed_;

  int present_position;
};

#endif  // READ_WRITE_NODE_HPP_
