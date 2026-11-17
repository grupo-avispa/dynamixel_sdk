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

/*******************************************************************************
// Node controlling the Morphia tablet servomotor (DYNAMIXEL with U2D2).
//
// The serial device and all motion values are configurable as node
// parameters (device_name, baudrate, protocol_version, id, initial_pose,
// pose_down). If the servomotor bus is not ready yet (e.g. right after a
// reboot, while the robot finishes its power-up sequence), the initial pose
// setup keeps retrying in the background until the servomotor answers.
*******************************************************************************/

#include <cstdio>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <signal.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "read_write_node.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_MOVING_SPEED 32
#define ADDR_PRESENT_POSITION 36

// Default settings in radians (can be overridden with node parameters)
#define DEFAULT_BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEFAULT_DEVICE_NAME "/dev/ttyUSBDynamixel"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"
const double DEFAULT_PROTOCOL_VERSION = 1.0;
const double DEFAULT_POSE_INIT = 0.0;
const double DEFAULT_POSE_DOWN = -2.8274;
const int DEFAULT_DXL_ID = 1;
unsigned int POSE_DOWN = DEFAULT_POSE_DOWN;
uint8_t ID = DEFAULT_DXL_ID;

dynamixel::PortHandler * portHandler = nullptr;
dynamixel::PacketHandler * packetHandler = nullptr;

// Protects the serial port, shared between the setup thread, the callbacks
// and the shutdown sequence.
static std::mutex port_mutex;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

void setupDynamixel(uint8_t dxl_id);

ReadWriteNode::ReadWriteNode()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  // Dynamixel configuration (see params/morphia.yaml). Poses in radians.
  this->declare_parameter("device_name", std::string(DEFAULT_DEVICE_NAME));
  this->declare_parameter("baudrate", DEFAULT_BAUDRATE);
  this->declare_parameter("protocol_version", DEFAULT_PROTOCOL_VERSION);
  this->declare_parameter("id", DEFAULT_DXL_ID);
  this->declare_parameter("initial_pose", DEFAULT_POSE_INIT);
  this->declare_parameter("pose_down", DEFAULT_POSE_DOWN);

  // Joint bridge: move the servomotor from /joint_states (angle in radians),
  // e.g. with the sliders of joint_state_publisher_gui.
  this->declare_parameter("joint_name", std::string(""));
  this->declare_parameter("min_angle", -M_PI);
  this->declare_parameter("max_angle", M_PI);
  this->declare_parameter("min_position", 0);
  this->declare_parameter("max_position", 4095);
  // MX (firmware 1.0) and AX series use 2-byte position registers; X series
  // and MX with firmware 2.0 use 4-byte registers.
  this->declare_parameter("two_byte_registers", true);
  // Moving speed in ticks/s for MX-28 fw1.0 (0 = max speed, only applied
  // when two_byte_registers is true). 0 also means "do not touch".
  this->declare_parameter("moving_speed", 0);

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_position_subscriber_ =
    this->create_subscription<SetPosition>(
    "set_position",
    QOS_RKL10V,
    [this](const SetPosition::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      // Position Value of X series is 4 byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
      uint32_t goal_position = (unsigned int)msg->position;  // Convert int32 -> uint32

      // Write Goal Position (length : 4 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      {
        std::lock_guard<std::mutex> lock(port_mutex);
        dxl_comm_result =
        packetHandler->write4ByteTxRx(
          portHandler, (uint8_t) msg->id, ADDR_GOAL_POSITION, goal_position, &dxl_error);
      }

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
      }
    }
    );

  auto get_present_position =
    [this](
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response) -> void
    {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      present_position = static_cast<int>(readPresentPosition((uint8_t) request->id));

      RCLCPP_INFO(
        this->get_logger(), "Get [ID: %d] [Present Position: %d]", request->id, present_position);

      response->position = present_position;
    };

  get_position_server_ = create_service<GetPosition>("get_position", get_present_position);

  // Bridge from /joint_states to the servomotor (disabled if no joint name)
  joint_name_ = this->get_parameter("joint_name").as_string();
  min_angle_ = this->get_parameter("min_angle").as_double();
  max_angle_ = this->get_parameter("max_angle").as_double();
  min_position_ = static_cast<int>(this->get_parameter("min_position").as_int());
  max_position_ = static_cast<int>(this->get_parameter("max_position").as_int());
  two_byte_registers_ = this->get_parameter("two_byte_registers").as_bool();
  moving_speed_ = static_cast<int>(this->get_parameter("moving_speed").as_int());

  if (!joint_name_.empty()) {
    joint_state_subscriber_ = this->create_subscription<JointState>(
      "joint_states", 10,
      std::bind(&ReadWriteNode::jointStateCallback, this, std::placeholders::_1));
  }
}

ReadWriteNode::~ReadWriteNode()
{
  if (setup_thread_.joinable()) {
    setup_thread_.join();
  }
}

bool ReadWriteNode::initialize()
{
  const auto device_name = this->get_parameter("device_name").as_string();
  const auto baudrate = static_cast<int>(this->get_parameter("baudrate").as_int());
  const auto protocol_version = static_cast<float>(this->get_parameter("protocol_version").as_double());
  const auto id = static_cast<uint8_t>(this->get_parameter("id").as_int());
  const auto initial_pose = angleToPosition(this->get_parameter("initial_pose").as_double());
  POSE_DOWN = angleToPosition(this->get_parameter("pose_down").as_double());
  ID = id;

  portHandler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
  packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  // Open Serial Port. Retry a few times: right after a reboot the udev symlink
  // may not be ready yet.
  bool port_opened = false;
  for (int attempt = 1; attempt <= 5 && !port_opened; ++attempt) {
    if (attempt > 1) {
      RCLCPP_WARN(this->get_logger(), "Retrying to open the port (%d/5)...", attempt);
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
    port_opened = portHandler->openPort();
  }
  if (!port_opened) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the port %s!", device_name.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Succeeded to open the port %s.", device_name.c_str());

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  if (!portHandler->setBaudRate(baudrate)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Succeeded to set the baudrate.");

  // Move to the initial pose in the background: if the robot power-up sequence
  // is still running, the bus stays silent for a while, so keep retrying until
  // the servomotor answers. The node remains alive and fully functional in the
  // meantime (set_position topic, get_position service).
  setup_thread_ = std::thread([this, id, initial_pose]() {runSetup(id, initial_pose);});
  return true;
}

void ReadWriteNode::runSetup(uint8_t id, uint32_t initial_pose)
{
  int attempt = 0;
  uint8_t error = 0;
  int result = COMM_TX_FAIL;
  while (rclcpp::ok()) {
    attempt++;
    writeGoalPosition(id, initial_pose);
    result = dxl_comm_result;
    error = dxl_error;
    if (result == COMM_SUCCESS && error == 0) {
      break;
    }
    if (attempt % 10 == 0) {
      RCLCPP_WARN(
        this->get_logger(), "Servomotor not ready after %d attempts: %s", attempt,
        result != COMM_SUCCESS ? packetHandler->getTxRxResult(result) :
        packetHandler->getRxPacketError(error));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  if (!rclcpp::ok()) {
    return;
  }
  RCLCPP_INFO(
    this->get_logger(), "Succeeded to set the initial pose (position %u) after %d attempt(s).",
    static_cast<unsigned>(initial_pose), attempt);

  // Limit the servomotor speed (MX-28 fw1.0: 0 would be the maximum speed)
  if (two_byte_registers_ && moving_speed_ > 0) {
    std::lock_guard<std::mutex> lock(port_mutex);
    uint8_t speed_error = 0;
    packetHandler->write2ByteTxRx(
      portHandler, id, ADDR_MOVING_SPEED, static_cast<uint16_t>(moving_speed_), &speed_error);
  }

  setupDynamixel(BROADCAST_ID);
}

uint32_t ReadWriteNode::angleToPosition(double angle) const
{
  double tick = min_position_ +
    (angle - min_angle_) / (max_angle_ - min_angle_) * (max_position_ - min_position_);
  tick = std::max(static_cast<double>(min_position_),
    std::min(static_cast<double>(max_position_), tick));
  return static_cast<uint32_t>(std::lround(tick));
}

void ReadWriteNode::writeGoalPosition(uint8_t id, uint32_t position)
{
  std::lock_guard<std::mutex> lock(port_mutex);
  if (two_byte_registers_) {
    // MX (firmware 1.0) and AX series: 2-byte Goal Position register
    dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler, id, ADDR_GOAL_POSITION, static_cast<uint16_t>(position), &dxl_error);
  } else {
    // X series and MX (firmware 2.0): 4-byte Goal Position register
    dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, id, ADDR_GOAL_POSITION, position, &dxl_error);
  }
}

uint32_t ReadWriteNode::readPresentPosition(uint8_t id)
{
  std::lock_guard<std::mutex> lock(port_mutex);
  uint32_t position = 0;
  if (two_byte_registers_) {
    uint16_t value = 0;
    dxl_comm_result = packetHandler->read2ByteTxRx(
      portHandler, id, ADDR_PRESENT_POSITION, &value, &dxl_error);
    position = value;
  } else {
    dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, id, ADDR_PRESENT_POSITION,
      reinterpret_cast<uint32_t *>(&present_position), &dxl_error);
    position = static_cast<uint32_t>(present_position);
  }
  return position;
}

void ReadWriteNode::jointStateCallback(const JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] != joint_name_) {
      continue;
    }
    const uint32_t tick = angleToPosition(msg->position[i]);
    writeGoalPosition(ID, tick);
    if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
      RCLCPP_INFO(
        this->get_logger(), "Joint '%s' angle %.3f rad -> position %u", joint_name_.c_str(),
        msg->position[i], static_cast<unsigned>(tick));
    } else if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else {
      RCLCPP_WARN(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
    }
    return;
  }
}

void ReadWriteNode::shutdown()
{
  // Send the final pose
  writeGoalPosition(ID, POSE_DOWN);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "Failed to send final pose.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to send final pose.");
  }

  // Disable Torque of DYNAMIXEL
  std::lock_guard<std::mutex> lock(port_mutex);
  uint8_t error = 0;
  packetHandler->write1ByteTxRx(portHandler, BROADCAST_ID, ADDR_TORQUE_ENABLE, 0, &error);
}

void setupDynamixel(uint8_t dxl_id)
{
  // Enable Torque of DYNAMIXEL
  std::lock_guard<std::mutex> lock(port_mutex);
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}

void signalHandler(int signum)
{
  std::cout << std::endl << "Interrupt signal (" << signum << ") received." << std::endl;
  // Let main() finish the cleanup (final pose, disable torque) instead of
  // calling exit() from the signal handler, which aborts with std::system_error.
  rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();

  if (!readwritenode->initialize()) {
    rclcpp::shutdown();
    return -1;
  }

  signal(SIGINT, signalHandler);

  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Send the final pose and disable torque of DYNAMIXEL
  readwritenode->shutdown();

  return 0;
}
