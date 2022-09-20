// Copyright 2020 ROBOTIS CO., LTD.
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



#include <ros/ros.h>

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "dynamixel_sdk_examples/GetPosition.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "string"
#include "vector"
using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132

PortHandler * portHandler;
PacketHandler * packetHandler;

std::vector<int> IDs;
std::vector<int> bias;
double protocol_version;
int baudrate;
std::string device_name;

void joint1PositionCallback(const std_msgs::Float32::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int pos = std::floor(msg->data * 2048 / 3.14 + bias[0]);

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position = (unsigned int)pos; // Convert int32 -> uint32


  if (abs(msg->data) <= 1.57){
    // Write Goal Position (length : 4 bytes)
    // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
    dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, (uint8_t)IDs[0], ADDR_GOAL_POSITION, position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("setPosition : [ID:%d] [POSITION:%f]", IDs[0], msg->data);
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
  }else{
    ROS_ERROR("Limits are: [-1.57; 1.57]");
  }

}

void joint2PositionCallback(const std_msgs::Float32::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int pos = std::floor(msg->data * 2048 / 3.14 + bias[0]);

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position = (unsigned int)pos; // Convert int32 -> uint32


  if (abs(msg->data) <= 1.57){
    // Write Goal Position (length : 4 bytes)
    // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
    dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, (uint8_t)IDs[0], ADDR_GOAL_POSITION, position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("setPosition : [ID:%d] [POSITION:%f]", IDs[0], msg->data);
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
  }else{
    ROS_ERROR("Limits are: [-1.57; 1.57]");
  }

}

void joint3PositionCallback(const std_msgs::Float32::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int pos = std::floor(msg->data * 2048 / 3.14 + bias[0]);

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position = (unsigned int)pos; // Convert int32 -> uint32


  if (abs(msg->data) <= 1.57){
    // Write Goal Position (length : 4 bytes)
    // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
    dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, (uint8_t)IDs[0], ADDR_GOAL_POSITION, position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("setPosition : [ID:%d] [POSITION:%f]", IDs[0], msg->data);
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
  }else{
    ROS_ERROR("Limits are: [-1.57; 1.57]");
  }

}

void joint4PositionCallback(const std_msgs::Float32::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int pos = std::floor(msg->data * 2048 / 3.14 + bias[0]);

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position = (unsigned int)pos; // Convert int32 -> uint32


  if (abs(msg->data) <= 1.57){
    // Write Goal Position (length : 4 bytes)
    // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
    dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, (uint8_t)IDs[0], ADDR_GOAL_POSITION, position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("setPosition : [ID:%d] [POSITION:%f]", IDs[0], msg->data);
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
  }else{
    ROS_ERROR("Limits are: [-1.57; 1.57]");
  }

}

void joint5PositionCallback(const std_msgs::Float32::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int pos = std::floor(msg->data * 2048 / 3.14 + bias[0]);

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position = (unsigned int)pos; // Convert int32 -> uint32


  if (abs(msg->data) <= 1.57){
    // Write Goal Position (length : 4 bytes)
    // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
    dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, (uint8_t)IDs[0], ADDR_GOAL_POSITION, position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("setPosition : [ID:%d] [POSITION:%f]", IDs[0], msg->data);
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
  }else{
    ROS_ERROR("Limits are: [-1.57; 1.57]");
  }

}
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "joint_control");
  ros::NodeHandle nh;
  // Getting parameters from .yaml file
  ros::param::get("/IDs", IDs);
  ros::param::get("/initial_position", bias);
  ros::param::param<double>("/protocol", protocol_version, 2.0);
  ros::param::param<int>("/baudrate", baudrate, 57600);
  ros::param::param<std::string>("/device_name", device_name, "/dev/ttyUSB0");


  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(device_name.c_str());
  packetHandler = PacketHandler::getPacketHandler(protocol_version);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(baudrate)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  for (int i = 0; i < IDs.size(); i++){
    dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, IDs[i], ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      ROS_ERROR("Failed to enable torque for Dynamixel ID %d", IDs[i]);
      return -1;
    }
  }


  // ros::ServiceServer get_position_srv = nh.advertiseService("/get_position", getPresentPositionCallback);
  ros::Subscriber joint1_position_sub = nh.subscribe("/joint1_position/command", 10, joint1PositionCallback);
  ros::Subscriber joint2_position_sub = nh.subscribe("/joint2_position/command", 10, joint2PositionCallback);
  ros::Subscriber joint3_position_sub = nh.subscribe("/joint3_position/command", 10, joint3PositionCallback);
  ros::Subscriber joint4_position_sub = nh.subscribe("/joint4_position/command", 10, joint4PositionCallback);
  ros::Subscriber joint5_position_sub = nh.subscribe("/joint5_position/command", 10, joint5PositionCallback);
  ros::spin();
  //
  // portHandler->closePort();
  return 0;
}
