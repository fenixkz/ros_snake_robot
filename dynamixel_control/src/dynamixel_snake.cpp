#include <dynamixel_control/dynamixel_snake.h>

dynamixel_snake::dynamixel_snake(ros::NodeHandle& nh) : nh_(nh) {
  ros::param::param<double>("/protocol", protocol_version, 2.0);
  ros::param::param<int>("/baudrate", baudrate, 57600);
  ros::param::param<std::string>("/device_name", device_name, "/dev/ttyUSB0");
  ros::param::get("/IDs", IDs);
  ros::param::get("/initial_position", init_pose);
// Dynamixel RS485 protocol initialization
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
  packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return;
  }

  if (!portHandler->setBaudRate(baudrate)) {
    ROS_ERROR("Failed to set the baudrate!");
    return;
  }

  for (int i = 0; i < IDs.size(); i++){
    dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, IDs[i], ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      ROS_ERROR("Failed to enable torque for Dynamixel ID %d", IDs[i]);
      return;
    }
  }

}
dynamixel_snake::~dynamixel_snake() {
}
