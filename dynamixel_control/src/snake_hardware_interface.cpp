#include <dynamixel_control/snake_hardware_interface.h>


SnakeRobot::SnakeRobot(ros::NodeHandle& nh, bool use_sim) : nh_(nh), is_sim(use_sim) {
  getParams();
  if (!is_sim){
    startDXL();
  }else{
    ROS_INFO_STREAM("Starting in simulation mode");
  }
  registerJoints();
}

SnakeRobot::~SnakeRobot() {
    // Clean up any resources used by the robot
    // Close the port
    if (!is_sim){
      portHandler->closePort();
    }
}

void SnakeRobot::getParams(){
  ros::param::param<double>("/protocol", protocol_version, 2.0);
  ros::param::param<int>("/baudrate", baudrate, 57600);
  ros::param::param<std::string>("/device_name", device_name, "/dev/ttyUSB0");
  ros::param::get("/IDs", IDs);
  ros::param::get("/initial_position", init_pose);
}

void SnakeRobot::startDXL() {
    // Dynamixel RS485 protocol initialization
    
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    portHandler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);
    bulkRead.reset(new dynamixel::GroupBulkRead(portHandler, packetHandler));
    bulkWrite.reset(new dynamixel::GroupBulkWrite(portHandler, packetHandler));
    positions.resize(5);
    // Initialize PortHandler instance, open the port
    ROS_INFO_STREAM("Trying to open the port: " << device_name << " with baudrate: "<< baudrate);
    if (!portHandler->openPort()) {
      ROS_ERROR("Failed to open the port!");
      return;
    }
    ROS_INFO_STREAM("Port is open");
    // Set port baudrate
    ROS_INFO("Setting the baudrate: %d", baudrate);
    if (!portHandler->setBaudRate(baudrate)){
      ROS_ERROR("Failed to set the baudrate!");
      return;
    }
    ROS_INFO("Baudrate set");
    // Ping the motors
    for (int i = 0; i < IDs.size(); i++){
        uint16_t model_number = 0;
        dxl_comm_result = packetHandler->ping(portHandler, IDs[i], &model_number, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            ROS_ERROR("Failed to ping Dynamixel ID %d", IDs[i]);
            ROS_ERROR("Dynamixel error: %s", packetHandler->getRxPacketError(dxl_error));
            return;
        }else{
            ROS_INFO("Dynamixel ID %d pinged successfully. Model number: %d", IDs[i], model_number);
        }
    }
    // Enable Dynamixel Torque
    for (int i = 0; i < IDs.size(); i++){
      ROS_INFO("Enabling torque for Dynamixel ID %d", IDs[i]);
      dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, IDs[i], ADDR_TORQUE_ENABLE, 1, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to enable torque for Dynamixel ID %d", IDs[i]);
        return;
      }
      ROS_INFO("Torque enabled for Dynamixel ID %d", IDs[i]);
    }
}

void SnakeRobot::registerJoints(){
  // Create interfaces for all joints
  for (int i = 0; i < 5; ++i) {
      std::string joint_name = "Joint" + std::to_string(i + 1);
      
      // Create joint_state_interface
      hardware_interface::JointStateHandle jointStateHandle(joint_name, &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
      joint_state_interface_.registerHandle(jointStateHandle);

      // Create position joint interface
      hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
      position_joint_interface_.registerHandle(jointPositionHandle);

      // Create Joint Limit interface
      joint_limits_interface::getJointLimits(joint_name, nh_, limits);
      joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle(jointPositionHandle, limits);
      positionJointSaturationInterface.registerHandle(jointLimitsHandle);
  }

  // Register all joints interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&positionJointSaturationInterface);
}





void SnakeRobot::read(){
  if (is_sim){
    // Just forward the command to the sensor values in sim mode
    for (size_t i = 0; i < IDs.size(); i++){
      joint_position_[i] = joint_position_command_[i];
      joint_velocity_[i] = 0;
      joint_effort_[i] = 0;
    }
  }else{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;
    // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
    // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
    for (int i = 0; i < IDs.size(); i++){
      dxl_addparam_result = bulkRead->addParam((uint8_t)IDs[i], ADDR_PRESENT_POSITION, 4);
    }

    dxl_comm_result = bulkRead->txRxPacket();
    if (dxl_comm_result == COMM_SUCCESS) {
      for (int i = 0; i < IDs.size(); i++){
        int32_t enc = (int32_t)bulkRead->getData((uint8_t)IDs[i], ADDR_PRESENT_POSITION, 4);
        joint_position_[i] = (enc - init_pose[i]) * 6.28 / 4096;
      }
      bulkRead->clearParam();
    }
    // Read Present Velocity (length : 4 bytes) 
    for (int i = 0; i < IDs.size(); i++){
      dxl_addparam_result = bulkRead->addParam((uint8_t)IDs[i], ADDR_PRESENT_VELOCITY, 4);
    }
    dxl_comm_result = bulkRead->txRxPacket();
    if (dxl_comm_result == COMM_SUCCESS) {
      for (int i = 0; i < IDs.size(); i++){
        joint_velocity_[i] = (int32_t)bulkRead->getData((uint8_t)IDs[i], ADDR_PRESENT_VELOCITY, 4);
        }
      bulkRead->clearParam();
    }
    // Read Present Load (length : 2 bytes)
    for (int i = 0; i < IDs.size(); i++){
      dxl_addparam_result = bulkRead->addParam((uint8_t)IDs[i], ADDR_PRESENT_LOAD, 2);
    }
    dxl_comm_result = bulkRead->txRxPacket();
    if (dxl_comm_result == COMM_SUCCESS) {
      for (int i = 0; i < IDs.size(); i++){
        joint_effort_[i] = (int16_t)bulkRead->getData((uint8_t)IDs[i], ADDR_PRESENT_LOAD, 2);
        }
      bulkRead->clearParam();
    }
  }
}

void SnakeRobot::write(ros::Duration dt) {
  if (!is_sim){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;
    uint8_t param_goal_position[4];
    // Safety
    positionJointSaturationInterface.enforceLimits(dt); 
    // Write Goal Position (length : 4 bytes)
    for (int i = 0; i < IDs.size(); i++){
      double pos = joint_position_command_[i] * 4096 / 6.28 + init_pose[i];
      positions[i] = (unsigned int)pos;
      // ROS_INFO("Joint %d: command %f\n", i, joint_position_command_[i]);
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(positions[i]));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(positions[i]));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(positions[i]));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(positions[i]));
      dxl_addparam_result = bulkWrite->addParam((uint8_t)IDs[i], ADDR_GOAL_POSITION, 4, param_goal_position);
    }
    dxl_comm_result = bulkWrite->txPacket();

    bulkWrite->clearParam();
  }
}

