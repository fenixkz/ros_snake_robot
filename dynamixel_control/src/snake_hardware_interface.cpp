#include <dynamixel_control/snake_hardware_interface.h>


SnakeRobot::SnakeRobot(ros::NodeHandle& nh) : nh_(nh) {
  // Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
  init();

  // Create the controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

  //Set the frequency of the control loop.
  loop_hz_=100;
  ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

  //Run the control loop
  my_control_loop_ = nh_.createTimer(update_freq, &SnakeRobot::update, this);
}

SnakeRobot::~SnakeRobot() {
    // Clean up any resources used by the robot
    // Close the port
    portHandler->closePort();
}

void SnakeRobot::init() {
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
    if (!portHandler->setBaudRate(baudrate)) {
      ROS_ERROR("Failed to set the baudrate!");
      return;
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
    // Register all joints for hardware interface
    SnakeRobot::registerJoints();
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


void SnakeRobot::update(const ros::TimerEvent& e) {
    /**
     * @brief Update function for the SnakeRobot. Basically the control loop.
     * 
     * This function is called periodically to update the robot's state and control.
     * It performs the following steps:
     * 1. Calculates the elapsed time since the last update.
     * 2. Reads the current state of the robot's joints.
     * 3. Updates the controller manager.
     * 4. Writes new commands to the robot's joints.
     * 
     * @param e The ROS timer event containing timing information.
     */
    
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}


void SnakeRobot::read(){
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

void SnakeRobot::write(ros::Duration elapsed_time) {
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  uint8_t param_goal_position[4];
  // Safety
  positionJointSaturationInterface.enforceLimits(elapsed_time); 
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

int main(int argc, char** argv)
{
    //Initialze the ROS node.
    ros::init(argc, argv, "snake_hardware_interface_node");
    ros::NodeHandle nh;

    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(1);

    // Create the object of the robot hardware_interface class and spin the thread.
    SnakeRobot ROBOT(nh);
    spinner.spin();

    return 0;
}
