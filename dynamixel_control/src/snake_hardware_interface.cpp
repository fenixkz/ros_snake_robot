#include <dynamixel_control/snake_hardware_interface.h>

MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh) {


// Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();

// Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

//Set the frequency of the control loop.
    loop_hz_=100;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

//Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);

}

MyRobot::~MyRobot() {
}

void MyRobot::init() {

// Create joint_state_interface for Joint1
    hardware_interface::JointStateHandle jointStateHandle1("Joint1", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandle1);

// Create position joint interface as Joint1 accepts position command.
    hardware_interface::JointHandle jointPositionHandle1(jointStateHandle1, &joint_position_command_[0]);
    position_joint_interface_.registerHandle(jointPositionHandle1);

// Create Joint Limit interface for Joint1
    joint_limits_interface::getJointLimits("Joint1", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle1(jointPositionHandle1, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle1);

// Create joint_state_interface for Joint2
    hardware_interface::JointStateHandle jointStateHandle2("Joint2", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandle2);

// Create position joint interface as Joint2 accepts position command.
    hardware_interface::JointHandle jointPositionHandle2(jointStateHandle2, &joint_position_command_[1]);
    position_joint_interface_.registerHandle(jointPositionHandle2);

// Create Joint Limit interface for Joint2
    joint_limits_interface::getJointLimits("Joint2", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle2(jointPositionHandle2, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle2);

// Create joint_state_interface for Joint3
    hardware_interface::JointStateHandle jointStateHandle3("Joint3", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
    joint_state_interface_.registerHandle(jointStateHandle3);

// Create position joint interface as Joint3 accepts position command.
    hardware_interface::JointHandle jointPositionHandle3(jointStateHandle3, &joint_position_command_[2]);
    position_joint_interface_.registerHandle(jointPositionHandle3);

// Create Joint Limit interface for Joint3
    joint_limits_interface::getJointLimits("Joint3", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle3(jointPositionHandle3, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle3);

// Create joint_state_interface for Joint4
    hardware_interface::JointStateHandle jointStateHandle4("Joint4", &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]);
    joint_state_interface_.registerHandle(jointStateHandle4);

// Create position joint interface as Joint4 accepts position command.
    hardware_interface::JointHandle jointPositionHandle4(jointStateHandle4, &joint_position_command_[3]);
    position_joint_interface_.registerHandle(jointPositionHandle4);

// Create Joint Limit interface for Joint4
    joint_limits_interface::getJointLimits("Joint4", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle4(jointPositionHandle4, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle4);

// Create joint_state_interface for Joint5
    hardware_interface::JointStateHandle jointStateHandle5("Joint5", &joint_position_[4], &joint_velocity_[4], &joint_effort_[4]);
    joint_state_interface_.registerHandle(jointStateHandle5);

// Create position joint interface as Joint5 accepts position command.
    hardware_interface::JointHandle jointPositionHandle5(jointStateHandle5, &joint_position_command_[4]);
    position_joint_interface_.registerHandle(jointPositionHandle5);

// Create Joint Limit interface for Joint5
    joint_limits_interface::getJointLimits("Joint5", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle5(jointPositionHandle5, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle5);

// Register all joints interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&positionJointSaturationInterface);

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

//This is the control loop
void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}


void MyRobot::read(){
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
      // ROS_INFO("data: %d", enc);
    }
    bulkRead->clearParam();
  }

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

void MyRobot::write(ros::Duration elapsed_time) {
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  uint8_t param_goal_position[4];
  // Safety
  positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC
  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
  // the output commands need to send are joint_effort_command_[0] for JointA, joint_effort_command_[1] for JointB and
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
  //joint_position_command_ for JointC.
}

int main(int argc, char** argv)
{
    //Initialze the ROS node.
    ros::init(argc, argv, "snake_hardware_interface_node");
    ros::NodeHandle nh;

    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(0);

    // Create the object of the robot hardware_interface class and spin the thread.
    MyRobot ROBOT(nh);
    spinner.spin();

    return 0;
}
