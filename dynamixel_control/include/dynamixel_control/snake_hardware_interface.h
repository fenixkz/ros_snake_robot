#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "string"
#include "vector"
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_LOAD     126
#define DYN2RAD               4096/6.28

class SnakeRobot : public hardware_interface::RobotHW
{
    public:
        SnakeRobot(ros::NodeHandle& nh);
        ~SnakeRobot();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);

    protected:
        void registerJoints();
        
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;

        joint_limits_interface::JointLimits limits;
        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;


        double joint_position_[5];
        double joint_velocity_[5];
        double joint_effort_[5];
        double joint_position_command_[5];
        double protocol_version;
        int baudrate;
        std::string device_name;
        std::vector<int> IDs;
        std::vector<int> init_pose;
        std::vector<uint32_t> positions;
        dynamixel::PortHandler * portHandler;
        dynamixel::PacketHandler * packetHandler;
        ros::NodeHandle nh_;
        ros::Timer my_control_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
        boost::shared_ptr<dynamixel::GroupBulkRead> bulkRead;
        boost::shared_ptr<dynamixel::GroupBulkWrite> bulkWrite;
};
