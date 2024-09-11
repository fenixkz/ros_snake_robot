#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "string"
#include "vector"
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
        SnakeRobot(ros::NodeHandle& nh, bool use_sim);
        ~SnakeRobot();
        void read();
        void write(ros::Duration dt);
        ros::Time getTime() const { return ros::Time::now(); }
        ros::Duration getPeriod() const { return ros::Duration(0.01); }

    private:
        void registerJoints();
        void startDXL();
        void getParams();
        int baudrate;
        bool is_sim;
        double protocol_version;
        std::string device_name;
        std::vector<int> IDs;
        std::vector<int> init_pose;
        std::vector<uint32_t> positions;
        dynamixel::PortHandler * portHandler;
        dynamixel::PacketHandler * packetHandler;
        std::shared_ptr<dynamixel::GroupBulkRead> bulkRead;
        std::shared_ptr<dynamixel::GroupBulkWrite> bulkWrite;
        ros::NodeHandle nh_;
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        double joint_position_[5];
        double joint_velocity_[5];
        double joint_effort_[5];
        double joint_position_command_[5];
     
};
