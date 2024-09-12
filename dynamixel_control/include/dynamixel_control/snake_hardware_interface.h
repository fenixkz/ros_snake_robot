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

#define DYN2RAD 4096/6.28

struct DXLMemoryRegister{
        constexpr DXLMemoryRegister(uint16_t addr, uint8_t len)
        : address(addr), length(len) {};

        constexpr DXLMemoryRegister()
        : address(0), length(0) {};

        uint16_t address;
        uint16_t length;
    };

class ControlTable{
    public:
        ControlTable() = default; // a default constructor to compile 

        // This control table is only applicable to MX64-AT/R, for different models the addresses and lengths can be different
        // Refer to emanual for more details: https://emanual.robotis.com/docs/en/dxl/mx/mx-28/
        ControlTable(size_t protocol_version){
            if (protocol_version == 1.0){
                model_number = DXLMemoryRegister(0, 2);
                id = DXLMemoryRegister{3, 1};
                torque = DXLMemoryRegister{24, 1};
                goal_position = DXLMemoryRegister{30, 2};
                current_position = DXLMemoryRegister{36, 2};
                current_velocity = DXLMemoryRegister{38, 2};
                current_load = DXLMemoryRegister{40, 2};
                goal_velocity = std::nullopt;
                goal_current = std::nullopt;
            }else{ // protocol 2.0
                model_number = DXLMemoryRegister(0, 2);
                id = DXLMemoryRegister{7, 1};
                torque = DXLMemoryRegister{64, 1};
                current_position = DXLMemoryRegister{132, 4};
                current_velocity = DXLMemoryRegister{128, 4};
                current_load = DXLMemoryRegister{126, 4};
                goal_position  = DXLMemoryRegister{116, 4};
                goal_velocity = DXLMemoryRegister{104, 4};
                goal_current = std::nullopt;
            }
        }

    
        DXLMemoryRegister id;
        DXLMemoryRegister model_number;
        DXLMemoryRegister torque;
        DXLMemoryRegister current_position;
        DXLMemoryRegister current_velocity;
        DXLMemoryRegister current_load;
        DXLMemoryRegister goal_position;
        std::optional<DXLMemoryRegister> goal_velocity;
        std::optional<DXLMemoryRegister> goal_current;
};


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
        bool startDXL();
        void getParams();
        bool setTorque(bool enable);
        bool setTorque(int id, bool enable);

        int baudrate;
        bool is_sim;
        size_t num_joints;
        double protocol_version;
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        std::string device_name;
        std::vector<int> IDs;
        std::vector<int> init_pose;
        std::vector<uint32_t> positions;
        std::unique_ptr<dynamixel::PortHandler> portHandler;
        std::unique_ptr<dynamixel::PacketHandler> packetHandler;
        std::shared_ptr<dynamixel::GroupBulkRead> bulkRead;
        std::shared_ptr<dynamixel::GroupBulkWrite> bulkWrite;
        ros::NodeHandle nh_;
        mutable std::mutex comms_mutex_;
        ControlTable control_table;

    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        std::vector<double>  joint_position_;
        std::vector<double>  joint_velocity_;
        std::vector<double>  joint_effort_;
        std::vector<double>  joint_position_command_;
     
};
