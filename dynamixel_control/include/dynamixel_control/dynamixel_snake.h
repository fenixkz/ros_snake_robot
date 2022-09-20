#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/BulkGetItem.h"
#include "dynamixel_sdk_examples/BulkSetItem.h"
#include "dynamixel_sdk/dynamixel_sdk.h"





class dynamixel_snake:
{
  public:
    dynamixel_snake(ros::NodeHandle& nh);
    ~dynamixel_snake();
    void init();
    double read_joint();
    void write_position();
  protected:
    double protocol_version;
    int baudrate;
    std::string device_name;
    std::vector<int> IDs;
    std::vector<int> init_pose;
    dynamixel::PortHandler * portHandler;
    dynamixel::PacketHandler * packetHandler;
    ros::NodeHandle nh_;
}
