#include <ros/ros.h>
#include <std_msgs/Float64.h>


int main(int argc, char **argv){
  init(argc, argv, "area");
  ros::NodeHandle nh;
  ros::NodeHandle nh1;
  ros::NodeHandle nh2;
  ros::NodeHandle nh3;
  ros::NodeHandle nh4;
  ros::Publisher pub = nh.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 100);
  ros::Publisher pub1 = nh1.advertise<std_msgs::Float64>("/robot/joint2_position_controller/command", 100);
  ros::Publisher pub2 = nh2.advertise<std_msgs::Float64>("/robot/joint3_position_controller/command", 100);
  ros::Publisher pub3 = nh3.advertise<std_msgs::Float64>("/robot/joint4_position_controller/command", 100);
  ros::Publisher pub4 = nh4.advertise<std_msgs::Float64>("/robot/joint5_position_controller/command", 100);
  ros::Subscriber sub = nh5.subscribe("/robot/joint_states", 1000, coordCallback);
}
