#include "ros/ros.h"
#include "robot_control/state.h"
#include "dynamixel_msgs/JointState.h"
#include "sensor_msgs/JointState.h"

  double pos1;
	double pos2;
	double pos3;
	double pos4;
	double pos5;
	double vel1;
	double vel2;
	double vel3;
	double vel4;
	double vel5;

    sensor_msgs::JointState state;


void jointCallback(const dynamixel_msgs::JointState::ConstPtr& msg) {

 pos1 = msg->current_pos;
 vel1 = msg->velocity;

	ROS_INFO(" pos 1: %f", pos1);
}

void joint1Callback(const dynamixel_msgs::JointState::ConstPtr& msg) {

 pos2 = msg->current_pos;
 vel2 = msg->velocity;
	ROS_INFO(" pos2: %f", pos2);
}

void joint2Callback(const dynamixel_msgs::JointState::ConstPtr& msg) {

 pos3 = msg->current_pos;
 vel3 = msg->velocity;
	ROS_INFO(" pos3: %f", pos3);
}

void joint3Callback(const dynamixel_msgs::JointState::ConstPtr& msg) {

 pos4 = msg->current_pos;
 vel4 = msg->velocity;
	ROS_INFO(" pos4: %f", pos4);
}

void joint4Callback(const dynamixel_msgs::JointState::ConstPtr& msg) {

 pos5 = msg->current_pos;
 vel5 = msg->velocity;
	ROS_INFO(" pos5: %f", pos5);
}

int main(int argc, char **argv){
ros::init(argc, argv, "moveit_motor_state");

ros::NodeHandle n1;
ros::NodeHandle n2;
ros::NodeHandle n3;
ros::NodeHandle n4;
ros::NodeHandle n5;
ros::NodeHandle nh;

ros::Subscriber sub1 = n1.subscribe("/motortom2m/state", 1000, jointCallback);
ros::Subscriber sub2 = n2.subscribe("/joint2/state", 1000, joint1Callback);
ros::Subscriber sub3 = n3.subscribe("/joint4/state", 1000, joint2Callback);
ros::Subscriber sub4 = n4.subscribe("/joint6/state", 1000, joint3Callback);
ros::Subscriber sub5 = n5.subscribe("/end/state", 1000, joint4Callback);

ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/robot/joint_states", 100);
state.name.resize(5);
state.position.resize(5);
state.name[0] = "motortom2m";
state.name[1] = "joint2";
state.name[2] = "joint4";
state.name[3] = "joint6";
state.name[4] = "end";




ros::Rate loop_rate(10);
while(ros::ok()){
state.header.stamp = ros::Time::now();
state.position[0] = pos1;
state.position[1] = pos2;
state.position[2] = pos3;
state.position[3] = pos4;
state.position[4] = pos5;
pub.publish(state);

ros::spinOnce();
loop_rate.sleep();
}
return 0;
}
