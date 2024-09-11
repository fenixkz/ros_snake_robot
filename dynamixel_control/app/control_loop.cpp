#include <ros/ros.h>
#include <dynamixel_control/snake_hardware_interface.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
    //Initialze the ROS node.
    ros::init(argc, argv, "snake_hardware_interface_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~"); // To use namespace of a launch file
    
    bool use_sim = false;
    nh_priv.getParam("use_sim", use_sim);
    
    SnakeRobot robot(nh, use_sim);
    controller_manager::ControllerManager cm(&robot, nh);
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::AsyncSpinner spinner(4); 

    // Create the object of the robot hardware_interface class and spin the thread.
    spinner.start();

    while(ros::ok())
    {
        ros::Time now = robot.getTime();
        ros::Duration dt = robot.getPeriod();

        robot.read();
        
        cm.update(now, dt);

        robot.write(dt);
        
        dt.sleep();
    }
    spinner.stop();
    return 0;
}
