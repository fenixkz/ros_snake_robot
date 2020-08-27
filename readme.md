New version

How to launch moveit:

First, launch gazebo by:

roslaunch gazebo_robot moveit_gazebo.launch

Then, launch rviz with moveit:

roslaunch moveit_arm moveit_planning_execution.launch

---------------------------------------------------------

Control of the real robot:
After connecting the robot to power source and to PC, launch the controller manager:

roslaunch my_dynamixel_tutorial controller_manager.launch

It should output "5 motors found"
Then, start the controllers:

roslaunch my_dynamixel_tutorial start_moveit_arm_controllers.launch

After that corresponding topics will appear:
motortom2m - first joint
joint2 - second joint
joint4 - third joint
joint6 - fourth joint
end - fifth joint
