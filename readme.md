# 5 degree of freedom planar robot

## Project overview
This repository contains files that control either the simulated model or the real 5-DOF planar robot. The robot consists of five dynamixel MX-28 servomotors connected with 3D printed links.

![text](https://github.com/fenixkz/ros_snake_robot/blob/kinetic/5dofrobot.jpg)

The simulated model is spawned in the Gazebo engine. Also, visualization is available in RViz. Everything is interconnected within Robot Operating System (ROS). 

## Prerequisites 
The project has been implemented on Ubuntu 16.04 machine with ROS Kinetic. To be able to run the project ROS must be installed on your local machine. Also, the following libraries are required:

  - Gazebo
  - MoveIt

## Running
#### Simulation
How to launch moveit:

```
$ roslaunch gazebo_robot moveit_gazebo.launch

$ roslaunch moveit_arm moveit_planning_execution.launch
```
Then, the RViz will be opened and you can use the GUI to control the robot in the Gazebo simulation.
#### Real robot
The robot must be connected to a power source providing 12V. Also, an appropriate USB2Dynamixel adapter should be used to connect the robot to the PC. After that run:
```
$ roslaunch my_dynamixel_tutorial controller_manager.launch
```
The output "5 motors found"
Then, start the controllers:
```
$ roslaunch my_dynamixel_tutorial start_moveit_arm_controllers.launch
```
After that corresponding topics will appear:
motortom2m - first joint
joint2 - second joint
joint4 - third joint
joint6 - fourth joint
end - fifth joint
It is also possible to use MoveIt to control the real robot. For that a few changes should be made:

In **hand_tutorial_moveit_controller_manager.launch.xml** change:
```
  <rosparam file="$(find moveit_arm)/config/controllers.yaml"/>
  to
  <rosparam file="$(find moveit_arm)/config/controllers_real.yaml"/>
```

After that:
```
$ roslaunch moveit_arm moveit_planning_execution.launch
```

