# 5 degree of freedom planar robot

## Project overview
This repository contains files that control either the simulated model or the real 5-DOF planar robot. The robot consists of five dynamixel MX-28 servomotors connected with 3D printed links.

![text](https://github.com/fenixkz/ros_snake_robot/blob/kinetic/5dofrobot.jpg)

Everything is interconnected within Robot Operating System (ROS) Melodic distribution. 

## Prerequisites 
The project has been implemented on Ubuntu 18.04 machine with ROS Melodic. Dynamixel motor must have Protocol 2.0 in order to work with the current library. 

To be able to run the project ROS must be installed on your local machine. Also, the following libraries are required:

  - MoveIt
  - ros_controllers
  - [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)

## Running

The robot must be connected to a power source providing 12V. Also, an appropriate USB2Dynamixel/U2D2 adapter should be used to connect the robot to the PC. After that run:
```
roslaunch dynamixel_control snake_control.launch
```
After that 5 Position Controller should be spawned.

Each joint has a name: Joint#, where # is the number of the joint. Starting with 1 of the base joint to 5 of the end-effector.

After that corresponding topics will appear:
```
/snake/Joint1PositionCommand

/snake/Joint2PositionCommand

/snake/Joint3PositionCommand

/snake/Joint4PositionCommand

/snake/Joint5PositionCommand
```
## MoveIt
It is also possible to deploy MoveIt to control the robot in Cartesian space or do trajectory planning and collision avoidance. 

The neccessary package for MoveIt communication has been created: **moveit_snake**

In order to control the robot with MoveIt, first the joint trajectory controller should be spawned.

```
roslaunch dynamixel_control trajectory_control.launch
```

After that:
```
roslaunch moveit_arm moveit_planning_execution.launch
```

Now, you can use MoveIt to control the robot in the real world.
