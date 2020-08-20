#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

rospy.init_node('angles')
pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=5)
pub1 = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=5)
pub2 = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=5)
pub3 = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=5)
pub4 = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=5)
msg = [1.7564973394647119, -0.8303822610535567, -1.670635381665238, -0.8979926220318672, 1.642187184591668]

rate = rospy.Rate(50)
while not False:
	pub.publish(Float64(msg[4]))
	pub1.publish(Float64(msg[1]))
	pub2.publish(Float64(msg[2]))
	pub3.publish(Float64(msg[3]))
	pub4.publish(Float64(msg[0]))
	angles = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)

	if (abs(angles.position[4] - msg[4]) < 0.01 and abs(angles.position[1] - msg[1]) < 0.01 and \
	    abs(angles.position[2] - msg[2]) < 0.01 and abs(angles.position[3] - msg[3]) < 0.01 and abs(angles.position[0] - msg[0]) < 0.01):
		break
	rate.sleep()
