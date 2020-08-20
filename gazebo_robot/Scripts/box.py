#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
rospy.init_node('box')
pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=5)


quaternion = tf.transformations.quaternion_from_euler(0.000338, 0.003608, 0.000875)
msg = ModelState()
msg.model_name = 'robot1'
msg.pose.position.x = -2.873680
msg.pose.position.y = -0.128694
msg.pose.position.z = 0.012142
msg.pose.orientation.x = quaternion[0]
msg.pose.orientation.y = quaternion[1]
msg.pose.orientation.z = quaternion[2]
msg.pose.orientation.w = quaternion[3]
msg.reference_frame = 'world'
rate = rospy.Rate(50)

try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(msg)

except rospy.ServiceException, e:
        print "Service call failed: %s" % e
