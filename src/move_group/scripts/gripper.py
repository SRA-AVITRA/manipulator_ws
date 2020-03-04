#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float64
from dynamixel_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list, list_to_pose
import tf
import numpy as np

max_load = 0.125 #0.0624
current_load = 0.0
open_angle = -0.015 #-2.0
close_angle = -1.4
angle = 0

# def State_cb(data):
#     current_load = data.load

# rospy.Subscriber("/gripper_controller/state", JointState, State_cb)
pub = rospy.Publisher("/gripper_controller/command", Float64, queue_size=10)
rospy.init_node('gripper', anonymous=True)


def gripper_close():
	global angle
	global close_angle
	# gripper_open()
	# while((current_load < max_load) & (angle >= -0.84)):
	while(angle >= close_angle):
		pub.publish(angle)
		angle -= 0.009

def gripper_open():
	global open_angle
    	pub.publish(open_angle)


if __name__ == '__main__':
	if (len(sys.argv) == 2):
		if (sys.argv[1] == "open"):
	        	gripper_open()
		elif(sys.argv[1] == "close"):
	        	gripper_close()
	    	else: 
			"Supported Arguments : open or close"
	else: 
		gripper_open()
  
