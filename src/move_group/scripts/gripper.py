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

open_angle = -0.015 #-2.0	#angle when gripper is open
close_angle = -1.0			#angle when gripper is open
angle = 0

pub = rospy.Publisher("/gripper_controller/command", Float64, queue_size=10)		#publishes gripper joint angle 
rospy.init_node('gripper', anonymous=True)

def gripper_close():
	global angle
	global close_angle
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
  
