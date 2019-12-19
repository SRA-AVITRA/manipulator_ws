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

max_load = 0.001
current_load = 0.0


def State_cb(data):
    current_load = data.load

rospy.init_node('gripper', anonymous=True)
rospy.Subscriber("/gripper_controller/state", JointState, State_cb)
pub = rospy.Publisher("/gripper_controller/command", Float64, queue_size=10)


def gripper_close():
    angle = -0.1
    while(1):                          #(current_load < max_load):
        pub.publish(angle)
        angle -= 0.1


def gripper_open():
    for i in range(5):
        pub.publish(0.1)


gripper_open();
# if (len(sys.argv) == 2):
#     if (sys.argv[1] == "open"):
#         gripper_open()
#     elif(sys.argv[1] == "close"):
#         gripper_close()
#     else:
#         "Supported Arguments : open or close"



