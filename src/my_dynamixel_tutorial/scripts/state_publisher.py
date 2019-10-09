#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList
from math import pi
import time

rospy.init_node('State_Publisher')

# Name of the joints in the robotic arm
# Name of motor ID [1,2,3,4,5,6] is specified in the list.
joints = ["shoulder_yaw", "shoulder_pitch", "elbow", "wrist_pitch", "wrist_yaw", "wrist_roll", "gripper_motor"]

#Dynamixel Motors will not be in Position 0 when switched ON. Dynamixel motors will have some initial position value. 
#In the URDF it is assumed that initial joint value is 0 radian. 
#Hence offset is required to match the URDF and the real robot.
#Offset for motor id [1,2,3,4,5,6] is specified in the list.
# offset = [666,512,512,375,512,682]
# offset = [185, 3000, 2222, 512, 202]
offset = [2048, 2048, 2048, 512, 512, 512, 530]

'''
Function: process(). Callback for subscriber of raw data from dynamixel motor. 
Logic: Dynamixel position = 0 for 0 degree and Dynamixel position = 1023 for 300 degree.
       Current position can be calculated by (position*(300.0/1023))*(pi/180) radian.
       Where position = feddback-offset.
'''
def process(msg):
	joint_states = JointState()

	joint_states.header.stamp = rospy.Time.now()

	for x in msg.motor_states:
		if (x.id < 7) :
			joint_states.name.append(joints[x.id-1])

			if((x.id < 4)):
				joint_states.position.append((x.position - offset[x.id-1])*(360.0/4096)*(pi/180))
			else:
				joint_states.position.append((x.position - offset[x.id-1])*(300.0/1023)*(pi/180))
		
			#joint_states.velocity.append(x.velocity)
	
	pub.publish(joint_states)

# Subscriber for raw feedback from dynamixel motor. Position of the motor will be in the range of (0,1023).
sub = rospy.Subscriber('/motor_states/pan_tilt_port',MotorStateList,process)
# Publisher for the current position of dynamixel motor in radian
pub = rospy.Publisher('/robot/joint_states',JointState,queue_size=10)

rospy.spin()