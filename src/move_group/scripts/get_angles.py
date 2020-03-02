#!/usr/bin/env python

import struct
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

position_array = Float32MultiArray()

def callback(msg):
	position_array.data = msg.position
	
	# for i in range(len(position_array)):
	# 	x[i] = position_array[i]

	# pub.publish(Float32MultiArray, x)
	# r = rospy.Rate(10) # 10hz
	# while not rospy.is_shutdown():
    		# pub.publish(position_array)
    		# r.sleep()      
	pub.publish(position_array)
	print(position_array.data)





if __name__=='__main__':
	rospy.init_node('rosserial', anonymous=False)
	rospy.Subscriber("/joint_states", JointState, callback)
	pub = rospy.Publisher("angles",Float32MultiArray, queue_size=100)
	rospy.spin()

