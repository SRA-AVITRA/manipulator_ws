#!/usr/bin/env python

#################################################################
'''
velocity is published to /teleop and published to /ticks 
'''
#################################################################

import struct
import rospy
from sensor_msgs.msg import JointState

def callback(msg):
    position_array = msg.position
    print(position_array)
    # while not rospy.is_shutdown():
    #     try:
    #         pub.publish(velocity)
    #         rate.sleep()
    #     except KeyboardInterrupt:
    #         break

if __name__=="__main__":
    rospy.init_node('rosserial',anonymous=False)
    rospy.Subscriber("/move_group/fake_controller_joint_states", JointState, callback)
    rospy.spin()
    # pub = rospy.Publisher('angles', velocity_msg, queue_size=10)
    # rate = rospy.Rate(10)   #10Hz
    # velocity.motor_L = 0
    # velocity.motor_R = 0
    