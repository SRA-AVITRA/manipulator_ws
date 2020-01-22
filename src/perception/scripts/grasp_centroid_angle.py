#!/usr/bin/env python

import rospy
from std_msgs.msg import String, UInt32
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
from perception.msg import array,array_float
import math
import cv2

t_x,t_y,t_z = 0,0,0
counter = 0 

def on_new_point_cloud(data):
    print("here")
    global array
    array = ros_numpy.point_cloud2.pointcloud2_to_array(data,squeeze = True)
# 278 397 9.0
# 388 443 90.0
    x = array[443, 388][0]
    y = -array[443, 388][1]
    z = array[443, 388][2]
    theta = 90
   
    arr = [x,y,z,theta]
    print(arr)        
    pub.publish(arr)



def listener():
    rospy.init_node('3d_position', anonymous=True)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, on_new_point_cloud)

if __name__ == '__main__':
   listener()
   pub = rospy.Publisher("position", array_float, queue_size = 10)
   rospy.spin()
