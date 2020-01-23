#!/usr/bin/env python

import rospy
from std_msgs.msg import String, UInt32
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
from perception.msg import array,array_float
import math
import cv2
import os

t_x,t_y,t_z = 0,0,0
counter = 0 

f = open(os.getenv("HOME")+'/manipulator_ws/temp/points.txt','r')
s = f.read()
cx,_,s = s.partition("\n")
cy,_,s = s.partition("\n")
ctheta,_,s = s.partition("\n")
f.close()
def on_new_point_cloud(data):
    global array, cx, cy, ctheta
    cx, cy, ctheta = int(cx), int(cy), float(ctheta)

    array = ros_numpy.point_cloud2.pointcloud2_to_array(data,squeeze = True)
# 278 397 9.0
# 388 443 90.0
    x = array[cy, cx][0]
    y = -array[cy, cx][1]
    z = array[cy, cx][2]
    theta = ctheta
   
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
