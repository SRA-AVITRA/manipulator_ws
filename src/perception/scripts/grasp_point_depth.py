#!/usr/bin/env python

import rospy
from std_msgs.msg import String, UInt32
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
from perception.msg import array,array_float

# cx = input('x: ')
# cy = input('y: ')
# 207 262

cx = 251
cy = 416

def on_new_point_cloud(data):
    global cx, cy
    array = ros_numpy.point_cloud2.pointcloud2_to_array(data,squeeze = True)

    x = array[cy,cx][0]
    y = array[cy, cx][1]
    z = array[cy,cx][2]

    arr = [x, y, z]

    print(arr)

    pub.publish(arr)

    



def listener():
    rospy.init_node('position_3d', anonymous=True)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, on_new_point_cloud)

if __name__ == '__main__':
   listener()
   pub = rospy.Publisher("position", array_float, queue_size = 10)
   rospy.spin()
