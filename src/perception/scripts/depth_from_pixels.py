#!/usr/bin/env python

import rospy
from std_msgs.msg import String, UInt32
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
from perception.msg import array,array_float
import math
t_x,t_y,t_z = 0,0,0
counter = 0 

def on_new_point_cloud(data):
    global array
    array = ros_numpy.point_cloud2.pointcloud2_to_array(data,squeeze = True)

def on_new_centroid_array(data):
    global t_x,t_y,t_z,counter
    global array
    centroid_array = data.array
    try:
        x = array[centroid_array[1],centroid_array[0]][0]
        y = array[centroid_array[1],centroid_array[0]][1] 
        z = array[centroid_array[1],centroid_array[0]][2]
        

        print("--------------------------------------------")
        print("Centroid:",centroid_array)
        if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
            print('not nan')
            t_x +=x
            t_y +=y
            t_z +=z
            counter+=1
        print(counter)
        if counter==10:
            t_x/=10
            t_y/=10
            t_z/=10
            arr = [t_x,t_y,t_z]
            counter = 0
            
            pub.publish(arr)
            print("XYZ:",arr)

        
    except TypeError,ValueError:
        pass

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, on_new_point_cloud)
    rospy.Subscriber("/centroid", array, on_new_centroid_array)

if __name__ == '__main__':
   listener()
   pub = rospy.Publisher("position", array_float, queue_size = 10)
   rospy.spin()
