#!/usr/bin/env python

import rospy
from std_msgs.msg import String, UInt32
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
from perception.msg import array,array_float
import math
import cv2
from geometry_msgs.msg import PointStamped
import tf
import tf2_ros

t_x,t_y,t_z = 0,0,0
counter = 0 

obj = PointStamped()
obj.header.frame_id = "camera_link"
obj.header.stamp =rospy.Time(0)


def on_new_point_cloud(data):
    global array
    array = ros_numpy.point_cloud2.pointcloud2_to_array(data,squeeze = True)

def on_new_centroid_array(data):
    global t_x,t_y,t_z,counter
    global array
    centroid_array = data.array
    try:
        # x = array[centroid_array[1],centroid_array[0]][0]
        # y = array[centroid_array[1],centroid_array[0]][1] 
        # z = array[centroid_array[1],centroid_array[0]][2]



        #Robot axis reconfiguration
        x = array[centroid_array[1],centroid_array[0]][0]
        y = -array[centroid_array[1],centroid_array[0]][1]
        z = array[centroid_array[1],centroid_array[0]][2]

        if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
            t_x +=x
            t_y +=y
            t_z +=z
            counter+=1
        if counter==10:
            t_x/=10
            t_y/=10
            t_z/=10
            arr = [t_x,t_y,t_z]
            counter = 0
            obj.point.x = t_z
            obj.point.y = -t_x
            obj.point.z = t_y
            transformed_obj = listener.transformPoint("bot", obj)    
            pub.publish(arr)
            point_pub.publish(transformed_obj)
            print("XYZ:",arr)
        if cv2.waitKey(10) == ord('x'):
            cv2.destroyAllWindows()
            rospy.shutdown()
            sys.exit()
        
    except TypeError,ValueError:
        pass

def listener():
    rospy.init_node('3d_position', anonymous=True)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, on_new_point_cloud)
    rospy.Subscriber("/centroid", array, on_new_centroid_array)

if __name__ == '__main__':
   listener()
   pub = rospy.Publisher("position", array_float, queue_size = 10)
   point_pub = rospy.Publisher("object", PointStamped, queue_size = 1)
   cam_broadcaster = tf.TransformBroadcaster()
   listener = tf.TransformListener()
   # rospy.spin()
   while not rospy.is_shutdown():
        try:
            cam_broadcaster.sendTransform((0.2, 0, 0.125), (0, 0, 0, 1), rospy.Time.now(), "camera_link", "bot")
        except:
            break
