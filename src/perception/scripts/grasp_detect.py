#!/usr/bin/env python

import rospy 
import sys
import cv2
from perception.msg import array
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import ros_numpy
import message_filters
import matplotlib.pyplot as plt
# class Server:
#     def __init__(self):
#         self.rgb = None
#         self.depth = None

#     def rgb_callback(self, ros_image):
#         bridge = CvBridge()
#         try:
# 		frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
# 	except CvBridgeError, e:
# 		print e
#         self.rgb = frame
        

#         # Compute stuff.
#         self.compute_stuff()

#     def depth_callback(self, ros_image):
#         # "Store" the message received.
#         bridge = CvBridge()
# 	try:
# 		frame = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
# 	except CvBridgeError, e:
#     		print e 
#         self.depth = frame
        

#         # Compute stuff.
#         self.compute_stuff()

#     def compute_stuff(self):
#         if self.rgb is not None and self.depth is not None:
            
#             cv2.imshow('rgb',self.rgb)
#             self.rgb[:,:,2] = self.depth
#             print(type(self.rgb))

i = 1 
def image_callback(ros_rgb, ros_depth):
    a = input('Press enter to continue')
    global i
    bridge = CvBridge()
    try:
        rgb = bridge.imgmsg_to_cv2(ros_rgb, "bgr8")
        depth = bridge.imgmsg_to_cv2(ros_depth,"passthrough")
    except CvBridgeError, e:
        print e


    kernel = np.ones((2,2),np.uint8)

    cv2.imwrite('/home/shambhavi/manipulator_ws/src/perception/scripts/color.png',rgb)
    cv2.imwrite('/home/shambhavi/manipulator_ws/src/perception/scripts/depth.png',depth)
    color = cv2.imread('/home/shambhavi/manipulator_ws/src/perception/scripts/color.png')
    depth = cv2.imread('/home/shambhavi/manipulator_ws/src/perception/scripts/depth.png',0)
    color[:,:,0] = depth
    print("writing")
    cv2.imwrite('/home/shambhavi/manipulator_ws/src/perception/scripts/data/box/rgd'+str(a)+'.png',color)

if __name__ == '__main__':
    print("writing")
    rospy.init_node('rgb')

    rgb_sub = message_filters.Subscriber('/camera/color/image_rect_color', Image)
    depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)

    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(image_callback)
    rospy.spin()

