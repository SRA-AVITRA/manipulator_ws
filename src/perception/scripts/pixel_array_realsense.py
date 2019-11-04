#!/usr/bin/env python

import rospy 
import sys
import cv2
from perception.msg import array
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import ros_numpy


node_name = "cv_bridge_demo"

#Initialize the ros node
rospy.init_node(node_name)
# What we do during shutdown


rospy.loginfo("Waiting for image topics...")

def image_callback(ros_image):
	bridge = CvBridge()
	kernel = np.ones((2,2),np.uint8)
	lower_red = np.array([ 160,   100.,  100.])
	upper_red = np.array([ 180.,  220.,  210.])

	try:
		frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
	except CvBridgeError, e:
		print e
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	mask_red = cv2.inRange(hsv, lower_red, upper_red)
	mask_red = cv2.bilateralFilter(mask_red,9,75,75)
	mask_red = cv2.dilate(mask_red,kernel,iterations=1)
	# mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

	cv2.imshow("red",mask_red)
	_, contoursRed, hR = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	idx,current_max,counter = 0, 0, 0	
	for n in contoursRed:
		a = cv2.contourArea(n)
		if a>current_max:
			current_max=a
			idx = counter
		counter+=1


	redIndex, redValue = idx, current_max
	
	#Visulization
	cv2.drawContours(frame, contoursRed, redIndex, (0, 0, 255), 2)
	

	#Get contour array
	# cimg = np.zeros_like(frame)
	# cv2.drawContours(cimg, contoursRed, redIndex,(0,0,255),2)
	# pts = np.where(cimg==255)
	# # pts_pc2 = ros_numpy.point_cloud2.array_to_pointcloud2(pts)

	# pts = np.array(pts)
	# print(pts.shape)
	# c_row = np.mean(pts[:,0])
	# c_col = np.mean(pts[:,1])
	# arr = [c_row, c_col]
	x,y,w,h = cv2.boundingRect(contoursRed[redIndex])
	c_x = (x + w/2)
	c_y = (y + h/2)
	arr = [c_x,c_y]
	print(arr)
	centroid_pub.publish(arr)
	img = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
	cv2.imshow(node_name, frame)
	# pc2_pub.publish(pts_pc2)
	centroid_pub.publish(arr)
	keystroke = cv2.waitKey(5)
	if keystroke  == 27:
		rospy.shutdown()
		cv2.DestroyAllWindows()


def main(args):
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down vision node."
		cv2.DestroyAllWindows()

if __name__ == '__main__':
	image_sub = rospy.Subscriber("/camera/color/image_rect_color", Image,image_callback)
	centroid_pub = rospy.Publisher('centroid', array, queue_size = 10)
	# pc2_pub = rospy.Publisher('object_points_pc2',PointCloud2,queue_size = 10)

	main(sys.argv)
