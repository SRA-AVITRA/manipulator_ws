#!/usr/bin/env python

import rospy 
import sys
import cv2
from perception.msg import array
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import ros_numpy


node_name = "pixel_centroid"

#Initialize the ros node
rospy.init_node(node_name)

rospy.loginfo("Waiting for image topics...")



def image_callback(ros_image):

	# Bridge object for converting between ROS Topic and OpenCV object
	bridge = CvBridge()
	kernel = np.ones((2,2),np.uint8)

	# Select for PINK colour
#	lower = np.array([ 155.,   40.,  100.])
#	upper = np.array([ 170.,  250.,  250.])
	
	# Select for RED colour 
	lower = np.array([ 165.,   100.,  100.])
	upper = np.array([ 185., 220., 210.])

	try:
		frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
	except CvBridgeError, e:
		print e


	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)	# Convert Colourspace from BGR to HSV
	mask = cv2.inRange(hsv, lower, upper)			# Create a mask between the specified range
	mask = cv2.bilateralFilter(mask,9,75,75)		# Apply a bilateral filter for image filtering without loss of corner information
	mask = cv2.dilate(mask,kernel,iterations=1)		# dilate expands the necessary pixels

	_, contours, hR = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)	# Extract contours from Mask 
	
	idx,current_max,counter = 0, 0, 0		# Find the contour with maximum area
	for n in contours:
		a = cv2.contourArea(n)
		if a>current_max:
			current_max=a
			idx = counter
		counter+=1


	
	
	cv2.drawContours(frame, contours, idx, (0, 0, 255), 2)	#Visulization
	
	#For contour based centroid

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

	#For bounding-box based centroid
	x,y,w,h = cv2.boundingRect(contours[idx])
	c_x = (x + w/2)
	c_y = (y + h/2)
	arr = [c_x,c_y]

	img = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)	# Draw a rectangle along the bounding box for visualization
	cv2.imshow(node_name, frame)	# Display the node 
	centroid_pub.publish(arr)		# Publish the centroid for further processing 
	
	if cv2.waitKey(10) == ord('x'):
		rospy.shutdown()
		cv2.DestroyAllWindows()
		sys.exit()


def main(args):
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down vision node."
		cv2.DestroyAllWindows()
		rospy.shutdown()

if __name__ == '__main__':
	image_sub = rospy.Subscriber("/camera/color/image_rect_color", Image,image_callback)
	centroid_pub = rospy.Publisher('centroid', array, queue_size = 10)
	main(sys.argv)
