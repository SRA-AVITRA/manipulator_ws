#!/usr/bin/env python

import rospy 
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 

class cvBridgeDemo(object):

	def __init__(self):

		self.lower_red = np.array([0,0,0])
		self.upper_red = np.array([25,255,255])

		self.node_name = "cv_bridge_demo"
		self.kernel = np.ones((2,2),np.uint8)
		#Initialize the ros node
		rospy.init_node(self.node_name)
		# What we do during shutdown
		rospy.on_shutdown(self.cleanup)

		self.bridge = CvBridge()
		# Subscribe to the camera image and depth topics and set
		# the appropriate callbacks
		self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image,self.image_callback)
		rospy.loginfo("Waiting for image topics...")
		# self.lower_red = np.array([0,50,50])
  #   	self.upper_red = np.array([10,255,255])

	def image_callback(self, ros_image):

# def maxContour(cnts):
# 		    idc = 0
# 		    max_area = 0
# 		    counter = 0
# 		    for n in cnts:
# 		        a = cv2.contourArea(n)
# 		        if a>max_area:
# 		            max_area = a
# 		            idc = counter
# 		        counter+=1

# 			return idc,max_area
		# Use cv_bridge() to convert the ROS image to OpenCV format
		try:
			frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
		except CvBridgeError, e:
			print e
		# Convert the image to a Numpy array since most cv2 functions
		# require Numpy arrays.
		# frame = np.array(frame, dtype=np.uint8)
		# Process the frame using the process_image() function
		# display_image = self.process_image(frame)
		# Display the image.
		# bboxRed = cv2.selectROI(frame)
		# redObj = frame[int(bboxRed[1]):int(bboxRed[1]+bboxRed[3]), int(bboxRed[0]):int(bboxRed[0]+bboxRed[2])]
		
		# hR, sR, vR = np.median(redObj[:,:,0]), np.median(redObj[:,:,1]), np.median(redObj[:,:,2])
		# self.lower_red = np.array([hR-5, 0, vR-70])
		# self.upper_red = np.array([hR+5, sR+50, vR+70])

		# print('lower: ',self.lower_red)
		# print('higher: ',self.upper_red)
		
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		hsv = cv2.bilateralFilter(frame,9,75,75)
		mask_red = cv2.inRange(hsv, self.lower_red, self.upper_red)

		mask_red = cv2.dilate(mask_red,self.kernel,iterations=1)
		# mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, self.kernel)

		cv2.imshow("red",mask_red)
		_, contoursRed, hR = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		
		idx,current_max,counter = 0, 0, 0	
		for n in contoursRed:
			a = cv2.contourArea(n)
			if a>current_max:
				current_max=a
				idx = contoursRed.index(n)


		redIndex, redValue = idx, current_max
		x,y,w,h = cv2.boundingRect(contoursRed[redIndex])
		# centroid = (x + (x+w))/2
		img = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
		# redIndex, redValue = maxContour(contoursRed)
		print(redValue)
		cv2.drawContours(frame, contoursRed, redIndex, (0, 0, 255), 2)
		# cv2.rectangle(frame,(x,y,w,h))
		cv2.imshow(self.node_name, frame)
		# Process any keyboard commands
		self.keystroke = cv2.waitKey(5)
		if cv2.waitKey(1)  == 27:
			rospy.shutdown()

	# def depth_callback(self, ros_image):
	# 	# Use cv_bridge() to convert the ROS image to OpenCV format
	# 	try:
	# 	# The depth image is a single-channel float32 image
	# 		depth_image = self.bridge.imgmsg_to_cv2(ros_image, "32FC1")
	# 	except CvBridgeError, e:
	# 		print e
	# 	# Convert the depth image to a Numpy array since most cv2 functions
	# 	# require Numpy arrays.
	# 	depth_array = np.array(depth_image, dtype=np.float32)
	# 	# Normalize the depth image to fall between 0 (black) and 1 (white)
	# 	cv2.normalize(depth_array,
	# 	depth_array, 0, 1, cv2.NORM_MINMAX)
	# 	# Process the depth image
	# 	depth_display_image = self.process_depth_image(depth_array)
	# 	# Display the result
	# 	cv2.imshow("Depth Image", depth_display_image)
	# def maxContour(cnts):
	#     idc = 0
	#     max_area = 0
	#     counter = 0
	#     for n in cnts:
	#         a = cv2.contourArea(n)
	#         if a>max_area:
	#             max_area = a
	#             idc = counter
	#         counter+=1

	# 	return idc,max_area

	def process_image(self, frame):
		# Convert to grayscale
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask_red = cv2.inRange(hsv, self.lower_red, self.upper_red)

		_, contoursRed, hR = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		
		return contoursRed,hR

	# def process_depth_image(self, frame):
	# 	# Just return the raw image for this demo
	# 	return frame

	def cleanup(self):
		print "Shutting down vision node."
		cv2.destroyAllWindows()


def main(args):
	try:
		cvBridgeDemo()
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down vision node."
		cv2.DestroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)