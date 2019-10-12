#!/usr/bin/env python

import rospy 
import sys
import cv2
from depth_calc.msg import array
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
		self.centroid_pub = rospy.Publisher('centroid', array, queue_size = 10)
		rospy.loginfo("Waiting for image topics...")
		# self.lower_red = np.array([0,50,50])
  #   	self.upper_red = np.array([10,255,255])

	def image_callback(self, ros_image):
		try:
			frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
		except CvBridgeError, e:
			print e
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
				idx = counter
			counter+=1


		redIndex, redValue = idx, current_max
		x,y,w,h = cv2.boundingRect(contoursRed[redIndex])
		arr = [x,y,w,h]
		print(arr)
        self.centroid_pub.publish(arr)
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
