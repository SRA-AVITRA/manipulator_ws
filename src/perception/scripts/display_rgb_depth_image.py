#!/usr/bin/env python

import rospy 
import sys
import cv2
from perception.msg import array
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import ros_numpy


node_name = "rgb"

#Initialize the ros node
rospy.init_node(node_name)

rospy.loginfo("Waiting for image topics...")

# def depth_image_callback(ros_image):
	# bridge = CvBridge()
	# kernel = np.ones((2,2),np.uint8)
	# try:
		# frame = bridge.imgmsg_to_cv2(ros_image, "32FC1")
	# except CvBridgeError, e:
		# print e
# 
	# cv2.imshow("depth", frame)



def image_callback(ros_image):
	bridge = CvBridge()
	kernel = np.ones((2,2),np.uint8)
	

	try:
		frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
	except CvBridgeError, e:
		print e

	cv2.imshow(node_name, frame)

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
	# image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,depth_image_callback)
	main(sys.argv)
