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
# What we do during shutdown


rospy.loginfo("Waiting for image topics...")

def image_callback(ros_image):
	bridge = CvBridge()
	try:
		frame = bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")

	except CvBridgeError, e:
		print e


	cv2.imshow("rgb",frame)

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
	# pc2_pub = rospy.Publisher('object_points_pc2',PointCloud2,queue_size = 10)
	main(sys.argv)
