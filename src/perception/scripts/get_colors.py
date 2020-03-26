#!/usr/bin/env python

import rospy 
import sys
import cv2
from perception.msg import array
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 


lower_red = np.array([160,0,0])
upper_red = np.array([180,255,255])

node_name = "cv_bridge_demo"
kernel = np.ones((2,2),np.uint8)
#Initialize the ros node
rospy.init_node(node_name)
# What we do during shutdown
bridge = CvBridge()


rospy.loginfo("Waiting for image topics...")

def image_callback(ros_image):
    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError, e:
        print e
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    bboxRed = cv2.selectROI(frame)
    redObj = hsv[int(bboxRed[1]):int(bboxRed[1]+bboxRed[3]), int(bboxRed[0]):int(bboxRed[0]+bboxRed[2])]
    print()

    hR, sR, vR = np.median(redObj[:,:,0]), np.median(redObj[:,:,1]), np.median(redObj[:,:,2])
    # lower_rangeRed = np.array([hR-5, 0, vR-70])
    # higher_rangeRed = np.array([hR+5, sR+50, vR+70])

    # print('lower: ',lower_rangeRed)
    # print('higher: ',higher_rangeRed)
    print('[',hR,sR,vR,']')
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

    main(sys.argv)
