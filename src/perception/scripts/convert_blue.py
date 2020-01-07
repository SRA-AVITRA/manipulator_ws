import cv2
import numpy as np

rgb = cv2.imread('color.png')
depth = cv2.imread('depth.png',0)
rgb[:,:,2] = depth
cv2.imwrite('bgd.png',rgb)