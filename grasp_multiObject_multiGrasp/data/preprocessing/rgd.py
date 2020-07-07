import os
import math
import numpy as np
from PIL import Image
import glob
import cv2

path = os.path.join('pcd*.png')
images = glob.glob(path)
images.sort()
depth = True
d_image_path = images[0]
# Rgb replacing the blue channel with depth channel to give rgd
for image in images:
    if depth:
        depth = False
        d_image_path = image
    else:
        depth = True
        d_image = cv2.imread(d_image_path,0)
        rgb_image = cv2.imread(image,1)
        rgb_image[:,:,0] = d_image
        path = image.replace('r.png','rgd.png')
        cv2.imwrite(path,rgb_image)
