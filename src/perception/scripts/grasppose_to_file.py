#!/usr/bin/env python

import os
os.chdir(os.getenv("HOME")+'/manipulator_ws/grasp_multiObject_multiGrasp')
os.system('rosrun perception get_image.py && ./tools/demo_graspRGD.py --net res50 --dataset grasp --path rgd0.png && rosrun perception grasp_centroid_angle.py')