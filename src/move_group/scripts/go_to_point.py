#!/usr/bin/env python

import os

# os.system('rosrun move_group grasp_pose_sub.py && rosrun move_group cartesian.py && rosrun move_group gripper.py close')
os.system('rosrun move_group pose_sub.py && rosrun move_group cartesian.py && rosrun move_group gripper.py close')