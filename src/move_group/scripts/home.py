#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list, list_to_pose
import tf

def change_joint_angles(group, joint_goal):
    joints = group.get_current_joint_values()
    for i in range(len(joint_goal)):
        joints[i] = joint_goal[i]
    group.go(joints, wait=True)
    group.stop()
    print "\nJoint angles moved ", joints

#Initializing moveit functions
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('home',anonymous=True)
robot = moveit_commander.RobotCommander()           #interface b/w move_group node and the robot
scene = moveit_commander.PlanningSceneInterface()   #interface for the world around the robot

group_name = "arm"  #Planning group name set in Moveit setup assistant
group = moveit_commander.MoveGroupCommander(group_name)

group.set_goal_tolerance(0.0005)                    #Tolerence for end-effector position at the goal 
group.set_goal_orientation_tolerance(0.0001)    #Tolerence for end-effector orientation at the goal
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20) #publish trajectories for RViz to visualize

change_joint_angles(group, [0, 0, 0, 0, 0, 0])

