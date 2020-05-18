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

#RPY of end-effector
roll, pitch, yaw = 0, 0, 0
quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw) #RPY to quaternion conversion

#predefined poses
place = [0.517352231531, 0.333934163634, 0.482077397268, -0.0217592563181, 0.026508841935, 0.374750453666, 0.926491183736]

#Initializing moveit functions
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('inverse_kinematics',anonymous=True)
robot = moveit_commander.RobotCommander()           #interface b/w move_group node and the robot
scene = moveit_commander.PlanningSceneInterface()   #interface for the world around the robot

group_name = "arm"  #Planning group name set in Moveit setup assistant
group = moveit_commander.MoveGroupCommander(group_name)

group.set_goal_tolerance(0.0005)                    #Tolerence for end-effector position at the goal 
group.set_goal_orientation_tolerance(0.0001)    #Tolerence for end-effector orientation at the goal
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20) #publish trajectories for RViz to visualize


def change_joint_angles(group, joint_goal):     #forward kinematics - input: joint angles [0, 0, 0, 0, 0, 0] 
    joints = group.get_current_joint_values()
    for i in range(len(joint_goal)):
        joints[i] = joint_goal[i]
    group.go(joints, wait=True)       #motion execution
    group.stop()
    print "\nJoint angles moved to ", joints

def change_end_pose(group, input_pose):         #inverse kinematics - input: pose array [0, 0, 0, 0, 0, 0, 0]
    pose_goal = geometry_msgs.msg.Pose()
    group.set_pose_target(input_pose)
    
    if group.go(wait=True):                     #motion execution
        print "Solution found for ", input_pose
        print "Group moved to ", input_pose
    else:
        print "No solution found for", input_pose
    group.stop()
    group.clear_pose_targets()

if (len(sys.argv) == 2):
    if(sys.argv[1] == "home"):
      change_joint_angles(group, [0, 0, 0, 0, 0, 0])
    elif(sys.argv[1] == "place"):
      change_end_pose(group, place)
    else:
        "Supported Arguments : home, place"
else:        
    change_joint_angles(group, [0, 0, 0, 0, 0, 0])

