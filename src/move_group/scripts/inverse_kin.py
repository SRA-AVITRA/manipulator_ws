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
    print "\nJoint angles moved to ", joints

def change_end_pose(group, input_pose):
    pose_goal = geometry_msgs.msg.Pose()
    end_goal = pose_to_list(pose_goal)
    
    for i, ele in enumerate(input_pose):
        if ele != None:
            end_goal[i] = ele
    group.set_pose_target(list_to_pose(end_goal))
    if group.go(wait=True):
        print "Solution found for ", input_pose
        print "Group moved to ", end_goal
    else:
        print "No solution found for", input_pose
    group.stop()
    group.clear_pose_targets()

roll = 0 
pitch = 0
yaw = 0
# roll = -0.974681390288264  
# pitch = -0.4256494427892773
# yaw = 1.5285175517752887


quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

home = [0.3, 0.3, 0.4, quat[0], quat[1],quat[2],quat[3]]

post_grasp = [-0.14, -0.24, 0.53, quat[0], quat[1],quat[2],quat[3]  ]

test = [0.6, 0.17, 0.52, quat[0], quat[1],quat[2],quat[3]]

zdownby1_5 = [0.114535965537,-0.000209275828715,0.516932398762,-0.530234925441,-0.54221513755,0.458969531642,0.462818147308]

place = [0.517352231531, 0.333934163634, 0.482077397268, -0.0217592563181, 0.026508841935, 0.374750453666, 0.926491183736]
  # position: 
  #   x: 0.517352231531
  #   y: 0.333934163634
  #   z: 0.482077397268
  # orientation: 
  #   x: -0.0217592563181
  #   y: 0.026508841935
  #   z: 0.374750453666
  #   w: 0.926491183736 

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('inverse_kinematics', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_arm = moveit_commander.MoveGroupCommander("arm")
group_arm.set_planner_id("TRRT")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=20)

planning_frame = group_arm.get_planning_frame()
group_arm.set_goal_tolerance(0.0001)
group_arm.set_goal_orientation_tolerance(0.0001)

eef_link = group_arm.get_end_effector_link()
group_names = robot.get_group_names()

if (len(sys.argv) == 2):
    if (sys.argv[1] == "ztest"):
        change_end_pose(group_arm, zdownby1_5)
    elif(sys.argv[1] == "home"):
       change_joint_angles(group_arm, [0, 0, 0, 0, 0, 0])
    elif(sys.argv[1] == "test"):
        change_end_pose(group_arm, test)
    elif(sys.argv[1] == "post_grasp"):
        change_end_pose(group_arm, post_grasp)
    else:
        "Supported Arguments : home"
else:        
    change_end_pose(group_arm, home)

