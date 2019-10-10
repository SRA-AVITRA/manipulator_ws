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

roll = 1.57
pitch = 1.57
yaw = 1.57

quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

home = [0.0, 0.0, 0.4, quat[0], quat[1],quat[2],quat[3]]

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
    if (sys.argv[1] == "home"):
        change_end_pose(group_arm, home)
    elif(sys.argv[1] == "pre_grasp_f"):
       change_joint_angles(group_arm, [0, 0, 0, 0, 0, 0])
    elif(sys.argv[1] == "pre_grasp_i"):
        change_end_pose(group_arm, pre_grasp)
    else:
        "Supported Arguments : home"
else:        
    change_end_pose(group_arm, end_effector_coordinate)

