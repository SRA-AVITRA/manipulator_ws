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
    # joint_goal = group.get_current_joint_values()
    group.go(joint_goal, wait=True)
    group.stop()
    print "\nJoint angles moved to ", joint_goal

def change_end_pose(group, input_pose):
    end_goal = pose_to_list(group.get_current_pose().pose)
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


quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

home = [0.3, 0.3, 0.3, quat[3], quat[0],quat[1],quat[2]]
pre_grasp = [0.0039984, 0.47637, 0.36712, 0.0028514, -0.00081611, -0.0069344, 0.99997]
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


end_effector_coordinate = [0.0039984, 0.47637, 0.36712]

quaternion = [0.0028514, -0.00081611, -0.0069344, 0.99997]

end_effector_coordinate.extend(quaternion)

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

