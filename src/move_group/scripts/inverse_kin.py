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
    # print  "\nOUT: ", input_pose, end_goal
    group.set_pose_target(list_to_pose(end_goal))
    if group.go(wait=True):
    # if group.plan():
        print "\n \033[0;32;40m Solution found for ", input_pose
        print "\nGroup moved to ", end_goal, "\033[0;37;40m"
    else:
        print "\n \033[0;31;40m No solution found for ******************************************************** ", input_pose, "\033[0;37;40m"
    group.stop()
    group.clear_pose_targets()

home = [0.00014979, 0.16832, 0.76241, 0.99997, 0.003044, -0.0008173, -0.0069079]
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
print "============ \nReference frame: %s" % planning_frame
eef_link = group_arm.get_end_effector_link()
print "End effector: %s" % eef_link
group_names = robot.get_group_names()
print "Goal_tolerance\n", group_arm.get_goal_tolerance()


print "Group_arm pose\n", group_arm.get_current_pose()
print "Joint Angles\n", group_arm.get_current_joint_values()

# end_effector_coordinate = [0.0039, 0.472, 0.1178]
# end_effector_coordinate = [-0.0022319, -0.054486, 0.35541]
end_effector_coordinate = [0.0039984, 0.47637, 0.36712]

# end_effector_coordinate = [0.1, 0.197971252863, 0.157264105097]
# end_effector_orientatation = [None, None, None] #in euler
# end_effector_orientatation = [None, None, None] #in euler
# quaternion = tf.transformations.quaternion_from_euler(end_effector_orientatation[0],end_effector_orientatation[1],end_effector_orientatation[2])
# quaternion = [0.00072718, -0.015007, 0.99988, 0.0045895]
quaternion = [0.0028514, -0.00081611, -0.0069344, 0.99997]

# quaternion = [-0.015515, -0.00066536, -0.0084234, 0.99984]
end_effector_coordinate.extend(quaternion)

print "quat:", quaternion
print "array:", end_effector_coordinate

if (len(sys.argv) == 2):
    if (sys.argv[1] == "home"):
        change_end_pose(group_arm, home)
    elif(sys.argv[1] == "pre_grasp_f"):
       change_joint_angles(group_arm, [0, 0, 1.57, 1.57, 0, 1.57])
    elif(sys.argv[1] == "pre_grasp_i"):
        change_end_pose(group_arm, pre_grasp)
    else:
        "Supported Arguments : home"
else:        
    change_end_pose(group_arm, end_effector_coordinate)

print "\nGroup_arm pose\n", group_arm.get_current_pose()
print "Joint Angles\n", group_arm.get_current_joint_values()
