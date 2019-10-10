#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pose_goal',anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)



roll = 1.57
pitch = 1.57
yaw = 1.57

quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
print quat
if (len(sys.argv) == 2):
    if(sys.argv[1]== "home"): 
        joint_goal =group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0
        
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()




pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = quat[3]
pose_goal.orientation.x = quat[0]
pose_goal.orientation.y = quat[1]
pose_goal.orientation.z = quat[2]

pose_goal.position.x = 0
pose_goal.position.y = 0
pose_goal.position.z = 0
group.set_pose_target(pose_goal)

plan = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()
print "ROBOT STATE :"
print robot.get_current_state()
