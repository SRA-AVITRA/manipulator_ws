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

bot_x = 0.18
bot_y = 0
bot_z = 0.28
off_y = 0.46
off_z = 0.055

roll = 3.0329013541915026 #0
pitch = 1.5045738202377112 #0
yaw = 3.044896045079767 #1.57
roll, pitch, yaw = -0.010236535202312786, 0.023526037055784085, 0.006135923151542565
quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
def transform(x,y,z) :
    out = [0,0,0]
    sum_x, sum_y, sum_z = 0,0,0
    out_x = round((z + bot_x)-0.22, 2)
    out_y = round(-(x - off_y ), 2) 
    out_z = round((-(y - off_z) + bot_z), 2) 

    for i in range(100):  
        sum_x += out_x
        sum_y += out_y
        sum_z += out_z

    

    return sum_x/100, sum_y/100, sum_z/100

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


else:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = quat[3]
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    # [-0.033141941, 0.0059073088, 0.185]
    pose_goal.position.x, pose_goal.position.y, pose_goal.position.z =  transform(0, 0.11, 0.39)
   # [-0.042422794, 0.11115891, 0.39400002]
#transform(0.346233978271, 0.0664755020142, 0.44) #transform(data.array[0], data.array[1], data.array[2])
    # pose_goal.position.x = transform(34.6233978271 6.64755020142, 32)
    # pose_goal.position.y = 
    # pose_goal.position.z = 

    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    if(plan):
        print "MOVED"
    else:
        print "FAILED"

    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

