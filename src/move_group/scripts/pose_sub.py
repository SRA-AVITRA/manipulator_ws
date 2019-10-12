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
from perception.msg import array_float, array

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

def transform(x,y,z) :
    out = [0,0,0]
    out_x = round((x + 0.35), 2)
    out_y = round(-(z - 0.24), 2) 
    out_z = round((-y + 0.3), 2) 
    return out_x, out_y, out_z

def callback_xy(data):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = quat[3]
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]


    pose_goal.position.x, pose_goal.position.y, pose_goal.position.z = transform(data.array[0], data.array[1], data.array[2])
    rospy.loginfo("DATA: %s",pose_goal.position)
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    if(plan):
        print "MOVED"
    else:
        print "FAILED"
    group.stop()
    group.clear_pose_targets()


if __name__ == "__main__":
    rospy.Subscriber("/position",array_float,callback_xy)
    rospy.spin()

