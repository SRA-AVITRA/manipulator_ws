#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String , Float64
from dynamixel_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
import tf
from perception.msg import array_float, array

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pose_goal',anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)
group.set_goal_tolerance(0.0005)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

bot_x = 0.18
bot_y = 0
bot_z = 0.31 #0.28 
off_y = 0 #0.46
off_z = 0.079
off_x = 0.19
play_off_z = 0.05
play_off_y = 0.06
cartesian_off = 0.05

roll, pitch, yaw = 0, 0,0
quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

def transform(x,y,z) :
    out = [0,0,0]
    sum_x, sum_y, sum_z = 0,0,0
    out_x = round((z + bot_x + off_x - cartesian_off), 2)
    out_y = round(-(x - off_y ) + play_off_y, 2) 
    out_z = round(((y - off_z) + bot_z + play_off_z), 2) 

    return out_x, out_y, out_z
    
count = 0
        

def callback_xy(data):
    global count
    if count == 0:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = quat[3]
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]

        pose_goal.position.x, pose_goal.position.y, pose_goal.position.z =  transform(data.array[0], data.array[1], data.array[2])
# [-0.042422794, 0.11115891, 0.39400002]
        rospy.loginfo("DATA: %s",pose_goal.position)
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)
        if(plan):
            print "MOVED"
        
        else:
            print "FAILED"
            
        group.stop()
        group.clear_pose_targets()

        count+=1

if __name__ == "__main__":
    rospy.Subscriber("/position",array_float,callback_xy)
    rospy.spin()



