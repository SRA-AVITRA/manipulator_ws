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

#RPY of the end-effector
roll = 0
pitch = 0
yaw = 0 

#Offsets
#Base origin to shoulder_yaw_motor shaft
bot_x = 0.105
bot_y = 0
bot_z = 0.308 #0.28 
#Camera origin to shoulder_yaw_motor shaft
off_y = 0 #0.46
off_z = 0.194
off_x = 0.104
#To counter mechanical play
play_off_x = 0
play_off_y = 0.09
play_off_z = 0.1
#positioning the end-effector slightly behind target in x-axis
cartesian_off = 0
#When orientation is not constant, cartesian offset should be set to 0
count = 0

#Initializing moveit functions
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pose_goal',anonymous=True, disable_signals=True)
robot = moveit_commander.RobotCommander()               #interface b/w move_group node and the robot
scene = moveit_commander.PlanningSceneInterface()       #interface for the world around the robot

group_name = "arm"  #Planning group name set in Moveit setup assistant
group = moveit_commander.MoveGroupCommander(group_name)

group.set_goal_tolerance(0.0005)    #Tolerence for end-effector position at the goal 
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20) #publish trajectories for RViz to visualize

quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

def transform(x,y,z) :         #Computing target coordinates w.r.t base origin
    out = [0,0,0]
    sum_x, sum_y, sum_z = 0,0,0
    out_x = round((z + bot_x + off_x + play_off_x - cartesian_off), 2)
    out_y = round(-(x - off_y ) + play_off_y, 2) 
    out_z = round(((y - off_z) + bot_z + play_off_z), 2) 

    return out_x, out_y, out_z
    

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

        plan = group.go(wait=True)      #motion execution
        if(plan):
            print "MOVED"
        
        else:
            print "FAILED"
            
        group.stop()
        group.clear_pose_targets()

        count+=1
    rospy.signal_shutdown("Closing node")


if __name__ == "__main__":
    rospy.Subscriber("/position",array_float,callback_xy)
    rospy.spin()



