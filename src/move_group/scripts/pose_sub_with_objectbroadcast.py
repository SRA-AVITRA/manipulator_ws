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
from geometry_msgs.msg import PointStamped
import tf2_ros

#RPY of the end-effector
roll = 0
pitch = 0
yaw = 0	

#Offsets
play_off_x = 0.06		#countering mechanical play in x axis 
play_off_y = 0.06		#countering mechanical play in y axis 
play_off_z = 0.05 		#countering mechanical play in z axis 
cartesian_off = 0.05	#positioning the end-effector slightly behind target in x-axis
#When orientation is not constant, cartesian offset should be set to 0
count = 0

#Initializing moveit functions
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pose_goal',anonymous=True, disable_signals=True)
robot = moveit_commander.RobotCommander()				#interface b/w move_group node and the robot
scene = moveit_commander.PlanningSceneInterface()		#interface for the world around the robot

group_name = "arm"	#Planning group name set in Moveit setup assistant
group = moveit_commander.MoveGroupCommander(group_name)

group.set_goal_tolerance(0.0005)	#Tolerence for end-effector position at the goal 
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20) #publish trajectories for RViz to visualize

def callback_xy(data):
    global count
    if count == 0:
        pose_goal = geometry_msgs.msg.Pose()
        quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw)	#RPY to quaternion conversion

        pose_goal.orientation.w = quat[3]
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]

        pose_goal.position.x = round(data.point.x - cartesian_off ,2)
        pose_goal.position.y = round(data.point.y + play_off_y ,2)
        pose_goal.position.z = round(data.point.z + play_off_z ,2)
        
        rospy.loginfo("DATA: %s",pose_goal.position)
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)	#motion execution 
        if(plan):
            print "MOVED"  
        else:
            print "FAILED"
            
        group.stop()	#Calling `stop()` ensures that there is no residual movement
        group.clear_pose_targets()

        count+=1
        
    rospy.signal_shutdown("Closing node")


if __name__ == "__main__":
    cam_broadcaster = tf.TransformBroadcaster()		#Transforms camera origin to base origin
    rospy.Subscriber("/object",PointStamped,callback_xy)
    rospy.spin()


