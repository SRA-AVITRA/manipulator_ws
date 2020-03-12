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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pose_goal',anonymous=True, disable_signals=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)
group.set_goal_tolerance(0.0005)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

x,y,z = 0, 0, 0
bot_x = 0.105 #0.18 //Bot centre to manipulator centre
bot_y = 0
bot_z = 0.308 #0.28 
off_y = 0 #0.46 //Camera centre to manipulator centre
off_z = 0.194
off_x = 0.104 #0.19
play_off_z = 0.1 #0.05 #5
play_off_y = 0.09 #0.06 #6
play_off_x = 0.08
cartesian_off = 0.0 # 0.05
        
# def camera_coordinates(cam_array):
#     global obj_odom
#     obj.point.x = cam_array.array[2]
#     obj.point.y = -cam_array.array[0]
#     obj.point.z = 0.0
#     obj_odom=listener.transformPoint("bot",obj)

roll, pitch, yaw = 0, 0,0
quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
count = 0
def callback_xy(data):
    global count
    print("here")
    if count == 0:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = quat[3]
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]

        pose_goal.position.x = round(data.point.x,2)
        pose_goal.position.y = round(data.point.y,2)
        pose_goal.position.z = round(data.point.z,2)
        
        # pose_goal.position.x, pose_goal.position.y, pose_goal.position.z =  transform(data.array[0], data.array[1], data.array[2])
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
    rospy.signal_shutdown("Closing node")


if __name__ == "__main__":
    cam_broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber("/object",PointStamped,callback_xy)
    rospy.spin()


