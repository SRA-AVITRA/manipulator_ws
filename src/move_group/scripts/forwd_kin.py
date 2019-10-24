#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float64
from dynamixel_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list, list_to_pose
import tf
import numpy as np



PI = np.pi
PI_2 = np.pi/2

theta = [0,PI_2,0,0,0,0]

def rot_z(angle):
    a = np.array([[np.cos(angle),-np.sin(angle),0,0],
                [np.sin(angle),np.cos(angle),0,0],
                [0,0,1,0],
                [0,0,0,1]])
    return a
def rot_y(angle):
    a = np.array([[np.cos(angle),0,np.sin(angle),0],
                [0,1,0,0],
                [-np.sin(angle),0,np.cos(angle),0],
                [0,0,0,1]])
    return a
def rot_x(angle):
    a = np.array([[1,0,0,0],
                [0,np.cos(angle),-np.sin(angle),0],
                [0,np.sin(angle),np.cos(angle),0],
                [0,0,0,1]])
    return a

def trans_x(x):
    a = np.array([[1,0,0,x],
                [0,1,0,0],
                [0,0,1,0],
                [0,0,0,1]])
    return a

def trans_y(y):
    a = np.array([[1,0,0,0],
                [0,1,0,y],
                [0,0,1,0],
                [0,0,0,1]])
    return a

def trans_z(z):
    a = np.array([[1,0,0,0],
                [0,1,0,0],
                [0,0,1,z],
                [0,0,0,1]])
    return a

def change_joint_angles(group, joint_goal):
    joints = group.get_current_joint_values()
    for i in range(len(joint_goal)):
        joints[i] = joint_goal[i]
    group.go(joints, wait=True)
    group.stop()
    print "\nJoint angles moved to ", joints



z_trans = np.matmul(rot_z(theta[0]),trans_z(36))
x_trans = np.matmul(trans_x(0),rot_x(-PI_2))
T_0_1 = np.matmul(z_trans,x_trans)

z_trans = np.matmul(rot_z(theta[1]-PI_2),trans_z(0))
x_trans = np.matmul(trans_x(175),rot_x(0))
T_1_2 = np.matmul(z_trans,x_trans)

z_trans = np.matmul(rot_z(theta[2]),trans_z(0))
x_trans = np.matmul(trans_x(204),rot_x(0))
T_2_3 = np.matmul(z_trans,x_trans)

z_trans = np.matmul(rot_z(theta[3]+PI_2),trans_z(0))
x_trans = np.matmul(trans_x(73),rot_x(-PI_2))
T_3_4 = np.matmul(z_trans,x_trans)

z_trans = np.matmul(rot_z(theta[4]+PI_2),trans_z(0))
x_trans = np.matmul(trans_x(0),rot_x(PI_2))
T_4_5 = np.matmul(z_trans,x_trans)

z_trans = np.matmul(rot_z(theta[5]),trans_z(35))
x_trans = np.matmul(trans_x(0),rot_x(0))
T_5_6 = np.matmul(z_trans,x_trans)

T_0_2 = np.matmul(T_0_1,T_1_2)
T_0_3 = np.matmul(T_0_2,T_2_3)
T_0_4 = np.matmul(T_0_3,T_3_4)
T_0_5 = np.matmul(T_0_4,T_4_5)
T_0_6 = np.matmul(T_0_5,T_5_6)

shoulder_yaw = rospy.Publisher("/shoulder_yaw_controller/command", Float64, queue_size=10)
shoulder_pitch = rospy.Publisher("/shoulder_pitch_controller/command", Float64, queue_size=10)
elbow = rospy.Publisher("/elbow_controller/command", Float64, queue_size=10)
wrist_pitch = rospy.Publisher("/wrist_pitch_controller/command", Float64, queue_size=10)
wrist_yaw = rospy.Publisher("/wrist_yaw_controller/command", Float64, queue_size=10)
wrist_roll = rospy.Publisher("/wrist_roll_controller/command", Float64, queue_size=10)


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('forward_kinematics', anonymous=True)
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


print("x = ",T_0_6[0][3])
print("y = ",T_0_6[1][3])
print("z = ",T_0_6[2][3])


change_joint_angles(group_arm,theta)

rospy.spin()