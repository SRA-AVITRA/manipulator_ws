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

scale = 0.1

#Initializing moveit functions
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('cartesian',anonymous=True)
robot = moveit_commander.RobotCommander()           #interface b/w move_group node and the robot
scene = moveit_commander.PlanningSceneInterface()   #interface for the world around the robot

group_name = "arm"  #Planning group name set in Moveit setup assistant
group = moveit_commander.MoveGroupCommander(group_name)

group.set_goal_tolerance(0.0005)                    #Tolerence for end-effector position at the goal 
group.set_goal_orientation_tolerance(0.0001)        #Tolerence for end-effector orientation at the goal
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20) #publish trajectories for RViz to visualize

def draw(x,y,z):
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x += scale * x       #move along x-axis
    wpose.position.y += scale * y       #move along y-axis
    wpose.position.z += scale * z       #move along z-axis
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                   waypoints,       # waypoints to follow
                                   0.005,           # eef_step : resolution
                                   5)               # jump_threshold
    
    group.execute(plan, wait=True)  #motion execution
    print(fraction)

draw(5,0,0)    






