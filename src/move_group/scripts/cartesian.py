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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('cartesian', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

start = [0.431930257548,0.0163324526093,0.40211261149,0.7079249301,-0.706186690168,-0.0114258354273,0.00347882972886]
home = [0.111897564472,-0.0177947739322,0.721684148614,-0.500086242412,-0.508889465502,0.497331187906,0.493565549434]


def change_end_pose(group, input_pose):
    pose_goal = geometry_msgs.msg.Pose()
    end_goal = pose_to_list(pose_goal)
    
    for i, ele in enumerate(input_pose):
        if ele != None:
            end_goal[i] = ele
    group.set_pose_target(list_to_pose(end_goal))
    if group.go(wait=True):
        print "Solution found for ", input_pose
        print "Group moved to ", end_goal
    else:
        print "No solution found for", input_pose
    group.stop()
    group.clear_pose_targets()

def change_joint_angles(group, joint_goal):
    joints = group.get_current_joint_values()
    for i in range(len(joint_goal)):
        joints[i] = joint_goal[i]
    group.go(joints, wait=True)
    group.stop()
    print "\nJoint angles moved to ", joints

def init(group):
    change_end_pose(group, home)
    print("HOME")
    #change_end_pose(group, start)
    #print("START")
scale = 0.01
def draw(x,y,z):
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.x += scale * x
    wpose.position.y += scale * y
    wpose.position.z += scale * z
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.001,        # eef_step
                                   5)         # jump_threshold
    move_group.execute(plan, wait=True)
    print(fraction)

init(move_group)

draw(0,0,-5)
draw(0,+10,0)
draw(10,0,0)
draw(0,-10,0)
draw(-10,0,0)





