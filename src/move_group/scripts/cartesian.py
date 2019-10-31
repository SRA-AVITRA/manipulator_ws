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

start = [0.454690388429,0.000719827773983,0.476488014448,0.00349018704767,0.712495632033,0.000152936112879,0.701667848444]

home = [0.195131774248,0.00191430998864, 0.777714415449,9.14285230913e-05,-0.00179983146408,0.00281640408846,0.999994410042]





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
    change_end_pose(group, start)
    print("START")
    
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
#change_joint_angles(move_group,[0,0,0,0,0,0])

draw(0,5,0)
draw(5,0,0)
draw(0,-5,0)
draw(-5,0,0)





