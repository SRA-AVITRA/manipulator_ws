import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('cartesian', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

waypoints = []
scale  = 1
wpose = move_group.get_current_pose().pose
wpose.position.z -= scale * 0.1  # First move up (z)
wpose.position.y += scale * 0.2  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))



wpose.position.y -= scale * 0.1  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose))
print(waypoints)
# (plan, fraction) = move_group.compute_cartesian_path(
#                                    waypoints,   # waypoints to follow
#                                    0.01,        # eef_step
#                                    0.0)         # jump_threshold

# move_group.execute(plan, wait=True)