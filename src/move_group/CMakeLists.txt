cmake_minimum_required(VERSION 2.8.3)
project(move_group)

add_compile_options(-std=c++11)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  roscpp
  rospy
  std_msgs
)
find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  /opt/ros/kinetic/include/moveit/robot_model
  /opt/ros/kinetic/include/moveit/robot_model_loader
  /opt/ros/kinetic/include/moveit/robot_state
)
catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)
include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


##FOR EVERY NEW CPP FILE ADD BELOW 4 LINES

#####
add_executable(robot_model src/robot_model.cpp)
target_link_libraries(robot_model ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS robot_model DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#####