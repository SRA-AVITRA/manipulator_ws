#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "ros/ros.h"
#include <iostream>
using namespace std;
#include <cmath> 
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <move_group/array_float.h> 
#include <move_group/array.h> 

float bot_x = 0.18;
float bot_y = 0;
float bot_z = 0.31; //0.28 
float off_y = 0; //0.46
float off_z = 0.079;
float off_x = 0.19;
float play_off_z = 0.05;
float play_off_y = 0.06;
float cartesian_off = 0.05;
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;
float quat[4] = {0.0, 0.0, 0.0, 0.0};
int var = 0;

static const std::string PLANNING_GROUP = "arm";

// float* transform(float x, float y, float z)
// {
// 	float out[3] = {0.0, 0.0, 0.0};

// 	out[0] = round((z + bot_x + off_x - cartesian_off));
// 	out[1] = round(-(x - off_y ) + play_off_y); 
// 	out[2] = round(((y - off_z) + bot_z + play_off_z)); 

// 	return(out);
// }

void callback_xy(const move_group::array_float::ConstPtr& data)
{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	
	if(var == 0)
	{

		float x = data->array[0];
		float y = data->array[1];
		float z = data->array[2];
		float out[3] = {0.0, 0.0, 0.0};

		// ROS_INFO("RAW DATA: %f %f %f", x,y,z);
		// moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		// ROS_INFO("enter cb");

		tf2::Quaternion quat;
		quat.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
		
		geometry_msgs::Pose pose_goal;
	    pose_goal.orientation.w = quat[3];
	    pose_goal.orientation.x = quat[0];
	    pose_goal.orientation.y = quat[1];
	    pose_goal.orientation.z = quat[2];


		out[0] = ((z + bot_x + off_x - cartesian_off));
		out[1] = (-(x - off_y ) + play_off_y); 
		out[2] = (((y - off_z) + bot_z + play_off_z)); 

	    pose_goal.position.x = out[0];
	    pose_goal.position.y = out[1];
	    pose_goal.position.z = out[2];

	    ROS_INFO("DATA: %f %f %f", pose_goal.position.x, pose_goal.position.y, pose_goal.position.z);
	    move_group.setPoseTarget(pose_goal);
	    //move_group.setPlanningTime(10.0);

	    move_group.plan(my_plan);
	    ROS_INFO("Plan done");
	    move_group.execute(my_plan);
	    ROS_INFO("Execute done");
	    /* Uncomment below line when working with a real robot */		
	    // move_group.move(); 
	    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO("PLAN %s",success?"SUCCESS":"FAILED");


	}
	var++;
	move_group.stop();
	move_group.clearPoseTargets();	


}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "pose_goal");
	ros::NodeHandle node_handle;
  	ros::Subscriber sub = node_handle.subscribe("/position", 1000, callback_xy);
	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	// ros::waitForShutdown();
  	ros::spin();
  	return 0;
}