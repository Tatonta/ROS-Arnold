#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	// Initialize the simple navigation goals node
	ros::init(argc, argv, "move_tonino");
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	// Wait 5 sec for move base action server to come up
	while(!ac.waitForServer (ros::Duration(8.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	move_base_msgs::MoveBaseGoal goal;
	// set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	float goals[5][3] =  {{1.82, 1.28, 0.708}, {-1.24, 1.00, 2.26}, {-1.88, -4.70 , -2.195}, {2.46, -4.80, -0.859}, {0.0, 0.0, 6.28}}; //{1.82, 1.28, 0.708}, {-1.24, 1.00, 2.26}, {-1.88, -2.48 , -2.195}, {2.46, -2.67, -0.859}, {0.0, 0.0, 6.28}
    //float goals[5][3] = {{1.57, 1.03, 0.708}, {2.21, -2.42, -0.86}, {-1.0, 0.75, 2.26}, {-1.63, -2.23 , -2.195}, {0.0, 0.0, 6.28}}; // con 4.50 m
	for (int i =0; i< 5; i++){
		// Define a position and orientation for the robot to reach
		goal.target_pose.pose.position.x = goals[i][0];
		goal.target_pose.pose.position.y = goals[i][1];
		goal.target_pose.pose.orientation.w = goals[i][2];
		// Send the goal position and orientation for the robot to reach
		ROS_INFO ("Sending goal");
		ac.sendGoal (goal);
		// Wait an infinite time for the results
		ac.waitForResult();
		ros::Duration(2.0).sleep();
	}
	// Check if the robot reached ts goal
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO( "Hooray, reached drop off zone");
	else
		ROS_INFO("The base failed to move forward");
	return 0;
}