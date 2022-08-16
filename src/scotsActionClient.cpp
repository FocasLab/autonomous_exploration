/*
 * File Name: scotsActionServer.cpp
 *
 * Author: Jay Bhagiya 
*/

// ros includes
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <autonomous_exploration/autoExplAction.h>
#include <autonomous_exploration/Target.h>

// obstacle includes
#include <geometry_msgs/Pose2D.h>

// stl includes
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
	// ros node initialize
	ros::init(argc, argv, "scotsActionClient");

	// creating simple action client
	// ture for client needs to spin its own thread
	actionlib::SimpleActionClient<autonomous_exploration::autoExplAction> ac_("/scots", true);

	// waiting till action server is not started.
	std::cout << "Waiting for for action server to start.\n" << std::endl;
	ac_.waitForServer();

	std::cout << "Action server is started, sending goals.." << std::endl;

	autonomous_exploration::autoExplGoal goal;

	// adding targets to the goal
	std::vector<std::vector<double>> target_data = {
		{9.80, 10.2, 8.80, 9.20},
		{7.80, 8.20, 10.8, 11.2},
		{8.60, 9.00, 5.60, 6.00},
		{1.10, 1.50, 0.80, 1.20}
	};

	// total number of targets
	int num_targets = target_data.size();

	for(int i = 0; i < num_targets; i++) {
		// total number of points in one target
		int num_points = target_data[i].size();

		autonomous_exploration::Target tr;
		tr.id = i;

		for(int j = 0; j < num_points; j++) {
			tr.points.push_back(target_data[i][j]);
		}
		// adding all the targets.
		goal.targets.push_back(tr);
	}

	// sending the goal
	ac_.sendGoal(goal);

	// wait for the result
	bool result_state = ac_.waitForResult();

	if(result_state) {
		actionlib::SimpleClientGoalState state = ac_.getState();
		std::cout << "Goal finished, " << state.toString().c_str() << std::endl;
	}
	else
		std::cout << "Goal did not finished." << std::endl;

	return 0;
}