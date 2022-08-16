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
#include <autonomous_exploration/Obstacle.h>

// obstacle includes
#include <geometry_msgs/Pose2D.h>

// stl includes
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <Eigen/Dense>

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

	// adding obstacles to the goal
	std::vector<std::vector<Eigen::Vector2d>> vertices = {
		{{2.45, 1.15}, {2.90, 1.15}, {3.86, 1.95}, {3.86, 2.10}, 
		 {2.95, 2.10}, {2.95, 1.95}, {2.90, 1.40}, {2.45, 1.40}, 
		 {2.35, 1.95}, {2.35, 2.10}, {1.45, 2.10}, {1.45, 1.95}, 
		 {2.45, 1.4}},
		{{1.85, 3.20}, {3.35, 3.20}, {3.35, 3.35}, {1.85, 3.35}},
	};
	
	// total number of obstacles
	int num_obs = vertices.size();
	
	for(int i = 0; i < num_obs; i++) {
		// total number of points in one obstacle
		int num_points = vertices[i].size();
		
		// Obstacle data structure
		autonomous_exploration::Obstacle obs;
		obs.id = i;
		
		for(int j = 0; j < num_points; j++) {
			geometry_msgs::Point point;
			point.x = vertices[i][j][0];
			point.y = vertices[i][j][1];
			obs.points.push_back(point);
		}
		// adding all the obstacles.
		goal.obstacles.push_back(obs);
	}

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