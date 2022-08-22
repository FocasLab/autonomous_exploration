/*
 * File Name: scotsActionServer.cpp
 *
 * Author: Jay Bhagiya 
*/

// ros includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <autonomous_exploration/autoExplAction.h>
#include <autonomous_exploration/Target.h>

// ros robot includes
#include <phasespace_msgs/Markers.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Empty.h>

// rviz visualization
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// scots includes 
#include "autonomous_exploration/scots.hpp"
#include "autonomous_exploration/RungeKutta4.hpp"
#include "autonomous_exploration/TicToc.hpp"

// stl includes
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <Eigen/Dense>

// memory profiling
#include <sys/time.h>
#include <sys/resource.h>


class scotsActionServer
{
	protected:
		ros::NodeHandle nh_;
		
		// NodeHandle instance must be created before this line. Otherwise strange error occurs.
		actionlib::SimpleActionServer<autonomous_exploration::autoExplAction> as_;
		std::string action_name_;

		// create messages that are used to published feedback/result
		autonomous_exploration::autoExplFeedback feedback_;
		autonomous_exploration::autoExplResult result_;

		// state space
		std::vector<int> map_vector;
		double resolution;
		int width, height;

		// robot state 
		geometry_msgs::Pose2D curr_pose;

		// publisher and subscriber handlers
		std::string pose_topic_name_ = "/robot_pose";
		ros::Subscriber robot_pose;

		std::string vel_topic_name_ = "/cmd_vel";
		ros::Publisher robot_vel;

		// map subscriber
		std::string map_topic_name = "/map";
		ros::Subscriber map_sub;

		// for services
		std_srvs::Empty::Request req;
		std_srvs::Empty::Response resp;

		// origin update client
		std::string origin_update_service_name = "/update_origin";
		ros::ServiceClient origin_update_client;

		// 
		std::string send_new_goal_service_name = "/new_goal";
		ros::ServiceClient send_new_goal_client;

		// obstacle visualization
		std::string obs_topic_name = "/scots_visualization";
		ros::Publisher markers_pub;

		// robot path visualization
		std::string path_topic_name = "/path_visualization";
		ros::Publisher path_pub;

		// trajectory visualization
		std::string trajectory_topic_name = "/trajectory_visualization";
		ros::Publisher trajectory_pub;

		// global variables
		static const int state_dim = 3;
		static const int input_dim = 2;
		static constexpr double tau = 2.1;

		using state_type = std::array<double, state_dim>;
		using input_type = std::array<double, input_dim>;
		using abs_type = scots::abs_type;

		// for time profiling
		TicToc tt;
	
	public:
		scotsActionServer(std::string name) : 
		// Bind the callback to the action server. False is for thread spinning
		as_(nh_, name, boost::bind(&scotsActionServer::processGoal, this, _1), false),
		action_name_(name) {
			// subscribers
			robot_pose = nh_.subscribe(pose_topic_name_, 10, &scotsActionServer::robotPoseCallback_2, this);
			map_sub = nh_.subscribe(map_topic_name, 10, &scotsActionServer::mapCallback, this);
			
			// publishers
			robot_vel = nh_.advertise<geometry_msgs::Twist>(vel_topic_name_, 10);
			markers_pub = nh_.advertise<visualization_msgs::Marker>(obs_topic_name, 10);
			path_pub = nh_.advertise<nav_msgs::Path>(path_topic_name, 10);
			trajectory_pub = nh_.advertise<nav_msgs::Path>(trajectory_topic_name, 10);

			// actions
			as_.start();
			std::cout << "Scots Action Server is started, now you can send the goals." << std::endl;
			
			// services
			origin_update_client = nh_.serviceClient<std_srvs::Empty>(origin_update_service_name);
			send_new_goal_client = nh_.serviceClient<std_srvs::Empty>(send_new_goal_service_name);

			std::cout << "Waiting for services.." << std::endl;
			ros::service::waitForService(origin_update_service_name, ros::Duration(10));
			ros::service::waitForService(send_new_goal_service_name, ros::Duration(10));
			std::cout << "Done." << std::endl;

			bool send_new_goal_success = send_new_goal_client.call(req, resp);
		}

		~scotsActionServer(void)
		{
		}

		void robotPoseCallback_1(const phasespace_msgs::Markers &msg) {
			phasespace_msgs::Marker marker_dyn = msg.markers[0];
			phasespace_msgs::Marker marker_ori = msg.markers[1];

			curr_pose.x = marker_dyn.x;
			curr_pose.y = marker_dyn.y;

			double dy = marker_dyn.y - marker_ori.y;
			double dx = marker_dyn.x - marker_ori.x;

			double angle = std::atan2(dy, dx);
			curr_pose.theta = angle;
		}

		void robotPoseCallback_2(const geometry_msgs::Pose2D &msg) {
			curr_pose.x = msg.x;
			curr_pose.y = msg.y;
			curr_pose.theta = msg.theta;
		}

		void mapCallback(const nav_msgs::OccupancyGrid &msg) {
			map_vector.clear();
			resolution = msg.info.resolution;
			width = msg.info.width;
			height = msg.info.height;

			// map_vector = msg.data;
			for(int i = 0; i < width * height; i++) {
				map_vector.push_back(msg.data[i]);
			}
		}

		geometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw) {
			tf2::Quaternion quat;
			quat.setRPY(0, 0, yaw);

			geometry_msgs::Quaternion quat_msgs;
			tf2::convert(quat, quat_msgs);

			return quat_msgs;
		}

		std::vector<std::vector<int>> getMapMatrix(const std::vector<int> &map_vector, int width, int height) {
			std::vector<std::vector<int>> map;

			for(int i = 0; i < height; i++) {
				std::vector<int> map_i;
				for(int j = 0; j < width; j++) {
					int idx = width * i + j;
					if(map_vector[idx] > 0)
						map_i.push_back(1);
					else
						map_i.push_back(map_vector[idx]);
				}
				map.push_back(map_i);
			}
			return map;
		}

		void visualizeObstacles(const scots::UniformGrid &ss, std::vector<std::vector<int>> &maps) {
			// visaulization parameters
			visualization_msgs::Marker points;

			points.header.frame_id = "origin";
			points.header.stamp = ros::Time::now();
			
			points.ns = "obstacles";
			points.id = 0;
			points.type = visualization_msgs::Marker::POINTS;
			points.action = visualization_msgs::Marker::ADD;

			points.pose.orientation.w = 1;

			points.scale.x = 0.1;
			points.scale.y = 0.1;

			points.color.r = 1.0f;
			points.color.g = 1.0f;
			points.color.a = 0.6;

			points.lifetime = ros::Duration();

			// total number of cells
			abs_type num_cell = ss.size();
			std::vector<abs_type> NN = ss.get_nn();

			std::cout << "Number of cells: " << num_cell << std::endl;

			// check for only (x, y) state space (num_grid_x * num_grid_y)
			for(abs_type i = 0; i < NN[2]; i++) {
				state_type x;
				ss.itox(i, x);

				// ratio of scots grid(s_eta) to map grid (resolution)
				// 0.2 is added for floating point numbers
				std::vector<int> grid_ratio{int((ss.get_eta()[0] / resolution) + 0.2), int((ss.get_eta()[1] / resolution) + 0.2)};

				// coordinates to search in map matrix
				// 0.2 is added for floating point numbers.
				std::vector<int> cord{int((x[0] / resolution) + 0.2), int((x[1] / resolution) + 0.2)};

				geometry_msgs::Point pt;

				for(int i = -1; i < grid_ratio[1] + 1; i++) {
					for(int j = -1; j < grid_ratio[0] + 1; j++) {
						if(cord[1] + i >= 0 && cord[1] + i< height && cord[0] + j >= 0 && cord[0] + j < width) {
							if(maps[cord[1] + i][cord[0] + j] != 0){
								pt.x = x[0] + 0.05;
								pt.y = x[1] + 0.05;
								points.points.push_back(pt);
							}
						}
					}
				}
				markers_pub.publish(points);
			}
		}

		void visualizeTargets(const autonomous_exploration::Target &tr) {
			// visaulization parameters
			visualization_msgs::Marker lines;

			lines.header.frame_id = "origin";
			lines.header.stamp = ros::Time::now();
			
			lines.ns = "target_window";
			lines.id = 1;
			lines.type = visualization_msgs::Marker::LINE_STRIP;
			lines.action = visualization_msgs::Marker::ADD;

			lines.pose.orientation.w = 1;

			lines.scale.x = 0.02;

			lines.color.g = 1.0f;
			lines.color.a = 1.0;

			lines.lifetime = ros::Duration();

			double box_length = tr.points[1] - tr.points[0];
			double box_width = tr.points[3] - tr.points[2];

			std::vector<double> diff_x = {0, box_width, box_width, 0, 0};
			std::vector<double> diff_y = {0, 0, box_length, box_length, 0};

			for(int i = 0; i < diff_x.size(); i++) {
				geometry_msgs::Point pt;
				
				pt.x = tr.points[0] + diff_x[i];
				pt.y = tr.points[2] + diff_y[i];

				lines.points.push_back(pt);
			}

			markers_pub.publish(lines);
		}

		scots::StaticController getController(const scots::UniformGrid &ss, const scots::UniformGrid &is, const scots::TransitionFunction &tf, 
			const state_type &s_eta, const autonomous_exploration::Target &tr) {
			
			// defining target set
			auto target = [&ss, &s_eta, &tr](const abs_type& idx) {
				state_type x;
				ss.itox(idx, x);
				// function returns 1 if cell associated with x is in target set 
				if (tr.points[0] <= x[0] && x[0] <= tr.points[1] && 
					tr.points[2] <= x[1] && x[1] <= tr.points[3])
				  return true;
				return false;
			};

			std::cout << "\nSynthesis for target, " << tr.id << std::endl;
			tt.tic();
			scots::WinningDomain win_domain = scots::solve_reachability_game(tf, target);
			tt.toc();
			std::cout << "\nWinning domain for targer id " << tr.id << ", is " << win_domain.get_size() << std::endl;

			scots::StaticController controller = scots::StaticController(ss, is, std::move(win_domain));

			std::cout << "Writing to the file." << std::endl;
			if(write_to_file(controller, "autoExpl"))
				std::cout << "Done.\n";

			return controller;
		}

		bool rotateRobotForSomeTime(const int time) {

			geometry_msgs::Twist vel_msg_turtle;
			vel_msg_turtle.linear.x = 0.0;
			vel_msg_turtle.angular.z = 0.1;

			// this is to maintain, that robot will receive same speed for tau time.
			ros::Time beginTime = ros::Time::now();
			ros::Duration secondsIWantToSendMessagesFor = ros::Duration(time);
			ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;

			while(ros::Time::now() < endTime )
			{
				robot_vel.publish(vel_msg_turtle);

				// Time between messages, so you don't blast out an thousands of
				// messages in your tau secondperiod
				ros::Duration(0.1).sleep();
			}
			return true;
		}

		bool reachTarget(bool &success, const scots::StaticController &controller, const autonomous_exploration::Target &tr) {
			//defining dynamics of robot
			auto vehicle_post = [](state_type &x, const input_type &u) {
				auto rhs = [](state_type& xx, const state_type &x, const input_type &u) {
					xx[0] = u[0] * std::cos(x[2]); 
					xx[1] = u[0] * std::sin(x[2]);
					xx[2] = u[1];
				};
				scots::runge_kutta_fixed4(rhs, x, u, state_dim, tau, 10);
			};

			// defining target set
			auto target = [&tr](const state_type& x) {
				// function returns 1 if cell associated with x is in target set 
				if (tr.points[0] <= x[0] && x[0] <= tr.points[1] && tr.points[2] <= x[1] && x[1] <= tr.points[3])
				  return true;
				return false;
			};

			state_type robot_state = {curr_pose.x, curr_pose.y, curr_pose.theta};

			// path visualization object
			nav_msgs::Path path;
			path.header.stamp = ros::Time::now();
			path.header.frame_id = "origin";

			geometry_msgs::PoseStamped path_poses;

			path_poses.header.stamp = ros::Time::now();
			path_poses.header.frame_id = "origin";

			path_poses.pose.position.x = robot_state[0];
			path_poses.pose.position.y = robot_state[1];
			path_poses.pose.orientation = createQuaternionMsgFromYaw(robot_state[2]);

			path.poses.push_back(path_poses);

			// robot vel msg object
			geometry_msgs::Twist vel_msg_turtle;

			while(ros::ok()) {
				
				if(as_.isPreemptRequested()) {
					std::cout << "\nPreempted request for, " << action_name_.c_str() << std::endl;
					// set the action state to preempted
					as_.setPreempted();
					success = false;
					break;
				}

				robot_state = {curr_pose.x, curr_pose.y, curr_pose.theta};

				if(!(target(robot_state))) {
					// getting ready feedback handler
					std::cout << "Robot's Current Pose: " << robot_state[0] << ", " 
														  << robot_state[1] << ", " 
														  << robot_state[2] << std::endl;
					feedback_.curr_pose = curr_pose;

					std::vector<input_type> control_inputs = controller.peek_control<state_type, input_type>(robot_state);

					// if(control_inputs.size() < 1) {
					// 	vel_msg_turtle.linear.x = 0.0;
					// 	vel_msg_turtle.angular.z = 0.0;

					// 	robot_vel.publish(vel_msg_turtle);
					// 	success = false;
					// 	std::cout << "Robot is not in the Winning domain or the Winning domain is too small. Try with different targets." << std::endl;
					// 	break;
					// }

					vel_msg_turtle.linear.x = control_inputs[0][0];
					vel_msg_turtle.angular.z = control_inputs[0][1];

					// pusblishing the current feedback to action client
					as_.publishFeedback(feedback_);
				}
				else {
					// rotote the robot for some time so the map is updated (lidar is 360, but due to limitations of slam we rotating the robot.)
					bool status = rotateRobotForSomeTime(10);

					vel_msg_turtle.linear.x = 0.0;
					vel_msg_turtle.angular.z = 0.0;

					robot_vel.publish(vel_msg_turtle);
					success = true;
					break;
				}

				// this is to maintain, that robot will receive same speed for tau time.
				ros::Time beginTime = ros::Time::now();
				ros::Duration secondsIWantToSendMessagesFor = ros::Duration(tau);
				ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;

				while(ros::Time::now() < endTime )
				{
					robot_vel.publish(vel_msg_turtle);

					// Time between messages, so you don't blast out an thousands of
					// messages in your tau secondperiod
					ros::Duration(0.1).sleep();

					// pushing current pose in nav_msgs::Path for path visualization
					path_poses.pose.position.x = curr_pose.x;
					path_poses.pose.position.y = curr_pose.y;
					path_poses.pose.orientation = createQuaternionMsgFromYaw(curr_pose.theta);

					path.poses.push_back(path_poses);

					path_pub.publish(path);
				}
			}
			return success;
		}

		bool simulatePath(bool &success, const scots::StaticController &controller, const autonomous_exploration::Target &tr) {
			//defining dynamics of robot
			auto vehicle_post = [](state_type &x, const input_type &u) {
				auto rhs = [](state_type& xx, const state_type &x, const input_type &u) {
					xx[0] = u[0] * std::cos(x[2]); 
					xx[1] = u[0] * std::sin(x[2]);
					xx[2] = u[1];
				};
				scots::runge_kutta_fixed4(rhs, x, u, state_dim, tau, 10);
			};

			// defining target set
			auto target = [&tr](const state_type& x) {
				// function returns 1 if cell associated with x is in target set 
				if (tr.points[0] <= x[0] && x[0] <= tr.points[1] && tr.points[2] <= x[1] && x[1] <= tr.points[3])
				  return true;
				return false;
			};

			state_type robot_state = {curr_pose.x, curr_pose.y, curr_pose.theta};

			// path visualization objects
			nav_msgs::Path trajectory;
			trajectory.header.stamp = ros::Time::now();
			trajectory.header.frame_id = "origin";

			geometry_msgs::PoseStamped trajectory_poses;

			trajectory_poses.header.stamp = ros::Time::now();
			trajectory_poses.header.frame_id = "origin";

			trajectory_poses.pose.position.x = robot_state[0];
			trajectory_poses.pose.position.y = robot_state[1];
			trajectory_poses.pose.orientation = createQuaternionMsgFromYaw(robot_state[2]);

			trajectory.poses.push_back(trajectory_poses);

			while(ros::ok()) {
				// getting ready feedback handler
				// std::cout << "Simulation: Robot's Current Pose: " << robot_state[0] << ", " 
				// 									  			  << robot_state[1] << ", " 
				// 									  			  << robot_state[2] << std::endl;

				if(target(robot_state)) {
					// std::cout << "Reached: " << robot_state[0] << ", " 
					// 						 << robot_state[1] << ", " 
					// 						 << robot_state[2] << std::endl;

					success = true;
					trajectory_pub.publish(trajectory);
					break;
				}

				std::vector<input_type> control_inputs = controller.peek_control<state_type, input_type>(robot_state);

				vehicle_post(robot_state, control_inputs[0]);

				trajectory_poses.pose.position.x = robot_state[0];
				trajectory_poses.pose.position.y = robot_state[1];
				trajectory_poses.pose.orientation = createQuaternionMsgFromYaw(robot_state[2]);

				trajectory.poses.push_back(trajectory_poses);

			}
			return success;
		}

		void processGoal_1(const autonomous_exploration::autoExplGoalConstPtr &goal) {
			bool success = true;
			// Parsing targets
			int num_targets = goal->targets.size();
			// std::vector<scots::StaticController> controllers;

			// for(int i = 0; i < num_targets; i++) {
			// 	controllers.push_back(getController(ss, s_eta, goal->targets[i]));
			// }

			// scots::StaticController controller = getController(ss, is, tf, s_eta, goal->targets[1]);
			scots::StaticController controller; 
			if(!read_from_file(controller, "autoExpl")) {
				std::cout << "Could not able read for file." << std::endl;
				success = false;
			}

			std::cout << "\n\nTarget Locked, starting to proceed." << std::endl;
			success = simulatePath(success, controller, goal->targets[0]);
			success = reachTarget(success, controller, goal->targets[0]);

			if(success) {
				result_.target_id = 0;
				result_.synthesis_time = 0.0;
				result_.completion_time = 0.0;

				std::cout << "Succeeded for: " << action_name_.c_str() << std::endl;
				// set the action state to succeeded
				as_.setSucceeded(result_);
			}
		}
		
		void processGoal(const autonomous_exploration::autoExplGoalConstPtr &goal) {

			struct rusage usage;

		 	auto vehicle_post = [](state_type &x, const input_type &u) {
				auto rhs = [](state_type& xx, const state_type &x, const input_type &u) {
					xx[0] = u[0] * std::cos(x[2]); 
					xx[1] = u[0] * std::sin(x[2]);
					xx[2] = u[1];
				};
				scots::runge_kutta_fixed4(rhs, x, u, state_dim, tau, 10);
			};

			auto radius_post = [](state_type &r, const state_type &, const input_type &u) {
				const state_type w = {{0.01, 0.01}};
				r[0] = r[0] + r[2] * std::abs(u[0]) * tau + w[0];
				r[1] = r[1] + r[2] * std::abs(u[0]) * tau + w[1];
			};

			double lb = width * resolution;
			double ub = height * resolution;
			
			state_type s_lb={{0, 0, -3.5}};
			state_type s_ub={{std::ceil(lb * 100.0) / 100.0, std::ceil(ub * 100.0) / 100.0, 3.5}};
			state_type s_eta={{.1, .1, .2}};

			scots::UniformGrid ss(state_dim, s_lb, s_ub, s_eta);
			std::cout << std::endl;
			ss.print_info();
			
			input_type i_lb={{-0.22, -0.11}};
			input_type i_ub={{ 0.22, 0.11}};
			input_type i_eta={{.02, .01}};
			  
			scots::UniformGrid is(input_dim, i_lb, i_ub, i_eta);
			std::cout << std::endl;	
			is.print_info();

			// update the origin
			bool origin_update_success = origin_update_client.call(req, resp);

			// result parameter
			bool success = true;

			std::vector<std::vector<int>> maps = getMapMatrix(map_vector, width, height);

			int target_no = 0;

			visualizeObstacles(ss, maps);
			visualizeTargets(goal->targets[target_no]);

			auto avoid = [&maps, &ss, width=width, height=height, resolution=resolution](const abs_type& idx) {
				state_type x;
				ss.itox(idx, x);

				// ratio of scots grid(s_eta) to map grid (resolution)
				// 0.2 is added for floating point numbers
				std::vector<int> grid_ratio{int((ss.get_eta()[0] / resolution) + 0.2), int((ss.get_eta()[1] / resolution) + 0.2)};

				// coordinates to search in map matrix
				// 0.2 is added for floating point numbers.
				std::vector<int> cord{int((x[0] / resolution) + 0.2), int((x[1] / resolution) + 0.2)};

				geometry_msgs::Point pt;

				for(int i = -1; i < grid_ratio[1] + 1; i++) {
					for(int j = -1; j < grid_ratio[0] + 1; j++) {
						if(cord[1] + i >= 0 && cord[1] + i< height && cord[0] + j >= 0 && cord[0] + j < width) {
							if(maps[cord[1] + i][cord[0] + j] != 0){
					 			return true;
							}
						}
					}
				}
				return false;
			};

			std::cout << "\nComputing the transition function." << std::endl;
  
			/* transition function of symbolic model */
			scots::TransitionFunction tf;
			scots::Abstraction<state_type,input_type> abs(ss, is);

			tt.tic();
			abs.compute_gb(tf, vehicle_post, radius_post, avoid);
			tt.toc();

			if(!getrusage(RUSAGE_SELF, &usage))
				std::cout << "\nMemory per transition: " << usage.ru_maxrss / (double)tf.get_no_transitions() << std::endl;
				
			std::cout << "Number of transitions: " << tf.get_no_transitions() << std::endl;

			// Parsing targets
			int num_targets = goal->targets.size();
			// std::vector<scots::StaticController> controllers;

			// for(int i = 0; i < num_targets; i++) {
			// 	controllers.push_back(getController(ss, is, tf, s_eta, goal->targets[i]));

			// 	// std::cout << "\n\nTarget Locked, starting to proceed." << std::endl;
			// 	// success = simulatePath(success, controllers[i], goal->targets[i]);
			// 	// success = reachTarget(success, controllers[i], goal->targets[i]);
			// }

			scots::StaticController controller = getController(ss, is, tf, s_eta, goal->targets[target_no]);

			std::cout << "\n\nTarget Locked, starting to proceed." << std::endl;
			success = simulatePath(success, controller, goal->targets[target_no]);
			success = reachTarget(success, controller, goal->targets[target_no]);

			if(success) {
				result_.target_id = 0;
				result_.synthesis_time = 0.0;
				result_.completion_time = 0.0;

				bool send_new_goal_success = send_new_goal_client.call(req, resp);

				std::cout << "Succeeded for: " << action_name_.c_str() << std::endl;
				// set the action state to succeeded
				as_.setSucceeded(result_);
			}
		}
};

int main(int argc, char** argv) {
	// ros node initialize
	ros::init(argc, argv, "scotsActionServer");

	ros::AsyncSpinner spinner(0);
    spinner.start();

	// Create an action server object and spin ROS
	scotsActionServer scotsAS("/scots");

	ros::waitForShutdown();
	// ros::spin();

	return 0;
}