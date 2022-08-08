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
#include <visualization_msgs/Marker.h>

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
		ros::Subscriber robot_pose = nh_.subscribe(pose_topic_name_, 1, &scotsActionServer::robotPoseCallback_2, this);

		std::string vel_topic_name_ = "/cmd_vel";
		ros::Publisher robot_vel = nh_.advertise<geometry_msgs::Twist>(vel_topic_name_, 10);

		// map subscriber
		std::string map_topic_name = "/map";
		ros::Subscriber map_sub = nh_.subscribe(map_topic_name, 1, &scotsActionServer::mapCallback, this);

		// obstacle visualization
		std::string vis_topic_name = "/scots_visualization";
		ros::Publisher markers_pub = nh_.advertise<visualization_msgs::Marker>(vis_topic_name, 10);

		// global variables
		static const int state_dim = 3;
		static const int input_dim = 2;
		static constexpr double tau = 2.1;

		using state_type = std::array<double, state_dim>;
		using input_type = std::array<double, input_dim>;
		using abs_type = scots::abs_type;

		// ros loop rate
		// ros::Rate r(10);

		// for time profiling
		TicToc tt;
	
	public:
		scotsActionServer(std::string name) : 
		// Bind the callback to the action server. False is for thread spinning
		as_(nh_, name, boost::bind(&scotsActionServer::processGoal, this, _1), false),
		action_name_(name) {
			as_.start();

			std::cout << "Scots Action Server is started, now you can send the goals." << std::endl;
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
			resolution = msg.info.resolution;
			width = msg.info.width;
			height = msg.info.height;

			// map_vector = msg.data;
			for(int i = 0; i < width * height; i++) {
				map_vector.push_back(msg.data[i]);
			}
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

		scots::StaticController getController(const scots::UniformGrid &ss, const scots::UniformGrid &is, const scots::TransitionFunction &tf, 
			const state_type &s_eta, const autonomous_exploration::Target &tr) {
			
			// defining target set
			auto target = [&ss, &s_eta, &tr](const abs_type& idx) {
				state_type x;
				ss.itox(idx, x);
				// function returns 1 if cell associated with x is in target set 
				if (tr.points[0] <= (x[0] - s_eta[0] / 2.0) && (x[0] + s_eta[0] / 2.0) <= tr.points[1] && 
					tr.points[2] <= (x[1] - s_eta[1] / 2.0) && (x[1] + s_eta[1] / 2.0) <= tr.points[3])
				  return true;
				return false;
			};

			std::cout << "\nSynthesis for target, " << tr.id << std::endl;
			tt.tic();
			scots::WinningDomain win_domain = scots::solve_reachability_game(tf, target);
			tt.toc();
			std::cout << "\nWinning domain for targer id " << tr.id << ", is " << win_domain.get_size() << std::endl;

			return scots::StaticController(ss, is, std::move(win_domain));
		}

		bool reachTarget(bool &success, const scots::StaticController &controller, const autonomous_exploration::Target &tr) {
			// defining target set
			auto target = [&tr](const state_type& x) {
				// function returns 1 if cell associated with x is in target set 
				if (tr.points[0] <= x[0] && x[0] <= tr.points[1] && tr.points[2] <= x[1] && x[1] <= tr.points[3])
				  return true;
				return false;
			};

			state_type robot_state = {curr_pose.x, curr_pose.y, curr_pose.theta};

			ros::Duration(1).sleep();
			geometry_msgs::Twist vel_msg_turtle;

			ros::Rate rate(10);

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

					std::vector<input_type> control_inputs = controller.get_control<state_type, input_type>(robot_state);

					vel_msg_turtle.linear.x = control_inputs[0][0];
					vel_msg_turtle.angular.z = control_inputs[0][1];

					// pusblishing the current feedback to action client
					as_.publishFeedback(feedback_);
				}
				else {
					vel_msg_turtle.linear.x = 0.0;
					vel_msg_turtle.angular.z = 0.0;

					robot_vel.publish(vel_msg_turtle);
					success = true;
					break;
				}

				// robot_vel.publish(vel_msg_turtle);
				// rate.sleep();

				// this is to maintain, that robot will receive same speed for tau time.
				ros::Time beginTime = ros::Time::now();
				ros::Duration secondsIWantToSendMessagesFor = ros::Duration(2.1);
				ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;

				while(ros::Time::now() < endTime )
				{
					robot_vel.publish(vel_msg_turtle);

					// Time between messages, so you don't blast out an thousands of
					// messages in your 3 secondperiod
					ros::Duration(0.1).sleep();
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

			ros::Duration(1).sleep();

			// visaulization parameters
			visualization_msgs::Marker lines;

			lines.header.frame_id = "origin";
			lines.header.stamp = ros::Time::now();
			
			lines.ns = "lines";
			lines.id = 0;
			lines.type = visualization_msgs::Marker::LINE_STRIP;
			lines.action = visualization_msgs::Marker::ADD;

			lines.scale.x = 0.1;

			lines.color.g = 1.0f;
			lines.color.a = 1.0;

			lines.lifetime = ros::Duration();

			geometry_msgs::Point pts;

			pts.x = robot_state[0];
			pts.y = robot_state[1];
			lines.points.push_back(pts);

			while(ros::ok()) {
				// getting ready feedback handler
				std::cout << "Robot's Current Pose: " << robot_state[0] << ", " 
													  << robot_state[1] << ", " 
													  << robot_state[2] << std::endl;

				std::vector<input_type> control_inputs = controller.get_control<state_type, input_type>(robot_state);

				vehicle_post(robot_state, control_inputs[0]);

				pts.x = robot_state[0];
				pts.y = robot_state[1];

				lines.points.push_back(pts);
				markers_pub.publish(lines);

				if(target(robot_state)) {
					std::cout << "Reached: " << robot_state[0] << ", " 
											 << robot_state[1] << ", " 
											 << robot_state[2] << std::endl;

					success = true;
					break;
				}
			}

			return success;
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
				r[0] = r[0] + r[2] * std::abs(u[0]) * tau;
				r[1] = r[1] + r[2] * std::abs(u[0]) * tau;
			};
			
			state_type s_lb={{2, 6, -3.5}};
			state_type s_ub={{12, 16, 3.5}};
			state_type s_eta={{.1, .1, .2}};

			scots::UniformGrid ss(state_dim, s_lb, s_ub, s_eta);
			std::cout << std::endl;
			ss.print_info();
			
			input_type i_lb={{-0.22, -0.1}};
			input_type i_ub={{ 0.22, 0.1}};
			input_type i_eta={{.02, .01}};
			  
			scots::UniformGrid is(input_dim, i_lb, i_ub, i_eta);
			std::cout << std::endl;	
			is.print_info();

			// visaulization parameters
			visualization_msgs::Marker points;

			points.header.frame_id = "origin";
			points.header.stamp = ros::Time::now();
			
			points.ns = "obstacles";
			points.id = 0;
			points.type = visualization_msgs::Marker::POINTS;
			points.action = visualization_msgs::Marker::ADD;

			points.scale.x = 0.1;
			points.scale.y = 0.1;

			points.color.r = 1.0f;
			points.color.g = 1.0f;
			points.color.a = 0.7;

			points.lifetime = ros::Duration();

			// result parameter
			bool success = true;

			std::vector<std::vector<int>> maps = getMapMatrix(map_vector, width, height);

			auto avoid = [&maps, &points, ss, s_eta, markers_pub=markers_pub, resolution=resolution](const abs_type& idx) {
				state_type x;
				ss.itox(idx, x);

				std::vector<int> grid_ratio {int(s_eta[0] / resolution), int(s_eta[1] / resolution)};

				std::vector<int> cord {int(x[0] / resolution), int(x[1] / resolution)};

				// std::cout <<  "idx:" << idx;
				// std::cout << "(" << x[0] << ", " << x[1] <<  ")";
 
				geometry_msgs::Point pt;

				for(int i = 0; i < grid_ratio[1]; i++) {
					for(int j = 0; j < grid_ratio[0]; j++) {
						if(maps[cord[1] + i][cord[0] + j] != 0){
							// pt.x = x[0] + 0.05;
							// pt.y = x[1] + 0.05;
							// points.points.push_back(pt);
							// markers_pub.publish(points);
							return true;
						}
					}
				}
				return false;
			};

			// Parsing obstacles
			// int num_obs = goal->obstacles.size();

			// std::vector<std::vector<Eigen::Vector2d>> vertices;

			// for(int i = 0; i < num_obs; i++) {
			// 	// For storing obstacle instance
			// 	int num_points = goal->obstacles[i].points.size();
			// 	std::vector<Eigen::Vector2d> inner_obs;

			// 	for(int j = 0; j < num_points; j++) {
			// 		Eigen::Vector2d point{goal->obstacles[i].points[j].x, goal->obstacles[i].points[j].y};
			// 		inner_obs.push_back(point);
			// 	}
			// 	vertices.push_back(inner_obs);
			// }

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
			// 	controllers.push_back(getController(ss, s_eta, goal->targets[i]));
			// }

			scots::StaticController controller = getController(ss, is, tf, s_eta, goal->targets[1]);

			std::cout << "\n\nTarget Locked, starting to proceed." << std::endl;
			success = reachTarget(success, controller, goal->targets[1]);
			// success = simulatePath(success, controller, goal->targets[1]);

			if(success) {
				result_.target_id = 0;
				result_.synthesis_time = 0.0;
				result_.completion_time = 0.0;

				std::cout << "Succeeded for: " << action_name_.c_str() << std::endl;
				// set the action state to succeeded
				as_.setSucceeded(result_);
			}
		}
};

int main(int argc, char** argv) {
	// ros node initialize
	ros::init(argc, argv, "scotsActionServer");

	// Create an action server object and spin ROS
	scotsActionServer scotsAS("/scots");
	ros::spin();

	return 0;
}