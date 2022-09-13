/*
 * File Name: scotsActionServer.cpp
 *
 * Author: Jay Bhagiya 
*/

// ros includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <autoexpl_msgs/AutoExplAction.h>
#include <autoexpl_msgs/Target.h>

// ros robot includes
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

// memory profiling
#include <sys/time.h>
#include <sys/resource.h>


class scotsActionServer
{
	protected:
		ros::NodeHandle nh_;
		
		// NodeHandle instance must be created before this line. Otherwise strange error occurs.
		actionlib::SimpleActionServer<autoexpl_msgs::AutoExplAction> as_;
		std::string action_name_;

		// create messages that are used to published feedback/result
		autoexpl_msgs::AutoExplFeedback feedback_;
		autoexpl_msgs::AutoExplResult result_;

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

		// void robotPoseCallback_1(const phasespace_msgs::Markers &msg) {
		// 	phasespace_msgs::Marker marker_dyn = msg.markers[0];
		// 	phasespace_msgs::Marker marker_ori = msg.markers[1];

		// 	curr_pose.x = marker_dyn.x;
		// 	curr_pose.y = marker_dyn.y;

		// 	double dy = marker_dyn.y - marker_ori.y;
		// 	double dx = marker_dyn.x - marker_ori.x;

		// 	double angle = std::atan2(dy, dx);
		// 	curr_pose.theta = angle;
		// }

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

		void visualizeTargets(const autoexpl_msgs::Target &tr) {
			// visaulization parameters
			visualization_msgs::Marker frontier, target;

			frontier.header.frame_id = target.header.frame_id = "origin";
			frontier.header.stamp = target.header.stamp = ros::Time::now();
			
			frontier.ns = "frontier_window";
			target.ns = "target_window";
			
			frontier.id = target.id = 0;
			frontier.type = target.type = visualization_msgs::Marker::POINTS;
			frontier.action = target.action = visualization_msgs::Marker::ADD;

			frontier.pose.orientation.w = target.pose.orientation.w = 1;

			frontier.scale.x = frontier.scale.y = 1 * tr.clearance;
			target.scale.x = target.scale.y = 1 * tr.window;

			frontier.color.g = 1.0f;
			target.color.r = 1.0f;

			frontier.color.a = 0.5;
			target.color.a = 1.0;

			frontier.lifetime = target.lifetime = ros::Duration();

			double diff = tr.window;
			std::vector<visualization_msgs::Marker> marker_obj = {frontier, target};

			for(int i = 0; i < 2; i++) {
				geometry_msgs::Point pt;
				
				pt.x = tr.points[0] + diff / 2.0;
				pt.y = tr.points[2] + diff / 2.0;

				marker_obj[i].points.push_back(pt);
				markers_pub.publish(marker_obj[i]);
			}

		}

		scots::WinningDomain getDomain(const scots::UniformGrid &ss, const scots::TransitionFunction &tf, const autoexpl_msgs::Target &tr) {
			
			// defining target set
			auto target = [&ss, &tr](const abs_type& idx) {
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
			std::cout << "\nWinning domain for target id " << tr.id << ", is " << win_domain.get_size() << std::endl;

			return win_domain;
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

		bool reachTarget(const scots::StaticController &controller, const autoexpl_msgs::Target &tr) {
			//defining dynamics of robot
			auto vehicle_post = [](state_type &x, const input_type &u) {
				auto rhs = [](state_type& xx, const state_type &x, const input_type &u) {
					xx[0] = u[0] * std::cos(x[2]); 
					xx[1] = u[0] * std::sin(x[2]);
					xx[2] = u[1];
				};
				scots::runge_kutta_fixed4(rhs, x, u, state_dim, tau, 10);
			};

			// auto  vehicle_post = [](state_type &x, const input_type &u) {
			//   /* the ode describing the vehicle */
			//   auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) {
			//     double alpha=std::atan(std::tan(u[1]) / 2.0);
			//     xx[0] = u[0] * std::cos(alpha + x[2]) / std::cos(alpha);
			//     xx[1] = u[0] * std::sin(alpha + x[2]) / std::cos(alpha);
			//     xx[2] = u[0] * std::tan(u[1]);
			//   };
			//   /* simulate (use 10 intermediate steps in the ode solver) */
			//   scots::runge_kutta_fixed4(rhs, x, u, state_dim, tau, 10);
			// };

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

			// success flag
			bool success = false;

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
					// std::cout << "Robot's Current Pose: " << robot_state[0] << ", " 
					// 									  << robot_state[1] << ", " 
					// 									  << robot_state[2] << std::endl;
					feedback_.curr_pose = curr_pose;

					std::vector<input_type> control_inputs = controller.peek_control<state_type, input_type>(robot_state);

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

		bool simulatePath(const scots::StaticController &controller, const autoexpl_msgs::Target &tr) {
			//defining dynamics of robot
			ROS_INFO_STREAM("Publishing the trajectory..");

			auto vehicle_post = [](state_type &x, const input_type &u) {
				auto rhs = [](state_type& xx, const state_type &x, const input_type &u) {
					xx[0] = u[0] * std::cos(x[2]); 
					xx[1] = u[0] * std::sin(x[2]);
					xx[2] = u[1];
				};
				scots::runge_kutta_fixed4(rhs, x, u, state_dim, tau, 10);
			};

			// auto  vehicle_post = [](state_type &x, const input_type &u) {
			//   /* the ode describing the vehicle */
			//   auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) {
			//     double alpha=std::atan(std::tan(u[1]) / 2.0);
			//     xx[0] = u[0] * std::cos(alpha + x[2]) / std::cos(alpha);
			//     xx[1] = u[0] * std::sin(alpha + x[2]) / std::cos(alpha);
			//     xx[2] = u[0] * std::tan(u[1]);
			//   };
			//   /* simulate (use 10 intermediate steps in the ode solver) */
			//   scots::runge_kutta_fixed4(rhs, x, u, state_dim, tau, 10);
			// };

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

			bool success = false;

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

		void processGoal_1(const autoexpl_msgs::AutoExplGoalConstPtr &goal) {
			bool success = false;
			// Parsing targets
			int num_targets = goal->targets.size();
			// std::vector<scots::StaticController> controllers;

			// for(int i = 0; i < num_targets; i++) {
			// 	controllers.push_back(getDomain(ss, s_eta, goal->targets[i]));
			// }

			// scots::StaticController controller = getDomain(ss, is, tf, s_eta, goal->targets[1]);
			scots::StaticController controller; 
			if(!read_from_file(controller, "AutoExpl")) {
				std::cout << "Could not able read for file." << std::endl;
				success = false;
			}

			std::cout << "\n\nTarget Locked, starting to proceed." << std::endl;
			success = simulatePath(controller, goal->targets[0]);
			success = reachTarget(controller, goal->targets[0]);

			if(success) {
				result_.target_id = 0;
				result_.synthesis_time = 0.0;
				result_.completion_time = 0.0;

				std::cout << "Succeeded for: " << action_name_.c_str() << std::endl;
				// set the action state to succeeded
				as_.setSucceeded(result_);
			}
		}
		
		void processGoal(const autoexpl_msgs::AutoExplGoalConstPtr &goal) {

			ros::Time t_begin = ros::Time::now();

			struct rusage usage;

		 	auto vehicle_post = [](state_type &x, const input_type &u) {
				auto rhs = [](state_type& xx, const state_type &x, const input_type &u) {
					xx[0] = u[0] * std::cos(x[2]); 
					xx[1] = u[0] * std::sin(x[2]);
					xx[2] = u[1];
				};
				scots::runge_kutta_fixed4(rhs, x, u, state_dim, tau, 10);
			};

			/* we integrate the vehicle ode by tau sec (the result is stored in x)  */
			// auto  vehicle_post = [](state_type &x, const input_type &u) {
			//   /* the ode describing the vehicle */
			//   auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) {
			// 	double alpha=std::atan(std::tan(u[1]) / 2.0);
			// 	xx[0] = u[0] * std::cos(alpha + x[2]) / std::cos(alpha);
			// 	xx[1] = u[0] * std::sin(alpha + x[2]) / std::cos(alpha);
			// 	xx[2] = u[0] * std::tan(u[1]);
			//   };
			//   /* simulate (use 10 intermediate steps in the ode solver) */
			//   scots::runge_kutta_fixed4(rhs, x, u, state_dim, tau, 10);
			// };

			// /* we integrate the growth bound by 0.3 sec (the result is stored in r)  */
			// auto radius_post = [](state_type &r, const state_type &, const input_type &u) {
			// 	const state_type w = {{0.01, 0.01}};
			//   	double c = std::abs(u[0]) * std::sqrt(std::tan(u[1]) * std::tan(u[1]) / 4.0+1);
			//   	r[0] = r[0] + c * r[2] * tau + w[0];
			//   	r[1] = r[1] + c * r[2] * tau + w[1];
			// };

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
			input_type i_ub={{ 0.22,  0.11}};
			input_type i_eta={{.02, .01}};
			  
			scots::UniformGrid is(input_dim, i_lb, i_ub, i_eta);
			std::cout << std::endl;	
			is.print_info();

			// update the origin
			bool origin_update_success = origin_update_client.call(req, resp);

			// success flag
			bool success = false;

			std::vector<std::vector<int>> maps = getMapMatrix(map_vector, width, height);

			visualizeObstacles(ss, maps);
			visualizeTargets(goal->targets[0]);

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

			ros::Time s_begin = ros::Time::now();
			
			tt.tic();
			abs.compute_gb(tf, vehicle_post, radius_post, avoid);
			tt.toc();

			if(!getrusage(RUSAGE_SELF, &usage))
				std::cout << "\nMemory per transition: " << usage.ru_maxrss / (double)tf.get_no_transitions() << std::endl;
				
			std::cout << "Number of transitions: " << tf.get_no_transitions() << std::endl;

			// Parsing targets
			abs_type total_domain = ss.size();
			
			int target_no = 0;
			int num_targets = goal->targets.size();
			std::vector<scots::WinningDomain> domains;

			for(int i = 0; i < num_targets; i++) {
				visualizeTargets(goal->targets[i]);
				scots::WinningDomain win_domain = getDomain(ss, tf, goal->targets[i]);

				if(0.03 * total_domain < win_domain.get_size()) {
					domains.push_back(win_domain);
					target_no = i;
					break;
				}
				else {
					ROS_INFO_STREAM("Winning domain is less than 3% of total domain, going for the next target.");
				}
			}

			ros::Duration synthesis_time = ros::Time::now() - s_begin;

			if(domains.size() > 0) {
				scots::StaticController controller = scots::StaticController(ss, is, std::move(domains[0]));

				std::cout << "Writing to the file." << std::endl;
				if(write_to_file(controller, "AutoExpl"))
					std::cout << "Done.\n";

				std::cout << "\n\nRobot started, Reaching to the target." << std::endl;
				success = simulatePath(controller, goal->targets[target_no]);
				success = reachTarget(controller, goal->targets[target_no]);
			}
			else {
				ROS_INFO_STREAM("No reachable targets, Exploration is Done..");
			}

			ros::Duration completion_time = ros::Time::now() - t_begin - ros::Duration(10);

			if(success) {
				result_.target_id = target_no;
				result_.synthesis_time = synthesis_time.toSec();
				result_.completion_time = completion_time.toSec();

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