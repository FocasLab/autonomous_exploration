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
#include <turtlesim/Pose.h>

// scots includes 
#include "scots.hpp"
#include "RungeKutta4.hpp"
#include "TicToc.hpp"

// stl includes
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

		// robot state 
		geometry_msgs::Pose2D curr_pose;

		// publisher and subscriber handlers
		std::string pose_topic_name_ = "/turtle1/pose";
		ros::Subscriber robot_pose = nh_.subscribe(pose_topic_name_, 1, &scotsActionServer::robotPoseCallback_2, this);

		std::string vel_topic_name_ = "/turtle1/cmd_vel";
		ros::Publisher robot_vel = nh_.advertise<geometry_msgs::Twist>(vel_topic_name_, 1);

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

			ROS_INFO("Scots Action Server is started, now you can send the goals.");
		}

		~scotsActionServer(void)
		{
		}

		double get_points_in_line(const Eigen::Vector2d &pt1, const Eigen::Vector2d &pt2, const Eigen::Vector2d &query_point) {
			return ((query_point.y() - pt1.y()) * (pt2.x() - pt1.x())) - ((query_point.x() - pt1.x()) * (pt2.y() - pt1.y()));
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

		void robotPoseCallback_2(const turtlesim::Pose &msg){
			curr_pose.x = msg.x;
			curr_pose.y = msg.y;
			curr_pose.theta = msg.theta;
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

			ROS_INFO("Synthesis for target, %d", tr.id);
			tt.tic();
			scots::WinningDomain win_domain = scots::solve_reachability_game(tf, target);
			tt.toc();
			ROS_INFO("Winning domain for targer id %d, is %d", tr.id, win_domain.get_size());

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

			while(ros::ok()) {
				
				if(as_.isPreemptRequested()) {
					ROS_INFO("%s: Preempted", action_name_.c_str());
					// set the action state to preempted
					as_.setPreempted();
					success = false;
					break;
				}

				robot_state = {curr_pose.x, curr_pose.y, curr_pose.theta};

				if(!(target(robot_state))) {
					// getting ready feedback handler
					ROS_INFO("Robot's Current Pose, (%f, %f, %f)", robot_state[0], robot_state[1], robot_state[2]);
					feedback_.curr_pose = curr_pose;

					std::vector<input_type> control_inputs = controller.peek_control<state_type, input_type>(robot_state);

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

				// this is to maintain, that robot will receive same speed for tau time.
				ros::Time beginTime = ros::Time::now();
				ros::Duration secondsIWantToSendMessagesFor = ros::Duration(tau);
				ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;

				while(ros::Time::now() < endTime )
				{
					robot_vel.publish(vel_msg_turtle);

					// Time between messages, so you don't blast out an thousands of
					// messages in your 3 secondperiod
					ros::Duration(0.1).sleep();
				}

				// r.sleep();
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
			
			state_type s_lb={{0, 0, -3.5}};
			state_type s_ub={{4.5, 4.5, 3.5}};
			state_type s_eta={{.1, .1, .2}};

			scots::UniformGrid ss(state_dim, s_lb, s_ub, s_eta);
			ROS_INFO("State Space Grid Info (x, y, theta), (%f, %f, %f) -> (%f, %f, %f), with grid points, (%f, %f, %f)", 
				s_lb[0], s_lb[1], s_lb[2], s_ub[0], s_ub[1], s_ub[2], s_eta[0], s_eta[1], s_eta[2]);
			// ss.print_info();
			
			input_type i_lb={{-0.22, -0.1}};
			input_type i_ub={{ 0.22, 0.1}};
			input_type i_eta={{.02, .01}};
			  
			scots::UniformGrid is(input_dim, i_lb, i_ub, i_eta);
			ROS_INFO("Input Space Grid Info (linear, angular), (%f, %f) -> (%f, %f), with grid points, (%f, %f)", 
				i_lb[0], i_lb[1], i_ub[0], i_ub[1], i_eta[0], i_eta[1]);
			// is.print_info();

			// result parameter
			bool success = true;

			// Parsing obstacles
			int num_obs = goal->obstacles.size();

			std::vector<std::vector<Eigen::Vector2d>> vertices;

			for(int i = 0; i < num_obs; i++) {
				// For storing obstacle instance
				int num_points = goal->obstacles[i].points.size();
				std::vector<Eigen::Vector2d> inner_obs;

				for(int j = 0; j < num_points; j++) {
					Eigen::Vector2d point{goal->obstacles[i].points[j].x, goal->obstacles[i].points[j].y};
					inner_obs.push_back(point);
				}
				vertices.push_back(inner_obs);
			}

			auto avoid = [this, &vertices, ss, s_eta, num_obs](const abs_type& idx) {
				state_type x;
				ss.itox(idx, x);

				double c1 = s_eta[0] / 2.0+1e-10;
				double c2 = s_eta[1] / 2.0+1e-10;

				Eigen::Vector2d query_point{x[0], x[1]};

				for(size_t i = 0; i < num_obs; i++) {
					int wn = 0;
					const int num_sides_of_polygon = vertices[i].size();

					// Iterate over each side.
					for (size_t j = 0; j < num_sides_of_polygon; ++j) { 

						double point_in_line = get_points_in_line(vertices[i][j], 
							vertices[i][(j + 1) % num_sides_of_polygon], query_point);

						// Check if the point lies on the polygon.
						if (point_in_line == 0) {
							return true;
						}

						if (vertices[i][j].y() <= query_point.y()) {
							// Upward crossing.
							if (vertices[i][(j + 1) % num_sides_of_polygon].y() >
								query_point.y()) {
								if (point_in_line > 0) {
									++wn;  // query point is left of edge
								}
							}
						} 
						else {
							// Downward crossing.
							if (vertices[i][(j + 1) % num_sides_of_polygon].y() <
								query_point.y()) {
								if (point_in_line < 0) {
									--wn;  // query point is right of edge
								}
							}
						}
					}
					return (wn != 0) ? true : false;  // Point is inside polygon only if wn != 0
				}
				return false;
			};

			ROS_INFO("Computing the transition function.");
  
			/* transition function of symbolic model */
			scots::TransitionFunction tf;
			scots::Abstraction<state_type,input_type> abs(ss, is);

			tt.tic();
			abs.compute_gb(tf, vehicle_post, radius_post, avoid);
			tt.toc();

			if(!getrusage(RUSAGE_SELF, &usage))
				ROS_INFO("Memory per transition: %lf", usage.ru_maxrss / (double)tf.get_no_transitions());
				
			ROS_INFO("Number of transitions: %ld", tf.get_no_transitions());

			// Parsing targets
			int num_targets = goal->targets.size();
			// std::vector<scots::StaticController> controllers;

			// for(int i = 0; i < num_targets; i++) {
			// 	controllers.push_back(getController(ss, s_eta, goal->targets[i]));
			// }

			scots::StaticController controller = getController(ss, is, tf, s_eta, goal->targets[0]);

			ROS_INFO("Target Locked, starting to proceed.");
			success = reachTarget(success, controller, goal->targets[0]);

			if(success) {
				result_.target_id = 0;
				result_.synthesis_time = 0.0;
				result_.completion_time = 0.0;

				ROS_INFO("%s: Succeeded", action_name_.c_str());
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