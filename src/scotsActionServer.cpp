/*
 * File Name: scotsActionServer.cpp
 *
 * Author: Jay Bhagiya 
*/

// ros includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <autonomous_exploration/autoExplAction.h>

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
		actionlib::SimpleActionServer<autonomous_exploration::autoExpl> as_;
		std::string action_name_;

		// create messages that are used to published feedback/result
		autonomous_exploration::autoExplFeedback feedback_;
		autonomous_exploration::autoExplResult result_;

		// for time profiling
		TicToc tt;
	
	public:
		scotsActionServer(std::string name) : 
		// Bind the callback to the action server. False is for thread spinning
		as_(nh_, name, boost::bind(&autoExplAction::processGoal, this, _1), false),
		action_name_(name) {
			as_.start();

			ROS_INFO("Scots Action Server is started, now you can send the goals.");

			struct rusage usage;

			const int state_dim = 3;
			const int input_dim = 2;
			const double tau = 2.1;

			using state_type = std::array<double, state_dim>;
			using input_type = std::array<double, input_dim>;
			using abs_type = scots::abs_type;

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
		}

		~scotsActionServer(void)
		{
		}

		double get_points_in_line(const Eigen::Vector2d &pt1, const Eigen::Vector2d &pt2, const Eigen::Vector2d &query_point) {
			return ((query_point.y() - pt1.y()) * (pt2.x() - pt1.x())) - ((query_point.x() - pt1.x()) * (pt2.y() - pt1.y()));
		}
		
		void processGoal(const simple_action_example::autoExplGoalConstPtr &goal) {
			
			state_type s_lb={{0, 0, -3.5}};
			state_type s_ub={{4.5, 4.5, 3.5}};
			state_type s_eta={{.1, .1, .2}};

			scots::UniformGrid ss(state_dim, s_lb, s_ub, s_eta);
			ROS_INFO("State Space Grid Info (x, y, theta), (%d, %d, %d) -> (%d, %d, %d), with grid points, (%d, %d, %d)", 
				s_lb[0], s_lb[1], s_lb[2], s_ub[0], s_ub[1], s_ub[2], s_eta[0], s_eta[1], s_eta[2]);
			// ss.print_info();
			
			input_type i_lb={{-0.22, -0.1}};
			input_type i_ub={{ 0.22, 0.1}};
			input_type i_eta={{.02, .01}};
			  
			scots::UniformGrid is(input_dim, i_lb, i_ub, i_eta);
			ROS_INFO("Input Space Grid Info (linear, angular), (%d, %d) -> (%d, %d), with grid points, (%d, %d)", 
				i_lb[0], i_lb[1], i_ub[0], i_ub[1], i_eta[0], i_eta[1]);
			// is.print_info();

			int num_targets = goal->targets.size();
			int num_obs = goal->obstacles.size();

			std::vector<std::vector<Eigen::Vector2d>> vertices;

			for(int i = 0; i < num_obs; i++) {
				// For storing obstacle instance
				int num_points = goal->obstacles[i].size();
				std::vector<Eigen::Vector2d> inner_obs;

				for(int j = 0; j < num_points; j++) {
					Eigen::Vector2d point{goal->obstacles[i].points[j].x, goal->obstacles[i].points[j].y};
					inner_obs.push_back(point);
				}
				vertices.push_back(inner_obs)
			}

			auto avoid = [&vertices, ss, s_eta, num_obs](const abs_type& idx) {
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

						const auto point_in_line = get_points_in_line(
							vertices[i][j], vertices[i][(j + 1) % num_sides_of_polygon], query_point);

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
    			ROS_INFO("Memory per transition: %d", usage.ru_maxrss / (double)tf.get_no_transitions());
  				
  			ROS_INFO("Number of transitions: " << tf.get_no_transitions());
		}
};