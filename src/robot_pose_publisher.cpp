/*
 * File Name: robot_pose_publisher.cpp
 *
 * Author: Jay Bhagiya 
*/

// ros includes
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv)
{
	// initializes the ros and nodes
	ros::init(argc, argv, "robot_pose_publisher");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	// configuring parameters
	std::string map_frame, base_frame;
	double publish_frequency;
	ros::Publisher p_2d_pub;

	nh_priv.param<std::string>("map_frame", map_frame, "origin");
	nh_priv.param<std::string>("base_frame", base_frame, "base_footprint");
	nh_priv.param<double>("publish_frequency", publish_frequency, 10);

	p_2d_pub = nh.advertise<geometry_msgs::Pose2D>("/robot_pose", 1);

	// creates the listener;
	tf2_ros::Buffer tfbBuffer;
	tf2_ros::TransformListener listener(tfbBuffer);

	// loop rate for while loop
	ros::Rate rate(publish_frequency);

	while(nh.ok()) {
		geometry_msgs::TransformStamped transformStamped;

		try {
			transformStamped = tfbBuffer.lookupTransform(map_frame, base_frame, ros::Time(0));

			geometry_msgs::Vector3 position = transformStamped.transform.translation;
			geometry_msgs::Quaternion rotation = transformStamped.transform.rotation;

			geometry_msgs::Pose2D robot_pose;

			robot_pose.x = position.x;
			robot_pose.y = position.y;

			tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
			tf2::Matrix3x3 m(q);
			
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			robot_pose.theta = yaw;

			p_2d_pub.publish(robot_pose);
		}
		
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		rate.sleep();
	}

	return 0;
}