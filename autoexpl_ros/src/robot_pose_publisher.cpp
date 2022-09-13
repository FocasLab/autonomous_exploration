/*
 * File Name: robot_pose_publisher.cpp
 *
 * Author: Jay Bhagiya 
*/

// ros includes
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
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

	// config parameter defined
	std::string origin_frame, base_frame;
	double publish_frequency;
	bool is_stamped;
	ros::Publisher p_pub;

	// config parameters
	nh_priv.param<std::string>("origin_frame", origin_frame, "origin");
	nh_priv.param<std::string>("base_frame", base_frame, "base_footprint");
	nh_priv.param<double>("publish_frequency", publish_frequency, 10);
	nh_priv.param<bool>("is_stamped", is_stamped, false);

	// robot pose publisher handle
	if(is_stamped)
		p_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 10);
	else
		p_pub = nh.advertise<geometry_msgs::Pose2D>("/robot_pose", 10);

	// creates the listener;
	tf2_ros::Buffer tfbBuffer;
	tf2_ros::TransformListener listener(tfbBuffer);

	geometry_msgs::Vector3 position;
	geometry_msgs::Quaternion orientation;

	// loop rate for while loop (default freq 10 hz)
	ros::Rate rate(publish_frequency);

	while(nh.ok()) {
		geometry_msgs::TransformStamped transformStamped;

		try {
			transformStamped = tfbBuffer.lookupTransform(origin_frame, base_frame, ros::Time(0));

			position = transformStamped.transform.translation;
			orientation = transformStamped.transform.rotation;

			// storing pose stamped data
			geometry_msgs::PoseStamped pose_stamped;

			// frame and time
			pose_stamped.header.frame_id = origin_frame;
			pose_stamped.header.stamp = ros::Time::now();

			// position
			pose_stamped.pose.position.x = position.x;
			pose_stamped.pose.position.y = position.y;
			pose_stamped.pose.position.z = position.z;

			// orientation
			pose_stamped.pose.orientation = orientation;

			// storing simple robot pose data
			geometry_msgs::Pose2D robot_pose;
			
			tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
			tf2::Matrix3x3 m(q);
			
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			robot_pose.x = position.x;
			robot_pose.y = position.y;
			robot_pose.theta = yaw;

			if(is_stamped)
				p_pub.publish(pose_stamped);
			else
				p_pub.publish(robot_pose);
		}
		
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s", ex.what());
			ros::Duration(1.0).sleep();
		}

		rate.sleep();
	}

	return 0;
}