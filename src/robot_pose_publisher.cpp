/*
 * File Name: robot_pose_publisher.cpp
 *
 * Author: Jay Bhagiya 
*/

// ros includes
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
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
	bool is_stamped;
	ros::Publisher p_pub;

	nh_priv.param<std::string>("map_frame", map_frame, "origin");
	nh_priv.param<std::string>("base_frame", base_frame, "base_link");
	nh_priv.param<double>("publish_frequency", publish_frequency, 10);
	nh_priv.param<bool>("is_stamped", is_stamped, false);

	if(is_stamped)
		p_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);
	else
		p_pub = nh.advertise<geometry_msgs::Pose>("/robot_pose", 1);

	// creates the listener;
	tf2_ros::Buffer tfbBuffer;
	tf2_ros::TransformListener listener(tfbBuffer);

	// loop rate for while loop
	ros::Rate rate(publish_frequency);

	while(nh.ok()) {
		geometry_msgs::TransformStamped transformStamped;

		try {
			transformStamped = tfbBuffer.lookupTransform(map_frame, base_frame, ros::Time(0), ros::Duration(1.0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		// construct a pose message
		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header.frame_id = map_frame;
		pose_stamped.header.stamp = ros::Time::now();

		pose_stamped.pose.orientation.x = transformStamped.transform.rotation.x;
		pose_stamped.pose.orientation.y = transformStamped.transform.rotation.y;
		pose_stamped.pose.orientation.z = transformStamped.transform.rotation.z;
		pose_stamped.pose.orientation.w = transformStamped.transform.rotation.w;

		pose_stamped.pose.position.x = transformStamped.transform.translation.x;
		pose_stamped.pose.position.y = transformStamped.transform.translation.y;
		pose_stamped.pose.position.z = transformStamped.transform.translation.z;

		if(is_stamped)
			p_pub.publish(pose_stamped);
		else
			p_pub.publish(pose_stamped.pose);

		rate.sleep();
	}

	return 0;
}