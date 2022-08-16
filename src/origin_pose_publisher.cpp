/*
 * File Name: origin_pose_publisher.cpp
 *
 * Author: Jay Bhagiya 
*/

#include <iostream>
#include <string>

// ros includes
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/MapMetaData.h>

// global variables for storing the pose
geometry_msgs::Point position;
geometry_msgs::Quaternion orientation;

// map metadata callback & origin publisher
void mapMetaDataCallback(const nav_msgs::MapMetaData::ConstPtr &msg) {
	position = msg->origin.position;
	orientation = msg->origin.orientation;
}

int main(int argc, char** argv) {
	// Initializing the origin pose publisher node
	ros::init(argc, argv, "origin_pose_publisher");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	// parameter defined
	std::string map_frame, child_frame;
	double broadcast_frequency;

	// config parameters
	nh_priv.param<std::string>("map_frame", map_frame, "map");
	nh_priv.param<std::string>("child_frame", child_frame, "origin");
	nh_priv.param<double>("origin_broadcast_freq", broadcast_frequency, 1);

	ROS_INFO_STREAM("Origin Defined..");
  
	// Map metadata subscriber
	ros::Subscriber mapdata_sub = nh.subscribe("/map_metadata", 0, mapMetaDataCallback);

	// loop rate for while loop (default ferq 1 hz)
	ros::Rate rate(broadcast_frequency);

	// Transform broadcaster
	static tf2_ros::StaticTransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	ros::Duration(2.0).sleep();
	ros::spinOnce();

	while(ros::ok()) {
		// ros::spinOnce();

		// storing the data
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = map_frame;
		transformStamped.child_frame_id = child_frame;
			
		transformStamped.transform.translation.x = position.x;
		transformStamped.transform.translation.y = position.y;
		transformStamped.transform.translation.z = position.z;

		transformStamped.transform.rotation.x = orientation.x;
		transformStamped.transform.rotation.y = orientation.y;
		transformStamped.transform.rotation.z = orientation.z;
		transformStamped.transform.rotation.w = orientation.w;
		 
		// Broadcastin the tf
		br.sendTransform(transformStamped);

		rate.sleep();
	}
	return 0;
}