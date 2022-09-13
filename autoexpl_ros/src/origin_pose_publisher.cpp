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

// ros service
#include <std_srvs/Empty.h>

// global variables for storing the pose
geometry_msgs::Point position, updated_position;
geometry_msgs::Quaternion orientation, updated_orientation;

// map metadata callback & origin publisher
void mapMetaDataCallback(const nav_msgs::MapMetaData::ConstPtr &msg) {
	updated_position = msg->origin.position;
	updated_orientation = msg->origin.orientation;
}

// on request update the origin pose
bool updateOriginPose(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
	position = updated_position;
	orientation = updated_orientation;

	ROS_INFO_STREAM("Origin Pose Updated..");
	return true;
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
	ros::Subscriber mapdata_sub = nh.subscribe("/map_metadata", 0, &mapMetaDataCallback);

	// Empty Service server
	ros::ServiceServer update_server = nh.advertiseService("/update_origin", &updateOriginPose);

	// loop rate for while loop (default ferq 1 hz)
	ros::Rate rate(broadcast_frequency);

	// update origin once when this node is started
	ros::Duration(2).sleep(); 			// wait for some time to get data in global variables
	ros::spinOnce();

	position = updated_position;
	orientation = updated_orientation;

	// Transform broadcaster
	static tf2_ros::StaticTransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	while(ros::ok()) {
		ros::spinOnce();

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