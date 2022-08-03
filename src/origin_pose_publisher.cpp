#include <iostream>
#include <string>

// ros includes
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/MapMetaData.h>


// Rigid topic callback function, will be called when there is new messages on the topic.
// It also publishes tf of rigid bodies
void mapMetaDataCallback(const nav_msgs::MapMetaData &msg) {
	// Transform broadcaster
	static tf2_ros::TransformBroadcaster br;

	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "origin";
		
	transformStamped.transform.translation.x = msg.origin.position.x;
	transformStamped.transform.translation.y = msg.origin.position.y;
	transformStamped.transform.translation.z = msg.origin.position.z;

	transformStamped.transform.rotation.x = msg.origin.orientation.x;
	transformStamped.transform.rotation.y = msg.origin.orientation.y;
	transformStamped.transform.rotation.z = msg.origin.orientation.z;
	transformStamped.transform.rotation.w = msg.origin.orientation.w;
	 
	// Publishing the tf
	br.sendTransform(transformStamped);
}

int main(int argc, char** argv) {
	// Initializing the phasespace tf visualization node
  ros::init(argc, argv, "origin_pose_publisher");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Origin Defined..");
  
  // Rigid and Camera Sunscriber handler
  ros::Subscriber mapdata_sub = nh.subscribe("/map_metadata", 0, mapMetaDataCallback);
  
  ros::spin();  
  return 0;
}