#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  // Iterate through the point cloud and print the x, y, and z coordinates
  for (const auto& point : cloud.points)
  {
    std::cout << "Point: x = " << point.x << ", y = " << point.y << ", z = " << point.z << std::endl;
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "get_dynamic_location");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the PointCloud2 topic
  ros::Subscriber sub = nh.subscribe ("\velodyne_points", 1, cloud_callback);

  // Spin and process ROS callbacks
  ros::spin ();

  return 0;
}