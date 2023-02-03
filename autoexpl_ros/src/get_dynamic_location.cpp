#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <algorithm>
#include <numeric>

using namespace std;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


tf::TransformListener *tf_listener; 

 void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    cout << temp_cloud << endl;
    //do stuff with temp_cloud here
}

int main(int argc, char **argv){

    ros::init(argc, argv, "get_dynamic_location");
    ros::NodeHandle nh;

    ros::Subscriber kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, cloud_cb);

    return 0;
}