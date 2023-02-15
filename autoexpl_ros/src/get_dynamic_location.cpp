#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <ctime>

using namespace std;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*input, cloud);

  geometry_msgs::Point32 prev_point;
  vector<geometry_msgs::Point32>dyn_obs[1][1];
  int iter = 0;
  // cout << sizeof(cloud.points) << endl;
  // Iterate through the point cloud and print the x, y, and z coordinates
  for (const auto& point : cloud.points)
  {
    if (point.z>0)
    {
      if((point.x==prev_point.x)&&(point.y==prev_point.y)){
        dyn_obs[iter][end+1]=point;
      }
      else{
        iter = iter+1;
      }
      prev_point=point;
      // cout << "Point: x = " << point.x << ", y = " << point.y << ", z = " << point.z << std::endl;
    }
  }
  // cout << obstacle_coordinates << endl;
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init(argc, argv, "get_dynamic_location");
  ros::NodeHandle nh;

  time_t tstart, tend; 
  tstart = time(0);

  // Create a ROS subscriber for the PointCloud2 topic
  ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_callback);
  ros::Publisher pub = nh.advertise<vector<sensor_msgs::PointCloud>>("dyn_obs", 1000);

  ros::Rate rate(1);


  while(ros::ok()){
    pub.publish(dyn_obs);
    rate.sleep();
  }

  // Spin and process ROS callbacks
  ros::spin ();

  tend = time(0);
  cout << "It took "<< difftime(tend, tstart) <<" second(s)."<< endl;

  return 0;
} 