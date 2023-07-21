#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher blue;
ros::Publisher green;

void
cloud_blue (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);

  double down_rate;
  ros::param::get("down_rate", down_rate);
  sor.setLeafSize (0.1f, 0.02f, 1);
  sor.setLeafSize (down_rate, down_rate, down_rate);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  blue.publish (output);
  
}

void
cloud_green (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);

  double down_rate;
  ros::param::get("down_rate", down_rate);
  sor.setLeafSize (0.1f, 0.02f, 1.0f);
  sor.setLeafSize (down_rate, down_rate, down_rate);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  green.publish (output);
  
}
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "downsampling");
  ros::NodeHandle nh;

  // Set ROS param
  ros::param::set("down_rate", 0.25);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("blue", 1, cloud_blue);
  ros::Subscriber sub2 = nh.subscribe ("green", 1, cloud_green);

  // Create a ROS publisher for the output point cloud
  blue = nh.advertise<sensor_msgs::PointCloud2> ("point_blue", 1);
  green = nh.advertise<sensor_msgs::PointCloud2> ("point_green", 1);

  // Spin
  ros::spin ();
}
