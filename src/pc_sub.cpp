#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
 
static size_t counter = 0;
 
void SubscribePointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_message) {
  pcl::PointCloud<pcl::PointXYZI> point_cloud;
  pcl::fromROSMsg(*lidar_message, point_cloud);
  std::string file_name = "point_cloud_" + std::to_string(counter) + ".pcd";
  counter++;
  pcl::io::savePCDFile(file_name, point_cloud);
}
 
int main(int argc, char **argv) {
  ros::init(argc, argv, "point_cloud_subscriber");
  ros::NodeHandle node_handle;
  ros::Subscriber point_cloud_sub = 
    node_handle.subscribe("/total_lidar_pointcloud", 1, SubscribePointCloud);
  ros::spin();
 
  return 0;
}