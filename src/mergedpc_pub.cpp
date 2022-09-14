#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <sensor_msgs/PointCloud2.h>

using namespace std;

int main (int argc, char** argv){
  ros::init (argc, argv, "PointPub");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1000);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr total_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // string pcdfile_path0 = "/home/k/Desktop/data/Loam_livox_loop_20220908_test2/";

  std::string pcdfile_path0;
  nh.param<std::string>("pcdfile_path0",pcdfile_path0,"default_arg1");

  string pcdfile_path = "/home/haigejiujian/renov_ws/src/data/totalscan.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile_path, *cloud) == -1){
    PCL_INFO ("Couldn't read file test_pcd.pcd \n");
  }
  else{
    ROS_INFO("the pcd file is readed");
  }
  * total_cloud = * total_cloud + *cloud;
  total_cloud->resize(total_cloud->height * total_cloud->width);
  std::cout << "Loaded "
            << total_cloud->size()
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
  float leafSize = 0.01;
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
  voxel.setInputCloud(total_cloud);
  voxel.setLeafSize(leafSize, leafSize, leafSize);
  voxel.filter(*cloud_downsampled);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_zfilter_after (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;    
  pass.setInputCloud (cloud_downsampled);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-1, 0.2);
  pass.filter(*cloud_zfilter_after);


  // Convert the cloud to ROS message
  sensor_msgs::PointCloud2 output_pointcloud;
  pcl::toROSMsg(*cloud_zfilter_after, output_pointcloud);
  output_pointcloud.header.frame_id = "aubo_base_link";
  std::cout << "the size is: " << output_pointcloud.data.size() << std::endl;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    pcl_pub.publish(output_pointcloud);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();

  return (0);
}
