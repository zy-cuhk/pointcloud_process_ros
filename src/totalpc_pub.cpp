#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

int main (int argc, char** argv){
  ros::init (argc, argv, "PointPub");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1000);

  pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  string pcdfile_path0 = "/home/k/Desktop/data/Loam_livox_loop_20220908_test2/";
  string pcdfile_path;
  int totalpcdfile_num=1000;
  for (int i=1; i<totalpcdfile_num; i++){
    string str_num = to_string(i);
    pcdfile_path = pcdfile_path0 + "aft_mapp_"+str_num+".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdfile_path, *cloud) == -1){
      PCL_INFO ("Couldn't read file test_pcd.pcd \n");
      continue;
    }
    else{
      ROS_INFO("the pcd file is readed");
    }
    * total_cloud = * total_cloud + *cloud;
  }
  total_cloud->resize(total_cloud->height * total_cloud->width);
  std::cout << "Loaded "
            << total_cloud->size()
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  string output_pcdfile_path = pcdfile_path0 + "totalpcd.pcd";
	pcl::io::savePCDFile(output_pcdfile_path, *total_cloud);

  // Convert the cloud to ROS message
  sensor_msgs::PointCloud2 output_pointcloud;
  pcl::toROSMsg(*total_cloud, output_pointcloud);
  output_pointcloud.header.frame_id = "map";
  std::cout << "the size is: " << output_pointcloud.data.size() << std::endl;
  ros::Rate loop_rate(10);
  while (ros::ok()){
    pcl_pub.publish(output_pointcloud);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  ros::shutdown();
  return (0);
}
