#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

using namespace std;

int main (int argc, char** argv){
  ros::init (argc, argv, "PointPub");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1000);

  pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  // string pcdfile_path0 = "/home/k/Desktop/data/Loam_livox_loop_20220908_test2/";

  std::string pcdfile_path0;
  nh.param<std::string>("/totalpc_pub/pcdfile_path0",pcdfile_path0,pcdfile_path0);
  int pcdfile_endnum;
  nh.param<int>("/totalpc_pub/pcdfile_endnum",pcdfile_endnum,pcdfile_endnum);
  double leafSize;
  nh.param<double>("/totalpc_pub/leafSize", leafSize, leafSize);


  // merge the point cloud into total point cloud 
  string pcdfile_path;
  for (int i=1; i<pcdfile_endnum; i++){
    string str_num = to_string(i);
    pcdfile_path = pcdfile_path0 + "/Loam_livox_loop/aft_mapp_"+str_num+".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdfile_path, *cloud) == -1){
      // PCL_INFO ("Couldn't read file test_pcd.pcd \n");
      cout<<"Couldn't read file test_pcd.pcd "<<endl;
      continue;
    }
    else{
      cout<<"the "<<str_num<<" pcd file is readed"<<endl;
      // ROS_INFO("the pcd file is readed");
    }
    * total_cloud = * total_cloud + *cloud;
  }
  total_cloud->resize(total_cloud->height * total_cloud->width);
  // std::cout << "Loaded "
  //           << total_cloud->size()
  //           << " data points from test_pcd.pcd with the following fields: "
  //           << std::endl;

  // down sample the total point cloud 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(total_cloud);
  voxel.setLeafSize(leafSize, leafSize, leafSize);
  voxel.filter(*cloud_downsampled);

  // Save the total point cloud
  string output_pcdfile_path = pcdfile_path0 + "/totalpcd.pcd";
	pcl::io::savePCDFile(output_pcdfile_path, *cloud_downsampled);

  // Convert the cloud to ROS message
  sensor_msgs::PointCloud2 output_pointcloud;
  pcl::toROSMsg(*cloud_downsampled, output_pointcloud);
  output_pointcloud.header.frame_id = "aubo_base_link";
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
