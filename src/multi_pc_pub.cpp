#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

int main (int argc, char** argv)
{
    ros::init (argc,argv,"pub_pcb");
    ros::NodeHandle nh;
    ros::Publisher pub1; 
    pub1 = nh.advertise<sensor_msgs::PointCloud2>("pcbpub1",1); 
    sensor_msgs::PointCloud2 cloud1;
    pcl::io::loadPCDFile("/home/k/catkin_ws/src/pointcloud_process_pcl/data4/0_1.pcd", cloud1);
    cloud1.header.stamp=ros::Time::now();
    cloud1.header.frame_id="/odom"; 

    ros::Publisher pub2; 
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("pcbpub2",1); 
    sensor_msgs::PointCloud2 cloud2;
    pcl::io::loadPCDFile("/home/k/catkin_ws/src/pointcloud_process_pcl/data4/1_1.pcd", cloud2);
    cloud2.header.stamp=ros::Time::now();
    cloud2.header.frame_id="/odom"; 

    ros::Publisher pub3; 
    pub3 = nh.advertise<sensor_msgs::PointCloud2>("pcbpub3",1); 
    sensor_msgs::PointCloud2 cloud3;
    pcl::io::loadPCDFile("/home/k/catkin_ws/src/pointcloud_process_pcl/data4/2.pcd", cloud3);
    cloud3.header.stamp=ros::Time::now();
    cloud3.header.frame_id="/odom"; 

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        pub1.publish(cloud1);
        pub2.publish(cloud2);
        pub3.publish(cloud3);
        loop_rate.sleep();
    }
  return (0);
}
