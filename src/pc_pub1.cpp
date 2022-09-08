#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

void AlignWithGround(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground)
{
    pcl::PointXYZRGB min_p, max_p;
	pcl::getMinMax3D(*cloud, min_p, max_p);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // transform(2,3) = 0.0;
    transform(2,3) = -min_p.z;
    pcl::transformPointCloud (*cloud, *cloud_ground, transform);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr DownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const float& leafSize_)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointXYZRGB minPoint, maxPoint, centerPoint;
    pcl::getMinMax3D(*cloud,minPoint, maxPoint);
    centerPoint.x = 0.5*(minPoint.x + maxPoint.x);
    centerPoint.y = 0.5*(minPoint.y + maxPoint.y);
    centerPoint.z = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix(0,3) = -centerPoint.x;
    transform_matrix(1,3) = -centerPoint.y;
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform_matrix);

    float leafSize = leafSize_;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(transformed_cloud);
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    voxel.filter(*cloud_downsampled);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::transformPointCloud (*cloud_downsampled, *transformed_cloud2, transform_matrix.inverse());

    return transformed_cloud2;
}

int main(int argc, char** argv)
{
    std::string topic_num = argv[2] ;
    std::cout << "topic_num: " << topic_num << std::endl;

    ros::init (argc, argv, "PointPub"+topic_num);
    ros::NodeHandle nh;

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output"+topic_num, 1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[1], *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    // Downsample
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_downsampled = DownSample(cloud, 0.01);

    // Outlier removal
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliner(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_downsampled);
    sor.setMeanK(50);
    sor.setStddevMulThresh(5.0);
    sor.filter(*cloud_inliner);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZRGB>);
    AlignWithGround(cloud_inliner, cloud_ground);

    // Convert the cloud to ROS message
    sensor_msgs::PointCloud2 output_pointcloud;
    pcl::toROSMsg(*cloud_ground, output_pointcloud);
    output_pointcloud.header.frame_id = "map";
    std::cout << "Filterd size: " << output_pointcloud.data.size() << std::endl;

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output_pointcloud);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    ros::shutdown();
}