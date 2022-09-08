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
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Utils.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/OccupancyGrid.h>

const float octomap_resolution=0.05;
const float cloudCentroid[3]={0,0.0,0};
octomap::OcTree cloudAndUnknown(octomap_resolution);

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

void PointCloudCallBack(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    std::cout << "cloud.points.size(): " << cloud->points.size() << std::endl;
    for (size_t i = 0; i <cloud->points.size(); i++){ 
        octomap::OcTreeNode * cloudNode=cloudAndUnknown.updateNode(cloud->points[i].x+cloudCentroid[0],
                                                                    cloud->points[i].y+cloudCentroid[1],
                                                                    cloud->points[i].z+cloudCentroid[2],
                                                                    true);
        cloudNode->setValue(1);
    }

}

void octomapToOccupancyGrid(const octomap::OcTree octree, nav_msgs::OccupancyGrid &map, const double minZ_, const double maxZ_ )
{
    map.info.resolution = octree.getResolution();
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    octree.getMetricMin(minX, minY, minZ);
    octree.getMetricMax(maxX, maxY, maxZ);
    minZ = std::max(minZ_, minZ);
    maxZ = std::min(maxZ_, maxZ);
    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey, maxKey, curKey;
    if (!octree.coordToKeyChecked(minPt, minKey))
    {
        ROS_ERROR("Could not create OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
        return;
    }
    if (!octree.coordToKeyChecked(maxPt, maxKey))
    {
        ROS_ERROR("Could not create OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
        return;
    }
    map.info.width = maxKey[0] - minKey[0] + 1;
    map.info.height = maxKey[1] - minKey[1] + 1;
    // might not exactly be min / max:
    octomap::point3d origin =   octree.keyToCoord(minKey, octree.getTreeDepth());
    map.info.origin.position.x = origin.x() - octree.getResolution() * 0.5;
    map.info.origin.position.y = origin.y() - octree.getResolution() * 0.5;
    map.info.origin.orientation.x = 0.;
    map.info.origin.orientation.y = 0.;
    map.info.origin.orientation.z = 0.;
    map.info.origin.orientation.w = 1.;
    // Allocate space to hold the data
    map.data.resize(map.info.width * map.info.height, -1);
    //init with unknown
    for(std::vector<int8_t>::iterator it = map.data.begin(); it != map.data.end(); ++it) {
      *it = -1;
    }
    // iterate over all keys:
    unsigned i, j;
    for (curKey[1] = minKey[1], j = 0; curKey[1] <= maxKey[1]; ++curKey[1], ++j)
    {
        for (curKey[0] = minKey[0], i = 0; curKey[0] <= maxKey[0]; ++curKey[0], ++i)
        {
            for (curKey[2] = minKey[2]; curKey[2] <= maxKey[2]; ++curKey[2])
            { 
                //iterate over height
                octomap::OcTreeNode* node = octree.search(curKey);
                if (node)
                {
                    bool occupied = octree.isNodeOccupied(node);
                    if(occupied) 
                    {
                        map.data[map.info.width * j + i] = 100;
                        break;
                    }
                    else 
                    {
                        map.data[map.info.width * j + i] = 0;
                    }
                }
            }
        }
    }
}

int main(int argc, char** argv)
{
    std::string topic_num = argv[2] ;
    std::cout << "topic_num: " << topic_num << std::endl;

    ros::init (argc, argv, "PointPub"+topic_num);
    ros::NodeHandle nh;

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output"+topic_num, 1);
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("wall_octomap_pub", 10, false);
    ros::Publisher occupancy_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);

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


    // Craete OctomapMsg
    octomap_msgs::Octomap octomapMsg;
    octomapMsg.header.frame_id = "map";
    PointCloudCallBack(cloud_ground);
    octomap_msgs::binaryMapToMsg(cloudAndUnknown, octomapMsg);

    nav_msgs::OccupancyGrid occupancy_map;
    octomapToOccupancyGrid(cloudAndUnknown, occupancy_map, 0, 3);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output_pointcloud);
        octomap_pub.publish(octomapMsg);
        occupancy_pub_.publish(occupancy_map);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    ros::shutdown();
}