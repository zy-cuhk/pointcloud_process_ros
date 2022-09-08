#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <assert.h>
#include <jsoncpp/json/json.h> 


#include <ros/ros.h>  
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>  

#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>  

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Utils.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PoseArray.h>

#include <thread>

#include <renov_msgs/fov_pose.h>
#include <renov_msgs/array1D.h>
#include <renov_msgs/array2D.h>
#include <renov_msgs/array3D.h>
#include <renov_msgs/GridWaypoints.h>
#include <renov_msgs/array_GridWaypoints.h>

#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/Pose.h>

#include <time.h>
#include <eigen_conversions/eigen_msg.h>

#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace std;

float camera_depth=0.6, camera_width=0.05, fov_resolution=0.005;
float fov_plane_height = 0.4, fov_corner_height = 0.5;
float fov_distance_plane = 0.5, fov_distance_corner = 0.5;
float adjacentwaypoints_distance=0.05, adjacentpaths_distance=0.25;
float adjacentpaths_distance1=0.25;
float camera_depth1=0.7;

const float octomap_resolution=0.02;
const float cloudCentroid[3]={0,0.0,0};
octomap::OcTree cloudAndUnknown(octomap_resolution);
octomap::OcTree cloudAndUnknown2(0.05);
visualization_msgs::MarkerArray plane_marker;
visualization_msgs::MarkerArray corner_marker;
bool flag_point = false, flag_plane_marker = false, flag_corner_marker = false;

static nav_msgs::OccupancyGrid OccupancyGrid_msg;

/*
 * Creates a 2D Occupancy Grid from the Octomap.
 */
void octomapToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, const double minZ_, const double maxZ_ )
{
    map.info.resolution = octree.getResolution();
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    octree.getMetricMin(minX, minY, minZ);
    octree.getMetricMax(maxX, maxY, maxZ);
    ROS_WARN("Octree min %f %f %f", minX, minY, minZ);
    ROS_WARN("Octree max %f %f %f", maxX, maxY, maxZ);
    minZ = std::max(minZ_, minZ);
    maxZ = std::min(maxZ_, maxZ);

    octomap::point3d minPt(minX, minY, min#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Utils.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/OccupancyGrid.h>

const float octomap_resolution=0.1;
const float cloudCentroid[3]={0,0,0};
octomap::OcTree cloudAndUnknown(octomap_resolution), cloudAndUnknown1(octomap_resolution),cloudAndUnknown2(octomap_resolution) ;

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plane_normal,
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_corner_normal,
                   std::vector <pcl::PointIndices> &clusters)
{
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (40);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.1, 2.4);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (10000);
    reg.setMaxClusterSize (10000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (40);
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);
    reg.extract (clusters);

    // // build xyzrpy normal cloud
    // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointXYZRGBNormal>) ;
    // // cloud_normal->width = cloud->width;
    // // cloud_normal->height = cloud->height;
    // // cloud_normal->resize(cloud->width*cloud->height);
    // for (size_t i = 0; i < cloud->points.size(); i++)
    // {
    //     pcl::PointXYZRGBNormal p_normal;
    //     p_normal.x = cloud->points[i].x;
    //     p_normal.y = cloud->points[i].y;
    //     p_normal.z = cloud->points[i].z;
    //     p_normal.r = cloud->points[i].r;
    //     p_normal.g = cloud->points[i].g;
    //     p_normal.b = cloud->points[i].b;
    //     p_normal.normal_x = normals->points[i].normal_x;
    //     p_normal.normal_y = normals->points[i].normal_y;
    //     p_normal.normal_z = normals->points[i].normal_z;
    //     p_normal.curvature = normals->points[i].curvature;

    //     cloud_normal->push_back(p_normal);
    // }

    // // plane cloud ---------------------------------------------------------------------
    // pcl::PointIndices plane_index;
    // for (size_t i = 0; i < clusters.size(); i++)
    // {
    //     plane_index.indices.insert(plane_index.indices.end(), clusters[i].indices.begin(), clusters[i].indices.end());
    // }
    // pcl::ExtractIndices<pcl::PointXYZRGBNormal> plane_points_extract;
    // // Extract the inliers
    // plane_points_extract.setInputCloud (cloud_normal);

    // pcl::IndicesPtr plane_index_ptr = boost::make_shared<std::vector<int>>(plane_index.indices);

    // plane_points_extract.setIndices (plane_index_ptr);
    // plane_points_extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
    // plane_points_extract.filter (*cloud_plane_normal);
    // std::cout << "cloud_plane_normal size:" << cloud_plane_normal->points.size() << std::endl;

    // // ------------------------------------------------------------------------------

    // // corner cloud -----------------------------------------------------------------
    // // Extract the inliers
    // pcl::ExtractIndices<pcl::PointXYZRGBNormal> corner_points_extract;
    // corner_points_extract.setInputCloud (cloud_normal);
    // corner_points_extract.setIndices (plane_index_ptr);
    // corner_points_extract.setNegative (true);//如果设为true,可以提取指定index之外的点云
    // corner_points_extract.filter (*cloud_corner_normal);
    // std::cout << "cloud_corner_normal size:" << cloud_corner_normal->points.size() << std::endl;


    //可视化聚类的结果
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    std::cout << "num cloud: " << colored_cloud->points.size() << std::endl;

    return colored_cloud;
}


void octomapToOccupancyGrid2(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, const double minZ_, const double maxZ_ )
{
    map.info.resolution = octree.getResolution();
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    octree.getMetricMin(minX, minY, minZ);
    octree.getMetricMax(maxX, maxY, maxZ);
    ROS_WARN("Octree min %f %f %f", minX, minY, minZ);
    ROS_WARN("Octree max %f %f %f", maxX, maxY, maxZ);
    minZ = std::max(minZ_, minZ);
    maxZ = std::min(maxZ_, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey, maxKey, curKey;

    ROS_WARN("create OcTree minkey at %f %f %f", minPt.x(), minPt.y(), minPt.z());
    ROS_WARN("create OcTree maxkey at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());

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
    std::cout << "map.info.width * map.info.height: " << map.info.width * map.info.height << std::endl;

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
                        map.data[map.info.width * j + i] += 1;
                    }
                    // else 
                    // {
                    //     map.data[map.info.width * j + i] = 0;
                    // }
                }
            }
        }
    }
}

int main(int argc, char** argv)
{

    ros::init (argc, argv, "PointSeg");
    ros::NodeHandle nh;

    ros::Publisher pcl_wall1_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_wall1", 1);
    ros::Publisher pcl_wall2_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_wall2", 1);
    ros::Publisher pcl_wall_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_wall", 1);
    ros::Publisher octomap_pub1 = nh.advertise<octomap_msgs::Octomap>("wall1_octomap", 10, false);
    ros::Publisher octomap_pub2 = nh.advertise<octomap_msgs::Octomap>("wall2_octomap", 10, false);
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("wall_octomap", 10, false);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[1], *cloud_1) == -1)
    {
        std::cout << "Cloud1 reading failed." << std::e#include <nav_msgs/OccupancyGrid.h>
ndl;
        return (-1);
    }
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[2], *cloud_2) == -1)
    {
        std::cout << "Cloud2 reading failed." << std::endl;
        return (-1);
    }



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_zfilter1(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[3], *cloud_zfilter1) == -1)
    {
        // Downsample
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled1(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_downsampled1 = DownSample(cloud_1, 0.01);
        // Outlier removal
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliner1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor1;
        sor1.setInputCloud(cloud_downsampled1);
        sor1.setMeanK(50);
        sor1.setStddevMulThresh(5.0);
        sor1.filter(*cloud_inliner1);
        pcl::PassThrough<pcl::PointXYZRGB> pass1;
        pass1.setInputCloud (cloud_inliner1);
        pass1.setFilterFieldName ("z");
        pass1.setFilterLimits (0.1, 2.4);
        pass1.filter(*cloud_zfilter1);
        pcl::io::savePCDFileASCII ("/home/k/gazebo_ws/data/2021-10-29_18:09:04/2.pcd", *cloud_zfilter1); 
    }

    sensor_msgs::PointCloud2 output_pointcloud_wall1;
    pcl::toROSMsg(*cloud_zfilter1, output_pointcloud_wall1);
    output_pointcloud_wall1.header.frame_id = "map";
    std::cout << "data size is: " << output_pointcloud_wall1.data.size() << std::endl;
    for (size_t i = 0; i < cloud_zfilter1->points.size(); i++){ 
        octomap::OcTreeNode * cloudNode=cloudAndUnknown1.updateNode(cloud_zfilter1->points[i].x,
                                                                   cloud_zfilter1->points[i].y,
                                                                   cloud_zfilter1->points[i].z,
                                                                   false);
        cloudNode->setValue(0);
    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_zfilter2(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[4], *cloud_zfilter2) == -1)
    {
        // Downsample
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled2(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_downsampled2 = DownSample(cloud_2, 0.01);
        // Outlier removal
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliner2(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
        sor2.setInputCloud(cloud_downsampled2);
        sor2.setMeanK(50);
        sor2.setStddevMulThresh(5.0);
        sor2.filter(*cloud_inliner2);
        pcl::PassThrough<pcl::PointXYZRGB> pass2;
        pass2.setInputCloud (cloud_inliner2);
        pass2.setFilterFieldName ("z");
        pass2.setFilterLimits (0.1, 2.4);
        pass2.filter(*cloud_zfilter2);
        pcl::io::savePCDFileASCII ("/home/k/gazebo_ws/data/2021-10-29_18:09:04/3.pcd", *cloud_zfilter2); 
    }
    sensor_msgs::PointCloud2 output_pointcloud_wall2;
    pcl::toROSMsg(*cloud_zfilter2, output_pointcloud_wall2);
    output_pointcloud_wall2.header.frame_id = "map";
    std::cout << "data size is: " << output_pointcloud_wall2.data.size() << std::endl;
    for (size_t i = 0; i < cloud_zfilter2->points.size(); i++){ 
        octomap::OcTreeNode * cloudNode=cloudAndUnknown2.updateNode(cloud_zfilter2->points[i].x,
                                                                   cloud_zfilter2->points[i].y,
                                                                   cloud_zfilter2->points[i].z,
                                                                   false);
        cloudNode->setValue(0);
    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_zfilter(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[5], *cloud_zfilter) == -1)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        *cloud += *cloud_1 + *cloud_2;
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
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (cloud_inliner);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.1, 2.4);
        pass.filter(*cloud_zfilter);
        pcl::io::savePCDFileASCII ("/home/k/gazebo_ws/data/2021-10-29_18:09:04/4.pcd", *cloud_zfilter); 
    }
    sensor_msgs::PointCloud2 output_pointcloud_wall;
    pcl::toROSMsg(*cloud_zfilter, output_pointcloud_wall);
    output_pointcloud_wall.header.frame_id = "map";
    std::cout << "data size is: " << output_pointcloud_wall.data.size() << std::endl;
    for (size_t i = 0; i < cloud_zfilter->points.size(); i++)
    { 
        octomap::OcTreeNode * cloudNode=cloudAndUnknown.updateNode(cloud_zfilter->points[i].x,
                                                                   cloud_zfilter->points[i].y,
                                                                   cloud_zfilter->points[i].z,
                                                                   true);
        cloudNode->setValue(1);
    }
    cloudAndUnknown.updateInnerOccupancy();


    // int theta_num = 360;
    // int height_num = 26; 
    // octomap::point3d Point3dwall_plane(0.0,0.0,0.0); 
    // octomap::Pointcloud fov;  
    // for(int ii=0;ii<theta_num;ii++)
    // {
    //     for (int iii=0;iii<height_num;iii++)
    //     {
    //         Point3dwall_plane.x()=20.0*cos(M_PI*ii/180.0);      
    //         Point3dwall_plane.y()=20.0*sin(M_PI*ii/180.0); 
    //         Point3dwall_plane.z()=iii*0.1; 
    //         fov.push_back(Point3dwall_plane);
    //     }
    // }
    // octomap::Pointcloud variablefov;
    // octomap::point3d iterator;  
    // iterator.x()=0.0;
    // iterator.y()=0.0;
    // iterator.z()=0.0;
    // float roll, pitch, yaw;
    // roll=0.0;
    // pitch=0.0;
    // yaw=0.0;
    // octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());     
    // octomath::Quaternion Rotation2(roll,pitch,yaw); 
    // octomath::Pose6D RotandTrans2(Translation2,Rotation2);  
    // variablefov=fov;        
    // variablefov.transform(RotandTrans2);
    // octomap::KeyRay rayBeam;
    // for (int ii=0; ii<variablefov.size();ii++){
    //     bool Continue=true;     
    //     cloudAndUnknown.computeRayKeys(iterator,variablefov.getPoint(ii),rayBeam);
    //     for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
    //         octomap::OcTreeNode* node=cloudAndUnknown.search(*it);
    //         if(node!=NULL && cloudAndUnknown.isNodeOccupied(node))
    //         {
    //             cloudAndUnknown.updateNode(*it, false);
    //             Continue=false; 
    //         }
    //     }
    // }
    // cloudAndUnknown.updateInnerOccupancy();


    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        // pcl_wall1_pub.publish(output_pointcloud_wall1);
        // pcl_wall2_pub.publish(output_pointcloud_wall2);
        // pcl_wall_pub.publish(output_pointcloud_wall);

        octomap_msgs::Octomap octomapMsg;
        octomapMsg.header.frame_id = "map";
        octomap_msgs::binaryMapToMsg(cloudAndUnknown, octomapMsg);
        octomap_pub.publish(octomapMsg);


        // octomap_msgs::Octomap octomapMsg1;
        // octomapMsg1.header.frame_id = "map";
        // octomap_msgs::binaryMapToMsg(cloudAndUnknown1, octomapMsg1);
        // octomap_pub1.publish(octomapMsg1);


        // octomap_msgs::Octomap octomapMsg2;
        // octomapMsg2.header.frame_id = "map";
        // octomap_msgs::binaryMapToMsg(cloudAndUnknown2, octomapMsg2);
        // octomap_pub2.publish(octomapMsg2);

        std::cout<<"yes"<<std::endl;

        // ros::spinOnce();
        // loop_rate.sleep();
    }
    
    ros::shutdown();
}Z);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey, maxKey, curKey;

    ROS_WARN("create OcTree minkey at %f %f %f", minPt.x(), minPt.y(), minPt.z());
    ROS_WARN("create OcTree maxkey at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());

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
    std::cout << "map.info.width * map.info.height: " << map.info.width * map.info.height << std::endl;

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

void octomapToOccupancyGrid2(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, const double minZ_, const double maxZ_ )
{
    map.info.resolution = octree.getResolution();
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    octree.getMetricMin(minX, minY, minZ);
    octree.getMetricMax(maxX, maxY, maxZ);
    ROS_WARN("Octree min %f %f %f", minX, minY, minZ);
    ROS_WARN("Octree max %f %f %f", maxX, maxY, maxZ);
    minZ = std::max(minZ_, minZ);
    maxZ = std::min(maxZ_, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey, maxKey, curKey;

    ROS_WARN("create OcTree minkey at %f %f %f", minPt.x(), minPt.y(), minPt.z());
    ROS_WARN("create OcTree maxkey at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());

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
    std::cout << "map.info.width * map.info.height: " << map.info.width * map.info.height << std::endl;

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
                        map.data[map.info.width * j + i] += 1;
                    }
                    // else 
                    // {
                    //     map.data[map.info.width * j + i] = 0;
                    // }
                }
            }
        }
    }
}

void octomapChangeResolution(const octomap::OcTree& octree )
{
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    octree.getMetricMin(minX, minY, minZ);
    octree.getMetricMax(maxX, maxY, maxZ);
    ROS_WARN("Octree min %f %f %f", minX, minY, minZ);
    ROS_WARN("Octree max %f %f %f", maxX, maxY, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey, maxKey, curKey;

    ROS_WARN("create OcTree minkey at %f %f %f", minPt.x(), minPt.y(), minPt.z());
    ROS_WARN("create OcTree maxkey at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());

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
                        cloudAndUnknown2.updateNode(cloudAndUnknown.keyToCoord(curKey).x(),
                                                    cloudAndUnknown.keyToCoord(curKey).y(),
                                                    cloudAndUnknown.keyToCoord(curKey).z(),
                                                     true);
                    }
                }
            }
        }
    }
}

void PointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if(!flag_point == true)
    {
        std::cout << " recive pointcloud" << std::endl;

        sensor_msgs::PointCloud2 recive_msg;
        recive_msg = *msg;
        pcl::PCLPointCloud2 cloud;
        pcl_conversions::toPCL(recive_msg,cloud);
        pcl::PointCloud<pcl::PointXYZRGB> pcl_pc;
        pcl::fromPCLPointCloud2(cloud,pcl_pc);
        std::cout << "pcl_pc.points.size(): " << pcl_pc.points.size() << std::endl;
        for (size_t i = 0; i <pcl_pc.points.size(); i++){ 
            octomap::OcTreeNode * cloudNode=cloudAndUnknown.updateNode(pcl_pc.points[i].x+cloudCentroid[0],
                                                                        pcl_pc.points[i].y+cloudCentroid[1],
                                                                        pcl_pc.points[i].z+cloudCentroid[2],
                                                                        true);
            cloudNode->setValue(1);
        }

        OccupancyGrid_msg.header.frame_id = "map";
        OccupancyGrid_msg.header.stamp = ros::Time::now();
        octomapToOccupancyGrid2(cloudAndUnknown, OccupancyGrid_msg, 0, std::numeric_limits<double>::max());
        
        flag_point = true;
    }

}

void PlaneMarkerCallBack(const visualization_msgs::MarkerArrayConstPtr& msg)
{
    if(!flag_plane_marker == true)
    {
        std::cout << " recive PlaneMarker" << std::endl;
        for (size_t i = 0; i < msg->markers.size(); i++)
        {
            plane_marker.markers.push_back(msg->markers[i]);
        }
        std::cout << "plane_marker size: " << plane_marker.markers.size() << std::endl;
        flag_plane_marker = true;
    }
}

void CornerMarkerCallBack(const visualization_msgs::MarkerArrayConstPtr& msg)
{
    if(!flag_corner_marker == true)
    {
        std::cout << " recive CornerMarker" << std::endl;
        for (size_t i = 0; i < msg->markers.size(); i++)
        {
            corner_marker.markers.push_back(msg->markers[i]);
        }
        std::cout << "corner_marker size: " << corner_marker.markers.size() << std::endl;
        flag_corner_marker = true;
    }
}

void BuildFoVMarker(const Eigen::Quaternionf& quat,
                    const Eigen::Vector3f& deltaMove,
                    const int& id,
                    const std::string& name,
                    visualization_msgs::Marker &marker_FOV,
                    const float& camera_height)
{

    marker_FOV.id = id;
    // marker_FOV.ns = name;

    // float camera_depth=0.6, camera_width=0.05, camera_height=0.3, fov_resolution=0.005;
    //Camera is a pyramid. Define in camera coordinate system
    Eigen::Vector3f o,p1,p2,p3,p4;
    o << 0, 0, 0;
    p1 <<  camera_height/2.0,  camera_width/2.0, camera_depth;
    p2 <<  camera_height/2.0, -camera_width/2.0, camera_depth;
    p3 << -camera_height/2.0, -camera_width/2.0, camera_depth;
    p4 << -camera_height/2.0,  camera_width/2.0, camera_depth;

    Eigen::Isometry3f T = Eigen::Isometry3f::Identity(); 
    T.rotate(quat);
    T.pretranslate(deltaMove);

    Eigen::Vector3f ow = T*o;
    Eigen::Vector3f p1w = T*p1;
    Eigen::Vector3f p2w = T*p2;
    Eigen::Vector3f p3w = T*p3;
    Eigen::Vector3f p4w = T*p4;

    geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    msgs_o.x=ow[0];
    msgs_o.y=ow[1];
    msgs_o.z=ow[2];
    msgs_p1.x=p1w[0];
    msgs_p1.y=p1w[1];
    msgs_p1.z=p1w[2];
    msgs_p2.x=p2w[0];
    msgs_p2.y=p2w[1];
    msgs_p2.z=p2w[2];
    msgs_p3.x=p3w[0];
    msgs_p3.y=p3w[1];
    msgs_p3.z=p3w[2];
    msgs_p4.x=p4w[0];
    msgs_p4.y=p4w[1];
    msgs_p4.z=p4w[2];

    marker_FOV.points.push_back(msgs_o);
    marker_FOV.points.push_back(msgs_p1);
    marker_FOV.points.push_back(msgs_o);
    marker_FOV.points.push_back(msgs_p2);
    marker_FOV.points.push_back(msgs_o);
    marker_FOV.points.push_back(msgs_p3);
    marker_FOV.points.push_back(msgs_o);
    marker_FOV.points.push_back(msgs_p4);
    marker_FOV.points.push_back(msgs_p1);
    marker_FOV.points.push_back(msgs_p2);
    marker_FOV.points.push_back(msgs_p2);
    marker_FOV.points.push_back(msgs_p3);
    marker_FOV.points.push_back(msgs_p3);
    marker_FOV.points.push_back(msgs_p4);
    marker_FOV.points.push_back(msgs_p4);
    marker_FOV.points.push_back(msgs_p1);

    marker_FOV.header.stamp = ros::Time::now();

    // return marker_FOV;
}

void thread_processing(const int width, 
                        const std::vector<std::vector<float>> candidate_waypoint_positions, 
                        const octomap::Pointcloud pointwall,
                        octomap::OcTree octomap_,
                        std::map<int, octomap::OcTreeKey> &wall_index_waypoint_octkey,
                        std::vector<octomap::OcTreeKey> &wall_index_octkey,
                        const visualization_msgs::Marker& marker,
                        const bool& bCorner,
                        renov_msgs::array3D& waypoints_3D
                        // std::map<std::pair<int, int>, std::set<int> >& mGrid
                        )
                        
{
    int candidate_waypoints_num = candidate_waypoint_positions.size();
    int height = candidate_waypoints_num/width;
    int min_height = 0;
    int max_height = height;

    for (int j=width*min_height; j<width*max_height && j<candidate_waypoints_num; j++)
    {
        // phase 2-step 1.1: obtain the pose of camera waypoint 
        octomap::Pointcloud variablePointwall;
        octomap::point3d iterator;  
        iterator.x()=candidate_waypoint_positions[j][0];
        iterator.y()=candidate_waypoint_positions[j][1];
        iterator.z()=candidate_waypoint_positions[j][2];
        float roll, pitch, yaw;
        roll=candidate_waypoint_positions[j][3];
        pitch=candidate_waypoint_positions[j][4];
        yaw=candidate_waypoint_positions[j][5];
        octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());     
        octomath::Quaternion Rotation2(roll,pitch,yaw); 
        octomath::Pose6D RotandTrans2(Translation2,Rotation2);  
        variablePointwall=pointwall;        
        variablePointwall.transform(RotandTrans2);

        // phase 2-step 1.2: raycast to obtain voxel from the camera waypoint pose
        // std::vector
        octomap::KeyRay rayBeam;
        int unknownVoxelsInRay=0;
        int known_points_projection=0;
        renov_msgs::array2D octo_index;

        bool ok_fov = false;
        // std::map<std::pair<int, int>, std::set<int> > mGrid;

        for (int ii=0; ii<variablePointwall.size();ii++){
            bool Continue=true;     
            octomap_.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
            for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
                octomap::OcTreeNode* node=octomap_.search(*it);
                
                Eigen::Vector3f key_point(octomap_.keyToCoord(*it).x(), 
                                          octomap_.keyToCoord(*it).y(), 
                                          octomap_.keyToCoord(*it).z());
                Eigen::Vector3f wall_center(marker.pose.position.x,
                                            marker.pose.position.y,
                                            marker.pose.position.z);
                Eigen::Vector3f direction = wall_center - key_point;
                Eigen::Quaternionf quat(marker.pose.orientation.w,
                                        marker.pose.orientation.x,
                                        marker.pose.orientation.y,
                                        marker.pose.orientation.z);
                Eigen::Matrix3f Rotation_matrix = quat.toRotationMatrix();
                Eigen::Vector3f wall_normal = Rotation_matrix.block<3,1>(0,2);

                
                bool inner_flag = false;
                if(bCorner)
                    inner_flag = fabs(direction.dot(wall_normal)) <= 0.2;
                else
                    inner_flag = fabs(direction.dot(wall_normal)) <= 0.1;

                // inner_flag = true;

                if(node!=NULL && octomap_.isNodeOccupied(node) && inner_flag)
                {
                    // octomap_.integrateHit
                    octomap_.updateNode(*it, false);
                    // Continue=false; 
                    if(!wall_index_waypoint_octkey.count(j))
                        wall_index_waypoint_octkey.insert(std::make_pair(j, *it));

                    wall_index_octkey.push_back(*it);

                    renov_msgs::array1D coord_xyz;
                    coord_xyz.x = octomap_.keyToCoord(*it).x();
                    coord_xyz.y = octomap_.keyToCoord(*it).y();
                    coord_xyz.z = octomap_.keyToCoord(*it).z();

                    octo_index.coord.push_back(coord_xyz);
                    ok_fov = true;

                    // map grid
                    // std::pair<int, int> iGrid = std::make_pair(coord_xyz.x, coord_xyz.y);
                    // if(mGrid.find(iGrid) == mGrid.end())
                    // {
                    //     std::set<int> sWaypoints_Index;
                    //     sWaypoints_Index.insert(j);
                    //     mGrid.insert(std::make_pair(iGrid, sWaypoints_Index));
                    // }
                    // else
                    // {
                    //     mGrid[iGrid].insert(j);
                    // }
                }
            }
        }
        
        if(ok_fov)
        {
            if(octo_index.coord.size() != 0)
            {
                renov_msgs::fov_pose pose;

                waypoints_3D.FoV.push_back(octo_index);
                pose.Px = candidate_waypoint_positions[j][0];
                pose.Py = candidate_waypoint_positions[j][1];
                pose.Pz = candidate_waypoint_positions[j][2];
                pose.Rx = candidate_waypoint_positions[j][3];
                pose.Ry = candidate_waypoint_positions[j][4];
                pose.Rz = candidate_waypoint_positions[j][5];
                waypoints_3D.poses.push_back(pose);
                waypoints_3D.waypoints_index.push_back(j);
            }
        
        }
        octomap_.updateInnerOccupancy();

    }

    // 

}

void wallceilingcorner_division(int iMarker,
                                visualization_msgs::MarkerArray corner_marker,
                                std::vector<int>& wallcorner_markerindex, 
                                std::vector<int>& ceilcorner_markerindex,
                                std::vector<std::vector<float>>& wallcorner_positions, 
                                std::vector<std::vector<float>>& ceilcorner_positions)
{
    Eigen::Vector3f OBB_Position(corner_marker.markers[iMarker].pose.position.x,
                                 corner_marker.markers[iMarker].pose.position.y,
                                 corner_marker.markers[iMarker].pose.position.z-0.0);
    Eigen::Quaternionf quat(corner_marker.markers[iMarker].pose.orientation.w,
                            corner_marker.markers[iMarker].pose.orientation.x,
                            corner_marker.markers[iMarker].pose.orientation.y,
                            corner_marker.markers[iMarker].pose.orientation.z);
    Eigen::Matrix3f Rotation_matrix = quat.toRotationMatrix();
    Eigen::Vector3f major_vector = Rotation_matrix.block<3,1>(0,0);
    Eigen::Vector3f middle_vector = Rotation_matrix.block<3,1>(0,1);
    Eigen::Vector3f minor_vector = Rotation_matrix.block<3,1>(0,2);
    Eigen::Vector3f Start_Position = OBB_Position-major_vector*(corner_marker.markers[iMarker].scale.x/2)
                                                    -middle_vector*(corner_marker.markers[iMarker].scale.y/2)
                                                    -minor_vector*(corner_marker.markers[iMarker].scale.z/2);
    Eigen::Vector3f end_Position = OBB_Position+major_vector*(corner_marker.markers[iMarker].scale.x/2)
                                                +middle_vector*(corner_marker.markers[iMarker].scale.y/2)
                                                +minor_vector*(corner_marker.markers[iMarker].scale.z/2);
    std::cout<<"Start_Position is: "<<Start_Position[0]<<","<<Start_Position[1]<<","<<Start_Position[2]<<std::endl;
    std::cout<<"end_Position is: "<<end_Position[0]<<","<<end_Position[1]<<","<<end_Position[2]<<std::endl;
    std::vector<float> startendposition(3);
    for (int dof=0; dof<3; dof++)
    {
        startendposition[dof]=OBB_Position[dof];
        // startendposition[3+dof]=OBB_Position[dof];
    }

    Eigen::Vector3f direction_eulerAngle;        
    bool belong_ceil;
    Eigen::Vector3f z_axis = Rotation_matrix.col(2);
    Eigen::Vector3f x_axis = z_axis;
    x_axis[2] = 0;
    if(z_axis.dot(x_axis) > 0.9 )
    {
        belong_ceil = false;
        wallcorner_markerindex.push_back(iMarker);
        wallcorner_positions.push_back(startendposition);
    }
    else
    {
        belong_ceil = true;
        ceilcorner_markerindex.push_back(iMarker);
        ceilcorner_positions.push_back(startendposition);
    }
}

void cornerwaypoints_generation(int iMarker, 
                                int& iMarker1,
                                int& iMarker2,
                                octomap::OcTree& cloudAndUnknown,
                                octomap::Pointcloud pointwall_corner,
                                visualization_msgs::MarkerArray corner_marker,
                                visualization_msgs::Marker marker_FOV_Corner,
                                visualization_msgs::Marker marker_FOV2_Corner,
                                geometry_msgs::PoseArray& corner_posearray,
                                std::vector<visualization_msgs::MarkerArray>& vmsg_fov_marker_corner, 
                                std::vector<visualization_msgs::MarkerArray>& vmsg_fov_marker2_corner, 
                                Json::Value& ceilingcornerwaypoints_dict1,
                                Json::Value& ceilingcornerwaypoints_dict2,
                                Json::Value& wallcornerwaypoints_dict1, 
                                Json::Value& wallcornerwaypoints_dict2) 
{
    // step 1: generate candidate painting waypoints 
    Eigen::Vector3f OBB_Position(corner_marker.markers[iMarker].pose.position.x,
                                    corner_marker.markers[iMarker].pose.position.y,
                                    corner_marker.markers[iMarker].pose.position.z-0.0);
    Eigen::Quaternionf quat(corner_marker.markers[iMarker].pose.orientation.w,
                            corner_marker.markers[iMarker].pose.orientation.x,
                            corner_marker.markers[iMarker].pose.orientation.y,
                            corner_marker.markers[iMarker].pose.orientation.z);
    Eigen::Matrix3f Rotation_matrix = quat.toRotationMatrix();
    Eigen::Vector3f major_vector = Rotation_matrix.block<3,1>(0,0);
    Eigen::Vector3f middle_vector = Rotation_matrix.block<3,1>(0,1);
    Eigen::Vector3f minor_vector = Rotation_matrix.block<3,1>(0,2);
    Eigen::Vector3f Start_Position = OBB_Position-major_vector*(corner_marker.markers[iMarker].scale.x/2)
                                                 -middle_vector*(corner_marker.markers[iMarker].scale.y/2)
                                                 -minor_vector*(corner_marker.markers[iMarker].scale.z/2);

    Eigen::Vector3f direction_eulerAngle;        
    bool belong_ceil;
    int height, width;
    // Judge direction
    Eigen::Vector3f z_axis = Rotation_matrix.col(2);
    Eigen::Vector3f x_axis = z_axis;
    x_axis[2] = 0;
    if(z_axis.dot(x_axis) > 0.9 )
    {
        // rotate along with minor_vector
        Eigen::AngleAxisf rotationVector(M_PI/2,minor_vector);
        direction_eulerAngle = (rotationVector.toRotationMatrix()*quat.toRotationMatrix()).eulerAngles(2,1,0);
        height = ceil(corner_marker.markers[iMarker].scale.x/0.05);
        width = ceil(corner_marker.markers[iMarker].scale.y/0.25);
        belong_ceil = false;
    }
    else
    {
        // rotate along with minor_vector
        Eigen::AngleAxisf rotationVector(0,minor_vector);
        direction_eulerAngle = (rotationVector.toRotationMatrix()*quat.toRotationMatrix()).eulerAngles(2,1,0);
        height = ceil(corner_marker.markers[iMarker].scale.x/0.25);
        width = ceil(corner_marker.markers[iMarker].scale.y/0.05);
        belong_ceil = true;
    }
    int candidate_waypoints_num = width*height;
    int cartesian_freedom = 9;
    std::vector<std::vector<float>> candidate_waypoint_positions;
    candidate_waypoint_positions.reserve(candidate_waypoints_num);
    visualization_msgs::MarkerArray msg_fov_marker;
    for(int i=0; i<candidate_waypoints_num; i++)
    {
        int iWidth = int(i%width);
        int iHeight = int(i/width);

        Eigen::Vector3f deltHeight;
        Eigen::Vector3f deltaWidth;
        if(belong_ceil==false)
        {
            deltHeight = major_vector * (iHeight*0.05+0.025);
            deltaWidth = middle_vector * (iWidth*0.25);
        }
        else
        {
            deltHeight = major_vector * (iHeight*0.25);
            deltaWidth = middle_vector * (iWidth*0.05);
            fov_distance_corner = 0.4;
        }

        Eigen::Vector3f DistanceToWall = -minor_vector * fov_distance_corner;
        Eigen::Vector3f deltaMove = deltaWidth + deltHeight + DistanceToWall + Start_Position;
        // if(deltaMove[2] <= 0 )
        //     continue;
        std::vector<float> candidate_waypoint_position(9);
        candidate_waypoint_position[0] = deltaMove[0];
        candidate_waypoint_position[1] = deltaMove[1];
        candidate_waypoint_position[2] = deltaMove[2];
        candidate_waypoint_position[3] = direction_eulerAngle[2];
        candidate_waypoint_position[4] = direction_eulerAngle[1];
        candidate_waypoint_position[5] = direction_eulerAngle[0];
        candidate_waypoint_position[6] = direction_eulerAngle[2];
        candidate_waypoint_position[7] = direction_eulerAngle[1];
        candidate_waypoint_position[8] = direction_eulerAngle[0]+M_PI;
        candidate_waypoint_positions.push_back(candidate_waypoint_position);
        std::string name = "sample_wall"+to_string(iMarker+1);
        BuildFoVMarker(quat, deltaMove, i, name, marker_FOV_Corner, fov_corner_height);
        msg_fov_marker.markers.push_back(marker_FOV_Corner);
    }
    vmsg_fov_marker_corner.push_back(msg_fov_marker);

    // step 2: generate effective waypoints and update the octomap
    visualization_msgs::MarkerArray msg_fov_marker2;
    std::map<int, octomap::OcTreeKey> wall_index_waypoint_octkey;
    std::vector<octomap::OcTreeKey> wall_index_octkey;
    renov_msgs::array3D waypoints_3D;
    thread_processing( width, candidate_waypoint_positions, pointwall_corner, cloudAndUnknown, 
                        wall_index_waypoint_octkey,wall_index_octkey, corner_marker.markers[iMarker], 
                        true, waypoints_3D);
    std::cout<<"waypoints_3D size is: "<<waypoints_3D.waypoints_index.size()<<std::endl;

    std::vector<std::pair<int, octomap::OcTreeKey>> sWall_index_waypoint_octkey;
    sWall_index_waypoint_octkey.insert(sWall_index_waypoint_octkey.end(),wall_index_waypoint_octkey.begin(),wall_index_waypoint_octkey.end());  
    for(size_t i=0; i<sWall_index_waypoint_octkey.size(); i++)
    {
        const int index = sWall_index_waypoint_octkey[i].first;
        tf::Quaternion quat;
        quat.setRPY(candidate_waypoint_positions[index][3], 
                    candidate_waypoint_positions[index][4], 
                    candidate_waypoint_positions[index][5]);
        Eigen::Quaternionf q_eigen(quat.w(), quat.x(),quat.y(),quat.z());
        Eigen::Vector3f t_eigen(candidate_waypoint_positions[index][0],
                                candidate_waypoint_positions[index][1],
                                candidate_waypoint_positions[index][2]);
        std::string name = "cover_wall"+to_string(iMarker+1);
        BuildFoVMarker(q_eigen, t_eigen, 2, name, marker_FOV2_Corner, fov_corner_height);
        msg_fov_marker2.markers.push_back(marker_FOV2_Corner);
        geometry_msgs::Pose pose_marker;
        pose_marker.position.x = t_eigen(0);
        pose_marker.position.y = t_eigen(1);
        pose_marker.position.z = t_eigen(2);
        pose_marker.orientation.w = quat.w();
        pose_marker.orientation.x = quat.x();
        pose_marker.orientation.y = quat.y();
        pose_marker.orientation.z = quat.z();
        corner_posearray.poses.push_back(pose_marker);
    }
    vmsg_fov_marker2_corner.push_back(msg_fov_marker2);
    std::vector<octomap::OcTreeKey> sWall_index_octkey;
    sWall_index_octkey.insert(sWall_index_octkey.end(),wall_index_octkey.begin(),wall_index_octkey.end());      
    for (size_t i = 0; i < sWall_index_octkey.size(); i++)
    {
        cloudAndUnknown.updateNode(sWall_index_octkey[i], false);
    }
    cloudAndUnknown.updateInnerOccupancy();

    // step 3: generate wallcornerwaypoints_dict
    int waypoints_mat[width][height];
    for (int iWidth = 0; iWidth < width; iWidth++)
    {
        for (int iHeight = 0; iHeight < height; iHeight++)
            waypoints_mat[iWidth][iHeight]=0;
    }
    for (int waypoint_num=0; waypoint_num<waypoints_3D.waypoints_index.size(); waypoint_num++)
    {
        int waypoint_index = waypoints_3D.waypoints_index[waypoint_num];
        int iWidth = int(waypoint_index%width);
        int iHeight = int(waypoint_index/width);
        waypoints_mat[iWidth][iHeight]=1;
    }
    if(belong_ceil)
    {
        std::string str1_1 = "walls_num"+to_string(iMarker1);
        for (int iWidth = 0; iWidth < width; iWidth++)
        {
            for (int iHeight = 0; iHeight < height; iHeight++)
            {
                std::string str2 = "paths_num"+to_string(iHeight);
                std::string str3 = "waypoints_num"+to_string(width-1-iWidth); 
                ceilingcornerwaypoints_dict1[str1_1][str2][str3]=waypoints_mat[iWidth][iHeight];
            }
        }
        for(int i=0; i<candidate_waypoints_num; i++)
        {
            int iWidth1 = int(i%width);
            int iHeight1 = int(i/width);
            for (int dof=0; dof<9; dof++)
            {
                std::string str2 = "paths_num"+to_string(iHeight1);
                std::string str3 = "waypoints_num"+to_string(width-1-iWidth1); 
                ceilingcornerwaypoints_dict2[str1_1][str2][str3][dof]=candidate_waypoint_positions[i][dof];
            }
        }
        iMarker1=iMarker1+1;
    }
    else
    {
        std::string str1_2 = "walls_num"+to_string(iMarker2);
        for (int iWidth = 0; iWidth < width; iWidth++)
        {
            for (int iHeight = 0; iHeight < height; iHeight++)
            {
                std::string str2 = "paths_num"+to_string(width-1-iWidth); 
                std::string str3 = "waypoints_num"+to_string(iHeight);                   
                wallcornerwaypoints_dict1[str1_2][str2][str3]=waypoints_mat[iWidth][iHeight];
            }
        }
        for(int i=0; i<candidate_waypoints_num; i++)
        {
            int iWidth1 = int(i%width);
            int iHeight1 = int(i/width);
            for (int dof=0; dof<9; dof++)
            {
                std::string str2 = "paths_num"+to_string(width-1-iWidth1); 
                std::string str3 = "waypoints_num"+to_string(iHeight1);   
                wallcornerwaypoints_dict2[str1_2][str2][str3][dof]=candidate_waypoint_positions[i][dof];
            }
        }
        iMarker2=iMarker2+1;
    }
}

void wallceilingsurface_division(int iMarker,
                                visualization_msgs::MarkerArray plane_marker,
                                std::vector<int>& wallsurface_markerindex, 
                                std::vector<int>& ceilsurface_markerindex,
                                std::vector<std::vector<float>>& wallsurface_positions, 
                                std::vector<std::vector<float>>& ceilsurface_positions)
{
    Eigen::Quaternionf quat(plane_marker.markers[iMarker].pose.orientation.w,
                            plane_marker.markers[iMarker].pose.orientation.x,
                            plane_marker.markers[iMarker].pose.orientation.y,
                            plane_marker.markers[iMarker].pose.orientation.z);
    Eigen::Matrix3f Rotation_matrix = quat.toRotationMatrix();
    Eigen::Vector3f major_vector = Rotation_matrix.block<3,1>(0,0);
    Eigen::Vector3f middle_vector = Rotation_matrix.block<3,1>(0,1);
    Eigen::Vector3f minor_vector = Rotation_matrix.block<3,1>(0,2);
    Eigen::Vector3f OBB_Position;
    bool belong_ceil = false;
    Eigen::Vector3f z_axis(0,0,1);
    if(Rotation_matrix.col(2).dot(z_axis) < 0.9 )
    {
        OBB_Position = Eigen::Vector3f(plane_marker.markers[iMarker].pose.position.x,
                                       plane_marker.markers[iMarker].pose.position.y,
                                       plane_marker.markers[iMarker].pose.position.z-0.1);   
    }
    else
    {
        OBB_Position = Eigen::Vector3f(plane_marker.markers[iMarker].pose.position.x,
                                        plane_marker.markers[iMarker].pose.position.y,
                                        plane_marker.markers[iMarker].pose.position.z);   
        belong_ceil = true;
    }
    Eigen::Vector3f Start_Position = OBB_Position-major_vector*(plane_marker.markers[iMarker].scale.x/2)
                                                    -middle_vector*(plane_marker.markers[iMarker].scale.y/2)
                                                    -minor_vector*(plane_marker.markers[iMarker].scale.z/2);
    Eigen::Vector3f end_Position = OBB_Position+major_vector*(plane_marker.markers[iMarker].scale.x/2)
                                                +middle_vector*(plane_marker.markers[iMarker].scale.y/2)
                                                +minor_vector*(plane_marker.markers[iMarker].scale.z/2);
    std::cout<<"Start_Position is: "<<Start_Position[0]<<","<<Start_Position[1]<<","<<Start_Position[2]<<std::endl;
    std::cout<<"end_Position is: "<<end_Position[0]<<","<<end_Position[1]<<","<<end_Position[2]<<std::endl;
    std::vector<float> startendposition(3);
    for (int dof=0; dof<3; dof++)
    {
        startendposition[dof]=OBB_Position[dof];
        // startendposition[3+dof]=OBB_Position[dof];
    }
    if (belong_ceil == true)
    {
        ceilsurface_markerindex.push_back(iMarker);
        ceilsurface_positions.push_back(startendposition);
    }
    else
    {
        wallsurface_markerindex.push_back(iMarker);
        wallsurface_positions.push_back(startendposition);
    }
}

void surfacewaypoints_generation(int iMarker, 
                                int& iMarker1,
                                int& iMarker2,
                                octomap::OcTree& cloudAndUnknown,
                                octomap::Pointcloud pointwall_plane,
                                visualization_msgs::MarkerArray plane_marker,
                                visualization_msgs::Marker marker_FOV,
                                visualization_msgs::Marker marker_FOV2,
                                geometry_msgs::PoseArray& wall_posearray,
                                geometry_msgs::PoseArray& wall_posearray1,
                                Json::Value& ceilingsurfacewaypoints_dict1,
                                Json::Value& ceilingsurfacewaypoints_dict2,
                                Json::Value& wallsurfacewaypoints_dict1, 
                                Json::Value& wallsurfacewaypoints_dict2)
{
    // step 1: sample candidate painting waypoint positions
    Eigen::Quaternionf quat(plane_marker.markers[iMarker].pose.orientation.w,
                            plane_marker.markers[iMarker].pose.orientation.x,
                            plane_marker.markers[iMarker].pose.orientation.y,
                            plane_marker.markers[iMarker].pose.orientation.z);
    Eigen::Matrix3f Rotation_matrix = quat.toRotationMatrix();
    Eigen::Vector3f major_vector = Rotation_matrix.block<3,1>(0,0);
    Eigen::Vector3f middle_vector = Rotation_matrix.block<3,1>(0,1);
    Eigen::Vector3f minor_vector = Rotation_matrix.block<3,1>(0,2);
    Eigen::Vector3f OBB_Position;
    bool belong_ceil = false;
    Eigen::Vector3f z_axis(0,0,1);
    if(Rotation_matrix.col(2).dot(z_axis) < 0.9 )
    {
        OBB_Position = Eigen::Vector3f(plane_marker.markers[iMarker].pose.position.x,
                                       plane_marker.markers[iMarker].pose.position.y,
                                       plane_marker.markers[iMarker].pose.position.z-0.1);   
    }
    else
    {
        OBB_Position = Eigen::Vector3f(plane_marker.markers[iMarker].pose.position.x,
                                        plane_marker.markers[iMarker].pose.position.y,
                                        plane_marker.markers[iMarker].pose.position.z);   
        belong_ceil = true;
    }

    Eigen::Vector3f Start_Position = OBB_Position-major_vector*(plane_marker.markers[iMarker].scale.x/2)
                                                 -middle_vector*(plane_marker.markers[iMarker].scale.y/2)
                                                 -minor_vector*(plane_marker.markers[iMarker].scale.z/2);
    int width;
    if (belong_ceil == true)
        width = ceil(plane_marker.markers[iMarker].scale.x/adjacentpaths_distance1);
    else
        width = ceil(plane_marker.markers[iMarker].scale.x/adjacentpaths_distance);
    int height = ceil(plane_marker.markers[iMarker].scale.y/adjacentwaypoints_distance);
    // std::cout<<"height is: "<<height<<", width is: "<<width<<std::endl;
    
    int candidate_waypoints_num = width*height;
    int cartesian_freedom = 9;
    std::vector<std::vector<float>> candidate_waypoint_positions;
    candidate_waypoint_positions.reserve(candidate_waypoints_num);
    // std::cout<<"candidate_waypoint_positions size is: "<<candidate_waypoint_positions.size()<<std::endl;
    Eigen::AngleAxisf rotationVector(M_PI/2,minor_vector);
    Eigen::Vector3f direction_eulerAngle = (rotationVector.toRotationMatrix()*quat.toRotationMatrix()).eulerAngles(2,1,0);
    // Eigen::Vector3f direction_eulerAngle = quat.matrix().eulerAngles(2,1,0);
    
    visualization_msgs::MarkerArray msg_fov_marker;
    for(int i=0; i<candidate_waypoints_num; i++)
    {
        int iWidth = int(i%width);
        int iHeight = int(i/width);
        Eigen::Vector3f deltHeight;
        if (belong_ceil == true)
            deltHeight = major_vector * (iHeight*adjacentwaypoints_distance);
        else
            deltHeight = major_vector * (iHeight*adjacentwaypoints_distance);
        Eigen::Vector3f deltaWidth = middle_vector * (iWidth*adjacentpaths_distance); 

        Eigen::Vector3f DistanceToWall = -minor_vector * fov_distance_plane;
        Eigen::Vector3f deltaMove = deltaWidth + deltHeight + DistanceToWall + Start_Position;
        // if(deltaMove[2] <= 0 )
        //     continue;
        std::vector<float> candidate_waypoint_position(9);
        candidate_waypoint_position[0] = deltaMove[0];
        candidate_waypoint_position[1] = deltaMove[1];
        candidate_waypoint_position[2] = deltaMove[2];
        candidate_waypoint_position[3] = direction_eulerAngle[2];
        candidate_waypoint_position[4] = direction_eulerAngle[1];
        candidate_waypoint_position[5] = direction_eulerAngle[0];
        candidate_waypoint_position[6] = direction_eulerAngle[2];
        candidate_waypoint_position[7] = direction_eulerAngle[1];
        candidate_waypoint_position[8] = direction_eulerAngle[0]+M_PI;
        candidate_waypoint_positions.push_back(candidate_waypoint_position);
        std::string name = "sample_wall"+to_string(iMarker+1);
        BuildFoVMarker(quat, deltaMove, i, name, marker_FOV, fov_plane_height);
        // msg_fov_marker.markers.push_back(marker_FOV);
    }
    
    // step 2: generate effective painting waypoint positions and update the octomap
    int iHeight = std::ceil(height/3);
    std::map<int, octomap::OcTreeKey> wall_index_waypoint_octkey;
    std::vector<octomap::OcTreeKey> wall_index_octkey;
    renov_msgs::array3D waypoints_3D;
    thread_processing(  width, candidate_waypoint_positions, pointwall_plane, cloudAndUnknown, 
                        wall_index_waypoint_octkey,wall_index_octkey, plane_marker.markers[iMarker], 
                        false, waypoints_3D);
    std::cout<<"waypoints_3D size is: "<<waypoints_3D.waypoints_index.size()<<std::endl;
    std::vector<std::pair<int, octomap::OcTreeKey>> sWall_index_waypoint_octkey;
    sWall_index_waypoint_octkey.insert(sWall_index_waypoint_octkey.end(),wall_index_waypoint_octkey.begin(),wall_index_waypoint_octkey.end());  
    // std::cout<<"sWall_index_waypoint_octkey.size() is: "<<sWall_index_waypoint_octkey.size()<<std::endl;

    for (size_t i = 0; i < waypoints_3D.FoV.size(); i++)
    {   
        for (size_t j = 0; j < waypoints_3D.FoV[i].coord.size(); j++)
        {
            octomap::OcTreeKey get_key = cloudAndUnknown.coordToKey(waypoints_3D.FoV[i].coord[j].x,
                                                                    waypoints_3D.FoV[i].coord[j].y,
                                                                    waypoints_3D.FoV[i].coord[j].z);
            cloudAndUnknown.updateNode(get_key, false);
        }
    }
    cloudAndUnknown.updateInnerOccupancy();
    // step 3: generate wallsurfacewaypoints_dict and wallsurfacewaypoints_dict_ceil
    int waypoints_mat[width][height];
    for (int iWidth = 0; iWidth < width; iWidth++)
    {
        for (int iHeight = 0; iHeight < height; iHeight++)
            waypoints_mat[iWidth][iHeight]=0;
    }
    for (int waypoint_num=0; waypoint_num<waypoints_3D.waypoints_index.size(); waypoint_num++)
    {
        int waypoint_index = waypoints_3D.waypoints_index[waypoint_num];
        int iWidth = int(waypoint_index%width);
        int iHeight = int(waypoint_index/width);
        waypoints_mat[iWidth][iHeight]=1;
    }
    visualization_msgs::MarkerArray msg_fov_marker2;
    for(size_t i=0; i<sWall_index_waypoint_octkey.size(); i++)
    {
        const int index = sWall_index_waypoint_octkey[i].first;
        tf::Quaternion quat;
        quat.setRPY(candidate_waypoint_positions[index][3], 
                    candidate_waypoint_positions[index][4], 
                    candidate_waypoint_positions[index][5]);
        Eigen::Quaternionf q_eigen(quat.w(), quat.x(),quat.y(),quat.z());
        Eigen::Vector3f t_eigen(candidate_waypoint_positions[index][0],
                                candidate_waypoint_positions[index][1],
                                candidate_waypoint_positions[index][2]);
        std::string name = "cover_wall"+to_string(iMarker+1);
        BuildFoVMarker(q_eigen, t_eigen, i, name, marker_FOV2,fov_plane_height);
        msg_fov_marker2.markers.push_back(marker_FOV2);
        geometry_msgs::Pose pose_marker;
        pose_marker.position.x = t_eigen(0);
        pose_marker.position.y = t_eigen(1);
        pose_marker.position.z = t_eigen(2);            
        pose_marker.orientation.w = quat.w();
        pose_marker.orientation.x = quat.x();
        pose_marker.orientation.y = quat.y();
        pose_marker.orientation.z = quat.z();
        int iWidth = int(index%width);
        int iHeight = int(index/width);
        if (waypoints_mat[iWidth][iHeight]==1)
            wall_posearray.poses.push_back(pose_marker);
    }
        for(size_t i=0; i<sWall_index_waypoint_octkey.size(); i++)
    {
        const int index = sWall_index_waypoint_octkey[i].first;
        tf::Quaternion quat;
        quat.setRPY(candidate_waypoint_positions[index][3], 
                    candidate_waypoint_positions[index][4], 
                    candidate_waypoint_positions[index][5]);
        Eigen::Quaternionf q_eigen(quat.w(), quat.x(),quat.y(),quat.z());
        Eigen::Vector3f t_eigen(candidate_waypoint_positions[index][0],
                                candidate_waypoint_positions[index][1],
                                candidate_waypoint_positions[index][2]);
        std::string name = "cover_wall"+to_string(iMarker+1);
        BuildFoVMarker(q_eigen, t_eigen, i, name, marker_FOV2,fov_plane_height);
        msg_fov_marker2.markers.push_back(marker_FOV2);
        geometry_msgs::Pose pose_marker;
        pose_marker.position.x = t_eigen(0);
        pose_marker.position.y = t_eigen(1);
        pose_marker.position.z = t_eigen(2);            
        pose_marker.orientation.w = quat.w();
        pose_marker.orientation.x = quat.x();
        pose_marker.orientation.y = quat.y();
        pose_marker.orientation.z = quat.z();
        int iWidth = int(index%width);
        int iHeight = int(index/width);
        wall_posearray1.poses.push_back(pose_marker);
    }
    if(belong_ceil)
    {
        std::string str1_1 = "walls_num"+to_string(iMarker1);
        for (int iWidth = 0; iWidth < width; iWidth++)
        {
            for (int iHeight = 0; iHeight < height; iHeight++)
            {
                std::string str2 = "paths_num"+to_string(width-1-iWidth); 
                std::string str3 = "waypoints_num"+to_string(iHeight);
                ceilingsurfacewaypoints_dict1[str1_1][str2][str3]=waypoints_mat[iWidth][iHeight];
            }
        }
        for(int i=0; i<candidate_waypoints_num; i++)
        {
            int iWidth1 = int(i%width);
            int iHeight1 = int(i/width);
            std::string str2 = "paths_num"+to_string(width-1-iWidth1); 
            std::string str3 = "waypoints_num"+to_string(iHeight1);
            for (int dof=0; dof<9; dof++)
            {
                ceilingsurfacewaypoints_dict2[str1_1][str2][str3][dof]=candidate_waypoint_positions[i][dof];
            }
        } 
        iMarker1=iMarker1+1;
    }
    else
    {
        std::string str1_2 = "walls_num"+to_string(iMarker2);
        for (int iWidth = 0; iWidth < width; iWidth++)
        {
            for (int iHeight = 0; iHeight < height; iHeight++)
            {
                std::string str2 = "paths_num"+to_string(width-1-iWidth); 
                std::string str3 = "waypoints_num"+to_string(iHeight);
                wallsurfacewaypoints_dict1[str1_2][str2][str3]=waypoints_mat[iWidth][iHeight];
                // std::cout<<"waypoints_mat[iWidth][iHeight] is: "<<waypoints_mat[iWidth][iHeight]<<std::endl;
            }
        }
        // std::cout<<"candidate_waypoints_num is: "<<candidate_waypoints_num<<std::endl;
        for(int i=0; i<candidate_waypoints_num; i++)
        {
            // std::cout<<"i is: "<<i<<", candidate_waypoint_positions[i][dof] is:"<<candidate_waypoint_positions[i][0]<<std::endl;
            int iWidth1 = int(i%width);
            int iHeight1 = int(i/width);
            std::string str2 = "paths_num"+to_string(width-1-iWidth1); 
            std::string str3 = "waypoints_num"+to_string(iHeight1);
            for (int dof=0; dof<9; dof++)
            {
                wallsurfacewaypoints_dict2[str1_2][str2][str3][dof]=candidate_waypoint_positions[i][dof];
            }
        } 
        iMarker2=iMarker2+1;
    }   
}


void tspsolver(std::vector<std::vector<float>> unordered_wall_positions, 
               std::vector<std::vector<float>>& ordered_wall_positions,
               nav_msgs::Path& path)
{
    ordered_wall_positions.push_back(unordered_wall_positions[0]);
    auto iter = unordered_wall_positions.erase(std::begin(unordered_wall_positions));
    while (1)
    {
        int j=ordered_wall_positions.size();
        int min_index;
        float minlength=10000.0;
        for (int i=0; i<unordered_wall_positions.size(); i++)
        // for(std::vector<std::vector<float>>::iterator it=unordered_wall_positions.begin(); it!=unordered_wall_positions.end(); )
        {
            float deltax=unordered_wall_positions[i][0]-ordered_wall_positions[j-1][0];
            float deltay=unordered_wall_positions[i][1]-ordered_wall_positions[j-1][1];
            float length = sqrt(deltax*deltax+deltay*deltay);
            if (length<minlength)
            {
                minlength=length;
                min_index=i;
            }
            // it++;
        }
        ordered_wall_positions.push_back(unordered_wall_positions[min_index]);
        auto iter = unordered_wall_positions.erase(std::begin(unordered_wall_positions)+min_index);
        if (unordered_wall_positions.size()==0)
            break;
    }
    for (int i=0; i<ordered_wall_positions.size(); i++)
    {
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = ordered_wall_positions[i][0];
        this_pose_stamped.pose.position.y = ordered_wall_positions[i][1];
        this_pose_stamped.pose.position.z = ordered_wall_positions[i][2];
        this_pose_stamped.pose.orientation.x = 0.0;
        this_pose_stamped.pose.orientation.y = 0.0;
        this_pose_stamped.pose.orientation.z = 0.0;
        this_pose_stamped.pose.orientation.w = 1.0;
        this_pose_stamped.header.stamp=ros::Time::now();
        this_pose_stamped.header.frame_id="map";
        path.poses.push_back(this_pose_stamped);
    }    

}

int main(int argc,char**argv)
{
    // initialize ros node and setup ros topics
    ros::init (argc, argv, "waypoint_planning_using_octomap");  
    ros::NodeHandle nh;  
    int hz=1;
    ros::Rate loop_rate(hz);

    // phase 1-step 1: obtiain points cloud of covered wall workspace is shown as follows:
    ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("pcl_output", 10, PointCloudCallBack);
    ros::Subscriber plane_marker_sub = nh.subscribe<visualization_msgs::MarkerArray>("plane_marker", 10, PlaneMarkerCallBack);
    ros::Subscriber corner_marker_sub = nh.subscribe<visualization_msgs::MarkerArray>("corner_marker", 10, CornerMarkerCallBack);
    ros::Publisher wall_pose_array_pub1 = nh.advertise<geometry_msgs::PoseArray>("wall_pose_arrary1", 10);

    ros::Publisher wall_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("wall_pose_arrary", 10);
    ros::Publisher corner_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("corner_pose_arrary", 10);
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("wall_octomap_pub", 10, false);
    ros::Publisher fov_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sample_wall_fov_markers_pub", 1 );
    ros::Publisher fov2_markers_pub1_ = nh.advertise<visualization_msgs::MarkerArray>("wall_fov_markers_pub", 1 );
    ros::Publisher fov2_markers_pub_V_ = nh.advertise<visualization_msgs::MarkerArray>("wall_V_fov_markers_pub", 1 );
    ros::Publisher fov2_markers_pub_H_ = nh.advertise<visualization_msgs::MarkerArray>("wall_H_fov_markers_pub", 1 );
    ros::Publisher fov2_markers_pub_corner = nh.advertise<visualization_msgs::MarkerArray>("corner_fov_markers_pub", 1);
    ros::Publisher waypoints_3D_pub = nh.advertise<renov_msgs::array3D>("waypoints_3D", 10);
    ros::Publisher grid_waypoints_pub = nh.advertise<renov_msgs::array_GridWaypoints>("grid_waypoints", 10);
    ros::Publisher occupancy_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_projected_map", 5, 10);

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("wallsequence",1, true);
    ros::Publisher path_pub1 = nh.advertise<nav_msgs::Path>("ceilingsequence",1, true);


    std::string wallcornerwaypoints_savingpath1 = argv[1];
    std::string wallcornerwaypoints_savingpath2 = argv[2];

    std::string ceilingcornerwaypoints_savingpath1 = argv[3];
    std::string ceilingcornerwaypoints_savingpath2 = argv[4];
    
    std::string wallsurfacewaypoints_savingpath1 = argv[5];
    std::string wallsurfacewaypoints_savingpath2 = argv[6];

    std::string ceilingsurfacewaypoints_savingpath1 = argv[7];
    std::string ceilingsurfacewaypoints_savingpath2 = argv[8];

    Json::Value ceilingcornerwaypoints_dict1, ceilingcornerwaypoints_dict2, ceilingsurfacewaypoints_dict1, ceilingsurfacewaypoints_dict2;
    Json::Value wallcornerwaypoints_dict1, wallcornerwaypoints_dict2, wallsurfacewaypoints_dict1, wallsurfacewaypoints_dict2;

    // renov_msgs::array3D waypoints_3D;
    renov_msgs::array_GridWaypoints array_grid_waypoints;
    std::map<std::pair<int, int>, std::set<int> > grid_waypoints;

    // phase 1-step2: create octree from point cloud of covered wall workspace, Craete OctomapMsg
    octomap_msgs::Octomap octomapMsg;
    octomapMsg.header.frame_id = "map";

    // phase 1-step3: create octree of camera FOV
    float camera_height=fov_plane_height;
    int height_num = int(camera_height/fov_resolution)+1;
    int width_num = int(camera_width/fov_resolution)+1; 
    octomap::point3d Point3dwall_plane(0.0,0.0,camera_depth); 
    octomap::Pointcloud pointwall_plane;  
    for(int ii=0;ii<height_num;ii++){
        for (int iii=0;iii<width_num;iii++){
            Point3dwall_plane.x()= (-camera_height/2.0)+(ii*fov_resolution);       // (-0.15 -> 0.15)
            Point3dwall_plane.y()= (-camera_width/2.0)+(iii*fov_resolution);      // (-0.025 -> 0.025)
            pointwall_plane.push_back(Point3dwall_plane);
        }
    }
    camera_height=fov_corner_height;
    height_num = int(camera_height/fov_resolution)+1;
    width_num = int(camera_width/fov_resolution)+1; 
    octomap::point3d Point3dwall_corner(0.0,0.0,camera_depth1); 
    octomap::Pointcloud pointwall_corner;  
    for(int ii=0;ii<height_num;ii++){
        for (int iii=0;iii<width_num;iii++){
            Point3dwall_corner.x()= (-camera_height/2.0)+(ii*fov_resolution);       // (-0.15 -> 0.15)
            Point3dwall_corner.y()= (-camera_width/2.0)+(iii*fov_resolution);      // (-0.025 -> 0.025)
            pointwall_corner.push_back(Point3dwall_corner);
        }
    }

    ros::Rate waiting_loop(1);
    while((!flag_plane_marker || !flag_corner_marker || !flag_point) && ros::ok())
    {
        ROS_WARN("Waiting for the necessary data to process !!!");
        ros::spinOnce();
        waiting_loop.sleep();
    }

    octomapChangeResolution(cloudAndUnknown);
    geometry_msgs::PoseArray  wall_posearray, corner_posearray;
    wall_posearray.header.frame_id = "map";
    wall_posearray.header.stamp = ros::Time::now();
    corner_posearray.header.frame_id = "map";
    corner_posearray.header.stamp = ros::Time::now();

    // Corner
    std::vector<visualization_msgs::MarkerArray> vmsg_fov_marker_corner;
    std::vector<visualization_msgs::MarkerArray> vmsg_fov_marker2_corner;
    std::string name3 = "FOV_Corner";
    visualization_msgs::Marker marker_FOV_Corner;
    marker_FOV_Corner.header.frame_id = "map";
    marker_FOV_Corner.ns = name3;
    // marker_FOV_Corner.id = 2;
    marker_FOV_Corner.pose.orientation.w = 1;
    marker_FOV_Corner.type = visualization_msgs::Marker::LINE_LIST;
    marker_FOV_Corner.action=visualization_msgs::Marker::ADD;
    marker_FOV_Corner.scale.x=0.01;//0.2; 0.03
    marker_FOV_Corner.color.r = 1.0f;
    marker_FOV_Corner.color.a = 0.8;
    std::string name4 = "FOV2_Corner";
    visualization_msgs::Marker marker_FOV2_Corner;
    marker_FOV2_Corner.header.frame_id = "map";
    marker_FOV2_Corner.ns = name4;
    // marker_FOV2_Corner.id = 3;
    marker_FOV2_Corner.pose.orientation.w = 1;
    marker_FOV2_Corner.type = visualization_msgs::Marker::LINE_LIST;
    marker_FOV2_Corner.action=visualization_msgs::Marker::ADD;
    marker_FOV2_Corner.scale.x=0.01;//0.2; 0.03
    marker_FOV2_Corner.color.r = 1.0f;
    marker_FOV2_Corner.color.g = 1.0f;
    marker_FOV2_Corner.color.a = 0.8;

    std::vector<int> wallcorner_markerindex, ceilcorner_markerindex;
    std::vector<std::vector<float>> wallcorner_positions, ceilcorner_positions;
    for (size_t iMarker = 0; iMarker < corner_marker.markers.size(); iMarker++)
    {
        wallceilingcorner_division(iMarker,
                                   corner_marker,
                                   wallcorner_markerindex, 
                                   ceilcorner_markerindex,
                                   wallcorner_positions, 
                                   ceilcorner_positions);
    }
    int iMarker1, iMarker2;
    iMarker1=0, iMarker2=0;
    for (size_t index = 0; index < ceilcorner_markerindex.size(); index++)
    {
        int ceilcorner_marker=ceilcorner_markerindex[index];
        cornerwaypoints_generation( ceilcorner_marker, 
                                    iMarker1,
                                    iMarker2,
                                    cloudAndUnknown,
                                    pointwall_corner,
                                    corner_marker,
                                    marker_FOV_Corner,
                                    marker_FOV2_Corner, 
                                    corner_posearray,
                                    vmsg_fov_marker_corner, 
                                    vmsg_fov_marker2_corner, 
                                    ceilingcornerwaypoints_dict1,
                                    ceilingcornerwaypoints_dict2,
                                    wallcornerwaypoints_dict1, 
                                    wallcornerwaypoints_dict2); 
    }    
    iMarker1=0, iMarker2=0;
    for (size_t index = 0; index < wallcorner_markerindex.size(); index++)
    {
        int ceilcorner_marker=wallcorner_markerindex[index];
        cornerwaypoints_generation( ceilcorner_marker, 
                                    iMarker1,
                                    iMarker2,
                                    cloudAndUnknown,
                                    pointwall_corner,
                                    corner_marker,
                                    marker_FOV_Corner,
                                    marker_FOV2_Corner, 
                                    corner_posearray,
                                    vmsg_fov_marker_corner, 
                                    vmsg_fov_marker2_corner, 
                                    ceilingcornerwaypoints_dict1,
                                    ceilingcornerwaypoints_dict2,
                                    wallcornerwaypoints_dict1, 
                                    wallcornerwaypoints_dict2); 
    }    
    std::ofstream ofs1_ceil(ceilingcornerwaypoints_savingpath1);
    ofs1_ceil << ceilingcornerwaypoints_dict1;
    ofs1_ceil.close();
    std::ofstream ofs3_ceil(ceilingcornerwaypoints_savingpath2);
    ofs3_ceil << ceilingcornerwaypoints_dict2;
    ofs3_ceil.close();
    std::ofstream ofs1(wallcornerwaypoints_savingpath1);
    ofs1 << wallcornerwaypoints_dict1;
    ofs1.close();
    std::ofstream ofs3(wallcornerwaypoints_savingpath2);
    ofs3 << wallcornerwaypoints_dict2;
    ofs3.close();
    ROS_INFO("Save corner json file");

    // Plane
    bool plane_flag=false;
    std::vector<visualization_msgs::MarkerArray> vmsg_fov_marker;
    std::vector<visualization_msgs::MarkerArray> vmsg_fov_marker2;
    std::vector<visualization_msgs::MarkerArray> vmsg_fov_marker_V;
    std::vector<visualization_msgs::MarkerArray> vmsg_fov_marker_H;
    std::string name = "FOV";
    visualization_msgs::Marker marker_FOV;
    marker_FOV.header.frame_id = "map";
    marker_FOV.ns = name;
    // marker_FOV.id = 0;
    marker_FOV.pose.orientation.w = 1;
    marker_FOV.type = visualization_msgs::Marker::LINE_LIST;
    marker_FOV.action=visualization_msgs::Marker::ADD;
    marker_FOV.scale.x=0.02;//0.2; 0.03
    marker_FOV.scale.y=0.02;//0.2; 0.03
    marker_FOV.scale.z=0.02;//0.2; 0.03
    marker_FOV.color.b = 1.0f;
    marker_FOV.color.a = 1.0;
    std::string name2 = "FOV2";
    visualization_msgs::Marker marker_FOV2;
    marker_FOV2.header.frame_id = "map";
    marker_FOV2.ns = name2;
    // marker_FOV2.id = 1;
    marker_FOV2.pose.orientation.w = 1;
    marker_FOV2.type = visualization_msgs::Marker::LINE_LIST;
    marker_FOV2.action=visualization_msgs::Marker::ADD;
    marker_FOV2.scale.x=0.02;//0.2; 0.03
    marker_FOV2.scale.y=0.02;//0.2; 0.03
    marker_FOV2.scale.z=0.02;//0.2; 0.03
    marker_FOV2.color.g = 1.0f;
    marker_FOV2.color.a = 1.0;
    std::vector<int> wallsurface_markerindex, ceilsurface_markerindex;
    std::vector<std::vector<float>> wallsurface_positions, ceilsurface_positions;
    for (size_t iMarker = 0; iMarker < plane_marker.markers.size(); iMarker++)
    {
        wallceilingsurface_division(iMarker,
                                    plane_marker,
                                    wallsurface_markerindex, 
                                    ceilsurface_markerindex,
                                    wallsurface_positions, 
                                    ceilsurface_positions);
    }
    geometry_msgs::PoseArray wall_posearray1;
    wall_posearray1.header.frame_id = "map";
    wall_posearray1.header.stamp = ros::Time::now();
    
    iMarker1=0, iMarker2=0;
    for (size_t index = 0; index < wallsurface_markerindex.size(); index++)
    {
        int wallsurface_marker=wallsurface_markerindex[index];
        surfacewaypoints_generation(wallsurface_marker, 
                                    iMarker1,
                                    iMarker2,
                                    cloudAndUnknown,
                                    pointwall_plane,
                                    plane_marker,
                                    marker_FOV,
                                    marker_FOV2,
                                    wall_posearray,
                                    wall_posearray1,
                                    ceilingsurfacewaypoints_dict1,
                                    ceilingsurfacewaypoints_dict2,
                                    wallsurfacewaypoints_dict1, 
                                    wallsurfacewaypoints_dict2);
    }
    iMarker1=0, iMarker2=0;
    for (size_t index = 0; index < ceilsurface_markerindex.size(); index++)
    {
        int ceilsurface_marker=ceilsurface_markerindex[index];
        surfacewaypoints_generation(ceilsurface_marker, 
                                    iMarker1,
                                    iMarker2,
                                    cloudAndUnknown,
                                    pointwall_plane,
                                    plane_marker,
                                    marker_FOV,
                                    marker_FOV2,
                                    wall_posearray,
                                    wall_posearray1,
                                    ceilingsurfacewaypoints_dict1,
                                    ceilingsurfacewaypoints_dict2,
                                    wallsurfacewaypoints_dict1, 
                                    wallsurfacewaypoints_dict2);
    }
    std::ofstream ofs2_ceil(ceilingsurfacewaypoints_savingpath1);
    ofs2_ceil << ceilingsurfacewaypoints_dict1;
    ofs2_ceil.close();
    std::ofstream ofs4_ceil(ceilingsurfacewaypoints_savingpath2);
    ofs4_ceil << ceilingsurfacewaypoints_dict2;
    ofs4_ceil.close();
    std::ofstream ofs2(wallsurfacewaypoints_savingpath1);
    ofs2 << wallsurfacewaypoints_dict1;
    ofs2.close();
    std::ofstream ofs4(wallsurfacewaypoints_savingpath2);
    ofs4 << wallsurfacewaypoints_dict2;
    ofs4.close();
    ROS_INFO("Save surface json file");


    // std::vector<std::vector<float>> unordered_wall_positions, ordered_wall_positions;
    // int walls_num = wallcorner_positions.size()+wallsurface_positions.size();
    // for (int i=0; i<walls_num; i++)
    // {
    //     if (i<wallcorner_positions.size())
    //         unordered_wall_positions.push_back(wallcorner_positions[i]);
    //     else
    //         unordered_wall_positions.push_back(wallsurface_positions[i-wallcorner_positions.size()]);
    // }
    // nav_msgs::Path path;
    // path.header.stamp=ros::Time::now();
    // path.header.frame_id="map";
    // tspsolver(unordered_wall_positions, ordered_wall_positions, path);
    // std::vector<std::vector<float>> ordered_ceilcorner_positions;
    // nav_msgs::Path path1;
    // path1.header.stamp=ros::Time::now();
    // path1.header.frame_id="map";
    // tspsolver(ceilcorner_positions, ordered_ceilcorner_positions, path1);


    int count = 0;
    // fov_markers_pub_.publish(vmsg_fov_marker[vmsg_fov_marker.size()-1]);
    // fov2_markers_pub1_.publish(vmsg_fov_marker2[vmsg_fov_marker2.size()-1]);
    // fov2_markers_pub_corner.publish(vmsg_fov_marker2_corner[vmsg_fov_marker2_corner.size()-1]);
    octomap_msgs::binaryMapToMsg(cloudAndUnknown, octomapMsg);
    while (ros::ok())
    {
        // if(count%10 == 0)
        // {
        //     fov_markers_pub_.publish(vmsg_fov_marker[vmsg_fov_marker.size()-1]);
        //     fov2_markers_pub1_.publish(vmsg_fov_marker2[vmsg_fov_marker2.size()-1]);
        //     fov2_markers_pub_V_.publish(vmsg_fov_marker_V[vmsg_fov_marker_V.size()-1]);
        //     // fov2_markers_pub_H_.publish(vmsg_fov_marker_H[vmsg_fov_marker_H.size()-1]);
        //     fov2_markers_pub_corner.publish(vmsg_fov_marker2_corner[vmsg_fov_marker2_corner.size()-1]);
        // }

        // path_pub.publish(path);
        // path_pub1.publish(path1);

        // wall_pose_array_pub.publish(wall_posearray);
        // wall_pose_array_pub1.publish(wall_posearray1);

        corner_pose_array_pub.publish(corner_posearray);
        octomap_pub.publish(octomapMsg);
        occupancy_pub_.publish(OccupancyGrid_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}
