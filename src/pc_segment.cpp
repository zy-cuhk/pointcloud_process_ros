#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

using namespace std;
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
                                                     std::vector <pcl::PointIndices> &clusters){
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
    pass.setFilterLimits (0.1, 3.0);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (1);
    reg.setMaxClusterSize (10000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (40);
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (2 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);
    reg.extract (clusters);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    std::cout << "points num of colored cloud: " << colored_cloud->points.size() << std::endl;

    // build xyzrpb normal cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointXYZRGBNormal>) ;
    // cloud_normal->width = cloud->width;
    // cloud_normal->height = cloud->height;
    // cloud_normal->resize(cloud->width*cloud->height);
    for (size_t i = 0; i < cloud->points.size(); i++){
        pcl::PointXYZRGBNormal p_normal;
        p_normal.x = cloud->points[i].x;
        p_normal.y = cloud->points[i].y;
        p_normal.z = cloud->points[i].z;
        p_normal.r = cloud->points[i].r;
        p_normal.g = cloud->points[i].g;
        p_normal.b = cloud->points[i].b;
        p_normal.normal_x = normals->points[i].normal_x;
        p_normal.normal_y = normals->points[i].normal_y;
        p_normal.normal_z = normals->points[i].normal_z;
        p_normal.curvature = normals->points[i].curvature;
        cloud_normal->push_back(p_normal);
    }

    // plane cloud ---------------------------------------------------------------------
    pcl::PointIndices plane_index;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        plane_index.indices.insert(plane_index.indices.end(), clusters[i].indices.begin(), clusters[i].indices.end());
    }
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> plane_points_extract;
    // Extract the inliers
    plane_points_extract.setInputCloud (cloud_normal);
    pcl::IndicesPtr plane_index_ptr = boost::make_shared<std::vector<int>>(plane_index.indices);
    plane_points_extract.setIndices (plane_index_ptr);
    plane_points_extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
    plane_points_extract.filter (*cloud_plane_normal);
    std::cout << "cloud_plane_normal size:" << cloud_plane_normal->points.size() << std::endl;

    // corner cloud -----------------------------------------------------------------
    // Extract the inliers
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> corner_points_extract;
    corner_points_extract.setInputCloud (cloud_normal);
    corner_points_extract.setIndices (plane_index_ptr);
    corner_points_extract.setNegative (true);//如果设为true,可以提取指定index之外的点云
    corner_points_extract.filter (*cloud_corner_normal);
    std::cout << "cloud_corner_normal size:" << cloud_corner_normal->points.size() << std::endl;

    return colored_cloud;
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "PointSeg");
    ros::NodeHandle nh;

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    ros::Publisher pcl_corner_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_corner_output", 1);
    ros::Publisher pcl_seg_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_seg_output", 1);

    // std::string pcd_path = "/home/k/catkin_ws/src/bim_reconstruction/data/2022-08-17_20:45:45/output_totalpointcloud.pcd";
    // std::string pcd_path = "/home/k/catkin_ws/src/bim_reconstruction/data/2022-08-11_20:20:55/output_totalpointcloud.pcd";

    std::string pcd_path = "/home/k/catkin_ws/src/bim_reconstruction/data/2022-07-29_00:35:34/output_totalpointcloud.pcd";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (pcd_path, *cloud) == -1)
    {
        std::cout << "Cloud1 reading failed." << std::endl;
        return (-1);
    }
    
    // Downsample
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    // cloud_downsampled = DownSample(cloud, 0.01);

    // Outlier removal
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliner(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    // sor.setInputCloud(cloud_downsampled);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(5.0);
    // sor.filter(*cloud_inliner);

    // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // outrem.setInputCloud(cloud_downsampled);
    // outrem.setRadiusSearch(0.1);
    // outrem.setMinNeighborsInRadius (10);
    // outrem.filter (*cloud_inliner);

    // filter in the vertical direction, obtain the global point cloud 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_zfilter(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> pass1;
    pass1.setInputCloud (cloud);
    pass1.setFilterFieldName ("z");
    pass1.setFilterLimits (2.2, 2.7);
	pass1.filter(*cloud_zfilter);
    sensor_msgs::PointCloud2 output_pointcloud;
    pcl::toROSMsg(*cloud_zfilter, output_pointcloud);
    output_pointcloud.header.frame_id = "map";
    std::cout << "Filterd size: " << output_pointcloud.data.size() << std::endl;

    // segment the global point cloud into point cloud clusters of interior walls
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_corner (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    std::vector <pcl::PointIndices> clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = RegionGrowing(cloud_zfilter, cloud_plane, cloud_corner,  clusters);
    std::cout << "Number of clusters: " << clusters.size () << std::endl;
    sensor_msgs::PointCloud2 output_pointcloud_seg;
    pcl::toROSMsg(*colored_cloud, output_pointcloud_seg);
    output_pointcloud_seg.header.frame_id = "map";
    std::cout << "Filterd size: " << output_pointcloud_seg.data.size() << std::endl;

    cout<<"the size is: "<<cloud_corner->size()<<endl;
    sensor_msgs::PointCloud2 output_pointcloud_corner;
    pcl::toROSMsg(*cloud_corner, output_pointcloud_corner);
    output_pointcloud_corner.header.frame_id = "map";
    std::cout << "the point cloud of corner size: " << output_pointcloud_corner.data.size() << std::endl;

    // Create the filtering object
    // sensor_msgs::PointCloud2 output_pointcloud_v;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_v (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // extract.setInputCloud (colored_cloud);
    // extract.setIndices (indices);
    // extract.setNegative (false);
    // extract.filter (*cloud_plane_v);
    // pcl::toROSMsg(*cloud_plane_v, output_pointcloud_v);
    // output_pointcloud_v.header.frame_id = "map";
    // std::cout << "Filterd size: " << output_pointcloud_v.data.size() << std::endl;

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output_pointcloud);
        pcl_seg_pub.publish(output_pointcloud_seg);
        pcl_corner_pub.publish(output_pointcloud_corner);

        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
}