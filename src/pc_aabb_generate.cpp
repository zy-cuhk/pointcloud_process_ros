#include <vector>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <cmath>
#include <ctime>
#include <thread>
#include <iostream>
#include <fstream>
#include <thread>
#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
int main(int argc, char** argv)
{
    ros::init (argc, argv, "Point_boundingbox_Pub");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    ros::Publisher plane_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("plane_marker", 10 );
    ros::Publisher frontiers_orientation_pub = nh.advertise<visualization_msgs::MarkerArray>("boundingbox_orientation", 10, true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
    //     return (-1);

    for (int x=30; x>-30; x--) {
        for (int y=30; y>-30; y--) {
            pcl::PointXYZ endpoint; 
            endpoint.x = x*0.05f;
            endpoint.y = y*0.05f;
            endpoint.z = 0.0;
            cloud->push_back(endpoint);
        }
    }

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();
 
    std::vector <float> moment_of_inertia, eccentricity;
    pcl::PointXYZ min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Vector3f mass_center;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;

    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);
    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);

    Eigen::Vector3f unitvector;
    unitvector<<0, 1, 0;
    Eigen::Vector3f normvector = rotational_matrix_OBB*unitvector;

    visualization_msgs::MarkerArray plane_msg_marker;
    for (size_t i = 0; i < 1; i++)
    {
        visualization_msgs::Marker bbx_marker;
        bbx_marker.header.frame_id = "map";
        bbx_marker.header.stamp = ros::Time::now();
        bbx_marker.ns = "plane" + i;
        bbx_marker.type = visualization_msgs::Marker::CUBE;
        bbx_marker.action = visualization_msgs::Marker::ADD;
        bbx_marker.pose.position.x =  position[0];
        bbx_marker.pose.position.y =  position[1];
        bbx_marker.pose.position.z =  position[2];
        bbx_marker.pose.orientation.x = quat.x();
        bbx_marker.pose.orientation.y = quat.y();
        bbx_marker.pose.orientation.z = quat.z();
        bbx_marker.pose.orientation.w = quat.w();
        bbx_marker.scale.x = std::fabs(max_point_OBB.x - min_point_OBB.x);
        bbx_marker.scale.y = std::fabs(max_point_OBB.y - min_point_OBB.y);
        bbx_marker.scale.z = std::fabs(max_point_OBB.z - min_point_OBB.z);
        bbx_marker.color.b = 0;
        bbx_marker.color.g = 255;
        bbx_marker.color.r = 0;
        bbx_marker.color.a = 0.7;
        plane_msg_marker.markers.push_back(bbx_marker);
    }

    visualization_msgs::MarkerArray bbx_normvector_marker;
    visualization_msgs::Marker marker_FOV;
    marker_FOV.header.frame_id = "map";
    marker_FOV.pose.orientation.w = 1;
    marker_FOV.type = visualization_msgs::Marker::LINE_LIST;
    marker_FOV.action = visualization_msgs::Marker::ADD;
    marker_FOV.scale.x = 0.1;
    marker_FOV.scale.z = 0.4;
    const Eigen::Vector3f rgb(0.0, 0.0, 0.0);
    marker_FOV.color.r = rgb(0);
    marker_FOV.color.g = rgb(1);
    marker_FOV.color.b = rgb(2);
    marker_FOV.color.a = 1.0;
    const std::string name="waypoint"+to_string(0);
    marker_FOV.ns = name;
    geometry_msgs::Point msgs_p1, msgs_p2;
    msgs_p1.x = position[0];
    msgs_p1.y = position[1];
    msgs_p1.z = position[2];
    msgs_p2.x = position[0] + normvector.x();
    msgs_p2.y = position[1] + normvector.y();
    msgs_p2.z = position[2] + normvector.z();
    marker_FOV.points.push_back(msgs_p1);
    marker_FOV.points.push_back(msgs_p2);
    marker_FOV.header.stamp = ros::Time::now();
    bbx_normvector_marker.markers.push_back(marker_FOV);


    sensor_msgs::PointCloud2 output_pointcloud;
    pcl::toROSMsg(*cloud, output_pointcloud);
    output_pointcloud.header.frame_id = "map";

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        plane_markers_pub_.publish(plane_msg_marker);
        frontiers_orientation_pub.publish(bbx_normvector_marker);
        pcl_pub.publish(output_pointcloud);
        // ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
}