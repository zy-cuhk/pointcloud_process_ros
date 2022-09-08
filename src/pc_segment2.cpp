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
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>


#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
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
    pass.setFilterLimits (0.2, 2.7);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (10000);
    reg.setMaxClusterSize (100000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (40);
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (5 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);
    reg.extract (clusters);

    // build xyzrpy normal cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointXYZRGBNormal>) ;
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
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

    pcl::PointIndices plane_index;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        plane_index.indices.insert(plane_index.indices.end(), clusters[i].indices.begin(), clusters[i].indices.end());
    }

    // plane cloud ---------------------------------------------------------------------
    // Extract the inliers
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> plane_points_extract;
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
    map.info.origin.position.y = origin.y() - octree.getResolution() * 0.5-0.1;

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
                        map.data[map.info.width * j + i] = 1;
                    }
                    // else 
                    // {
                    //     map.data[map.info.width * j + i] = 1;
                    // }
                }
            }
        }
    }
}


void ComputeOBB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                // pcl::IndicesPtr plane_indice,
                pcl::PointXYZRGB &min_point_OBB,
                pcl::PointXYZRGB &max_point_OBB,
                pcl::PointXYZRGB &position_OBB,
                Eigen::Matrix3f &rotational_matrix_OBB,
                Eigen::Vector3f &major_vector, 
                Eigen::Vector3f &middle_vector, 
                Eigen::Vector3f &minor_vector)
{

    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);

}

pcl::ModelCoefficients::Ptr PlaneRefine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.05);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    // Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_plane);

    return coefficients;

}

void SegmentWallCloud(const std::vector <pcl::PointIndices> &clusters, 
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_refine_vertical,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_refine_horizontal,
                                std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &vCloud_Vertical_Patch_Ptr,
                                std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &vCloud_Horizontal_Patch_Ptr,
                                std::vector<Eigen::Matrix3f> &vPlane_Normal_Vertical,
                                std::vector<Eigen::Vector3f> &vPlane_Orign_Vertical,
                                std::vector<Eigen::Vector3f> &vPlane_Scale_Vertical,
                                std::vector<Eigen::Matrix3f> &vPlane_Normal_Horizontal,
                                std::vector<Eigen::Vector3f> &vPlane_Orign_Horizontal,
                                std::vector<Eigen::Vector3f> &vPlane_Scale_Horizontal)
{

    float longest_wall = 0;
    Eigen::Matrix3f longest_Normal = Eigen::Matrix3f::Identity();
    
    for (size_t i = 0; i < clusters.size(); i++)
    {
        // ComputeOBB
        pcl::PointXYZRGB min_point_OBB, max_point_OBB;
        pcl::PointXYZRGB position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;

        pcl::PointIndices plane_index = clusters[i];
        pcl::IndicesPtr plane_index_ptr = boost::make_shared<std::vector<int>>(plane_index.indices);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);

        // Create the extracted plane
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;

        // Extract the inliers
        extract.setInputCloud(cloud);
        extract.setIndices(plane_index_ptr);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Patch_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Patch_Ptr2(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::ModelCoefficients::Ptr coefficients = PlaneRefine(cloud_plane, cloud_Patch_Ptr);

        ComputeOBB(cloud_Patch_Ptr,
                min_point_OBB,
                max_point_OBB,
                position_OBB,
                rotational_matrix_OBB,
                major_vector,
                middle_vector,
                minor_vector );

        // Aligned Axis
        // minor_vector[2] = 0;
        // minor_vector = minor_vector.normalized();
        Eigen::Vector3f direction(position_OBB.x, position_OBB.y, position_OBB.z);
        direction.normalized();
        if(direction.dot(minor_vector) < 0)
            minor_vector = -minor_vector;

        Eigen::Vector3f z_axis(0,0,1);
        if(minor_vector.dot(z_axis) < 0.5)
        {
            major_vector << 0,0,-1;
            minor_vector[2] = 0;
        }
        // else
        // {
        //     minor_vector << 0,0,1;
        // }

        minor_vector = minor_vector.normalized();
        
        middle_vector = minor_vector.cross(major_vector);
        middle_vector = middle_vector.normalized();

        // Eigen::Vector3f vScale(max_point_OBB.x - min_point_OBB.x,
        //                        max_point_OBB.y - min_point_OBB.y,
        //                        max_point_OBB.z - min_point_OBB.z);
        // Eigen::Matrix3f tansfer_matrix = rotational_matrix_OBB_aligned.inverse()*rotational_matrix_OBB;
        // vScale = tansfer_matrix*vScale;

        pcl::PointXYZRGB min;
        pcl::PointXYZRGB max;
        pcl::getMinMax3D(*cloud_Patch_Ptr,min,max);
        
        Eigen::Vector3f vScale = Eigen::Vector3f::Zero();
        if(minor_vector.dot(z_axis) < 0.5)
        {
            // vScale[0] = (max.z - min.z);
            vScale[0] = (max.z - 0);
            vScale[1] = sqrt(pow((max.x - min.x),2)+pow((max.y - min.y),2));
            vScale[2] = 0.01;
        }
        else
        {
            // vScale[0] = 1.0*fabs(max.x - min.x);
            // vScale[1] = 1.0*fabs(max.y - min.y);
            vScale[0] = (max_point_OBB.x - min_point_OBB.x );
            vScale[1] = (max_point_OBB.y - min_point_OBB.y );
            vScale[2] = 0.01;
        }
        
        Eigen::Vector3f OBB_Position(0.5*(max.x + min.x),
                                     0.5*(max.y + min.y),
                                     0.5*(max.z + min.z));

        Eigen::Matrix3f rotational_matrix_OBB_aligned;
        rotational_matrix_OBB_aligned << major_vector, middle_vector, minor_vector;

        bool c1 = vScale[0] > 2.4;
        // bool c2 = vScale[0] > 1.0;
        // if(minor_vector[2] == 0 && vScale[0] > 2.4)
        // if(minor_vector[2] == 0 && (c1  || c2 ))
        // if(minor_vector[2] == 0 && (c1))
        if(minor_vector[2] == 0)
        {
            if(vScale[1] > longest_wall)
            {
                longest_wall = vScale[1];
                longest_Normal = rotational_matrix_OBB_aligned;
            }

            // vScale[0] = 2.4;
            // OBB_Position[2] = 1.2;

            *cloud_refine_vertical += *cloud_Patch_Ptr;
            vCloud_Vertical_Patch_Ptr.push_back(cloud_Patch_Ptr);
            vPlane_Normal_Vertical.push_back(rotational_matrix_OBB_aligned);
            vPlane_Orign_Vertical.push_back(OBB_Position);
            vPlane_Scale_Vertical.push_back(vScale);
        }

        // if(minor_vector[2] > 0.9 && OBB_Position[2] > 2.4)
        if(minor_vector[2] > 0.9)
        {
            *cloud_refine_horizontal += *cloud_Patch_Ptr;
            vCloud_Horizontal_Patch_Ptr.push_back(cloud_Patch_Ptr);
            vPlane_Normal_Horizontal.push_back(rotational_matrix_OBB_aligned);
            vPlane_Orign_Horizontal.push_back(OBB_Position);
            vPlane_Scale_Horizontal.push_back(vScale);
        }
    }

    // adapt rotation angle of Ceil Plane
    for (size_t i = 0; i < vPlane_Normal_Horizontal.size(); i++)
    {
        Eigen::Matrix3f ceil_normal = vPlane_Normal_Horizontal[i];
        Eigen::Vector3f z_axis = ceil_normal.col(2);
        Eigen::Vector3f y_axis = longest_Normal.col(2);
        Eigen::Vector3f x_axis = (y_axis.cross(z_axis)).normalized();
        ceil_normal.setZero();
        ceil_normal << x_axis, y_axis, z_axis;
        vPlane_Normal_Horizontal[i] = ceil_normal;
    }
}

void Clost_Plane(pcl::PointXYZRGBNormal corner_point,
                 const std::vector<Eigen::Vector3f> &vPlane_Origin,
                 const std::vector<Eigen::Matrix3f> &vPlane_Normal,
                 const std::vector<Eigen::Vector3f> &vPlane_Scale,
                 std::pair<int,int> &clost_plane_index)
{

    pcl::PointXYZRGBNormal p = corner_point;

    float min_dist = 0.1, submin_dist = 0.1;
    float min_dist_index = -1, submin_dist_index = -1;

    Eigen::Vector3f z_axis(0,0,1);
    for (size_t i = 0; i < vPlane_Origin.size(); i++)
    {
        Eigen::Quaternionf quat(vPlane_Normal[i]);
        Eigen::Vector3f trans = vPlane_Origin[i];

        Eigen::Isometry3f T_wall = Eigen::Isometry3f::Identity();
        T_wall.prerotate(quat);
        T_wall.pretranslate(trans);

        Eigen::Vector3f p_to_wall = T_wall.inverse() * Eigen::Vector3f(p.x, p.y, p.z);
        
        // float dist_to_origin = fabs(fabs(vPlane_Scale[i][1]/2) - std::sqrt(pow(p.x-vPlane_Origin[i][0],2) + pow(p.y-vPlane_Origin[i][1],2)));
        // if(fabs(p_to_wall[0]) > vPlane_Scale[i][0]/2 || fabs(p_to_wall[1]) > vPlane_Scale[i][1]/2 ||
        //                     fabs(p_to_wall[1]) > vPlane_Scale[i][2]/2)
        //     continue;

        // float d = fabs(fabs(vPlane_Scale[i][1]/2) - std::sqrt(pow(p_to_wall[0],2) + pow(p_to_wall[1],2)));
        
        float d_edge = 0;
        if(vPlane_Normal[i].col(2).dot(z_axis) < 0.1)
        {
            if(vPlane_Scale[i][1]/2 + 0.05 > fabs(p_to_wall[1]))
                d_edge = std::min(fabs(vPlane_Scale[i][1]/2 -  fabs(p_to_wall[1])),  fabs(vPlane_Scale[i][0]/2 -  fabs(p_to_wall[0])));
            else
                d_edge = 100;
        }
        // else if(vPlane_Normal[i].col(2).dot(z_axis) > 0.9)
        // {

        // }
        
        float d = fabs(p_to_wall[2]) + 2*d_edge;
        // std::cout << "d: " << d << std::endl;

        if(d <= min_dist)
        {
            submin_dist = min_dist;
            min_dist = d;

            submin_dist_index = min_dist_index;
            min_dist_index = i;
        }
        else if(d <= submin_dist)
        {
            submin_dist = d;
            submin_dist_index = i;
        }
    }

    if(min_dist_index > submin_dist_index)
    {
        clost_plane_index = std::make_pair(submin_dist_index, min_dist_index);
    }  
    else
    {
        clost_plane_index = std::make_pair(min_dist_index, submin_dist_index);
    }
}

pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr SegmentCornerCloud(
                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_corner,
                        const std::vector<Eigen::Vector3f> vPlane_Origin,
                        const std::vector<Eigen::Matrix3f> vPlane_Normal,
                        const std::vector<Eigen::Vector3f> vPlane_Scale,
                        std::vector<Eigen::Vector3f> &vCorner_Origin,
                        std::vector<Eigen::Matrix3f> &vCorner_Normal,
                        std::vector<Eigen::Vector3f> &vCorner_Scale)
{

    pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr corner_points (new pcl::PointCloud <pcl::PointXYZRGBNormal>);
    std::vector< std::pair<int,int> > vClost_plane_index;
    for (size_t i = 0; i < cloud_corner->points.size(); i++)
    {
        // if(cloud_corner->points[i].curvature > 0.1)
        if(true)
        {

            std::pair<int,int> clost_plane_index;
            Clost_Plane(cloud_corner->at(i), vPlane_Origin, vPlane_Normal, vPlane_Scale, clost_plane_index);

            if(clost_plane_index.first != -1)
            {
                corner_points->push_back(cloud_corner->at(i));
                vClost_plane_index.push_back(clost_plane_index);
            }
        }
    }
    // std::cout << "vClost_plane_index size: " << vClost_plane_index.size() << std::endl;
    std::map<std::pair<int,int>,int> map_clost_plane_index;
    for (size_t i = 0; i < corner_points->size(); i++)
    {
        Eigen::Vector3f nomal_mean = (vPlane_Normal[vClost_plane_index[i].first].col(2)
                                        +vPlane_Normal[vClost_plane_index[i].second].col(2))/2;

        corner_points->points[i].normal_x = nomal_mean[0];
        corner_points->points[i].normal_y = nomal_mean[1];
        corner_points->points[i].normal_z = nomal_mean[2];

        map_clost_plane_index.insert(std::make_pair(vClost_plane_index[i],i));
    }

    // std::cout << "size corner: " << map_clost_plane_index.size() << std::endl;
    std::vector<std::pair<int,int> > vpPlaneIndex;
    std::map<std::pair<int,int>, int>::iterator iter;
    for (iter = map_clost_plane_index.begin(); iter != map_clost_plane_index.end(); iter++)  
    {
        // std::cout<<iter->first.first<<' '<<iter->first.second<<std::endl;
        Eigen::Vector3f nomal_mean = (vPlane_Normal[iter->first.first].col(2)+vPlane_Normal[iter->first.second].col(2))/2;
        // std::cout << "nomal_mean: \n" << nomal_mean << std::endl;
        vpPlaneIndex.push_back(iter->first);
    }

    // cluster
    for (size_t i = 0; i < vpPlaneIndex.size(); i++)
    {
        Eigen::Vector3f mean_normal = (vPlane_Normal[vpPlaneIndex[i].first].col(2)
                                        +vPlane_Normal[vpPlaneIndex[i].second].col(2))/2;

        float max_x = -100, max_y = -100, max_z = -100;
        float min_x = 100, min_y = 100, min_z = 100;

        for (size_t j = 0; j < corner_points->points.size(); j++)
        {

            Eigen::Vector3f n(corner_points->points[j].normal_x,
                              corner_points->points[j].normal_y,
                              corner_points->points[j].normal_z);

            if(n == mean_normal)
            {
                // iCloud_corner->push_back(corner_points->points[j]);

                float px = corner_points->points[j].x;
                float py = corner_points->points[j].y;
                float pz = corner_points->points[j].z;
                
                if(px > max_x)
                    max_x = px;
                if(px < min_x)
                    min_x = px;

                if(py > max_y)
                    max_y = py;
                if(py < min_y)
                    min_y = py;

                if(pz > max_z)
                    max_z = pz;
                if(pz < min_z)
                    min_z = pz;
                    
            }
        }

        Eigen::Vector3f minor_axis = mean_normal;
        minor_axis.normalize();
        Eigen::Vector3f major_axis(0,0,-1);
        Eigen::Vector3f middle_axis = minor_axis.cross(major_axis);
        middle_axis.normalize();
        major_axis = middle_axis.cross(minor_axis);
        major_axis.normalize();

        Eigen::Matrix3f rotation_matrix;
        rotation_matrix << major_axis, middle_axis, minor_axis;
        vCorner_Normal.push_back(rotation_matrix);
        // vCorner_Scale.push_back(Eigen::Vector3f(fabs(max_z-min_z), 0.01, fabs(max_x-min_x)));
        // vCorner_Origin.push_back(Eigen::Vector3f((max_x+min_x)/2, (max_y+min_y)/2, (max_z+min_z)/2));
        // vCloud_Corner_Ptr.push_back(iCloud_corner);

        Eigen::Vector3f z_axis(0,0,1);
        if(mean_normal.dot(z_axis) < 0.1 )
            vCorner_Scale.push_back(Eigen::Vector3f(fabs(max_z-min_z), 0.01, fabs(max_x-min_x)));
        else
            vCorner_Scale.push_back( Eigen::Vector3f( 0.01, 
                                                      sqrt(pow((max_x-min_x),2) + pow((max_y-min_y),2)), 
                                                      0.1));

        vCorner_Origin.push_back(Eigen::Vector3f((max_x+min_x)/2, (max_y+min_y)/2, (max_z+min_z)/2));
    }
    return corner_points;
}


int main(int argc, char** argv)
{
    ros::init (argc, argv, "PointSeg1");
    ros::NodeHandle nh;

    ros::Publisher pcl_room_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_room", 1);
    ros::Publisher pcl_wall_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_wall", 1);
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("wall_octomap", 10, false);
    ros::Publisher occupancy_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("wall_map", 5, 10);

    ros::Publisher plane_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("plane_marker", 10 );
    // ros::Publisher plane_markers_pub_v_ = nh.advertise<visualization_msgs::MarkerArray>("plane_marker_v", 10 );
    // ros::Publisher plane_markers_pub_h_ = nh.advertise<visualization_msgs::MarkerArray>("plane_marker_h", 10 );
    ros::Publisher corner_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("corner_marker", 10 );

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_zfilter_before(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_zfilter_after(new pcl::PointCloud<pcl::PointXYZRGB>);

    ros::Time t1, t2;
    t1 = ros::Time::now();

    std::string pcd_path = "/home/k/2022scanningexperiment/point_cloud_3.pcd";
    pcl::io::loadPCDFile <pcl::PointXYZRGB> (pcd_path, *cloud_zfilter_before);
    // Downsample
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_downsampled = DownSample(cloud_zfilter_before, 0.05);
    sensor_msgs::PointCloud2 output_pointcloud_room;
    pcl::toROSMsg(*cloud_downsampled, output_pointcloud_room);
    output_pointcloud_room.header.frame_id = "map";
    std::cout << "data size is: " << output_pointcloud_room.data.size() << std::endl;

    // Outlier removal
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliner(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    // sor.setInputCloud(cloud_downsampled);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(5.0);
    // sor.filter(*cloud_inliner);

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(cloud_downsampled);
    outrem.setRadiusSearch(0.1);
    outrem.setMinNeighborsInRadius (10);
    outrem.filter (*cloud_inliner);

    pcl::PassThrough<pcl::PointXYZRGB> pass;    
    pass.setInputCloud (cloud_inliner);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.2, 2.7);
    pass.filter(*cloud_zfilter_after);


    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_corner (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    std::vector <pcl::PointIndices> clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = RegionGrowing(cloud_zfilter_after, cloud_plane, cloud_corner,  clusters);
    std::cout << "Number of clusters： " << clusters.size () << std::endl;

    sensor_msgs::PointCloud2 output_pointcloud_wall;
    pcl::toROSMsg(*colored_cloud, output_pointcloud_wall);
    output_pointcloud_wall.header.frame_id = "map";
    std::cout << "data size is: " << output_pointcloud_wall.data.size() << std::endl;

    // Segment Wall PointCloud
    // ---------------------------------------------------------------------------------------------------- //
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_V_Patch_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_H_Patch_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vCloud_V_Patch_Ptr;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vCloud_H_Patch_Ptr;
    std::vector<Eigen::Matrix3f> vPlane_Normal_V;
    std::vector<Eigen::Vector3f> vPlane_Origin_V;
    std::vector<Eigen::Vector3f> vPlane_Scale_V;
    std::vector<Eigen::Matrix3f> vPlane_Normal_H;
    std::vector<Eigen::Vector3f> vPlane_Origin_H;
    std::vector<Eigen::Vector3f> vPlane_Scale_H;
    SegmentWallCloud(clusters, colored_cloud, 
                     cloud_V_Patch_Ptr, cloud_H_Patch_Ptr,
                     vCloud_V_Patch_Ptr, vCloud_H_Patch_Ptr, 
                     vPlane_Normal_V, vPlane_Origin_V, vPlane_Scale_V,
                     vPlane_Normal_H, vPlane_Origin_H, vPlane_Scale_H);
    // Merge Vector
    std::vector<Eigen::Matrix3f> vPlane_Normal;
    std::vector<Eigen::Vector3f> vPlane_Origin;
    std::vector<Eigen::Vector3f> vPlane_Scale;
    vPlane_Normal.insert(vPlane_Normal.end(),vPlane_Normal_V.begin(),vPlane_Normal_V.end());
    vPlane_Origin.insert(vPlane_Origin.end(),vPlane_Origin_V.begin(),vPlane_Origin_V.end());
    vPlane_Scale.insert(vPlane_Scale.end(),vPlane_Scale_V.begin(),vPlane_Scale_V.end());
    vPlane_Normal.insert(vPlane_Normal.end(),vPlane_Normal_H.begin(),vPlane_Normal_H.end());
    vPlane_Origin.insert(vPlane_Origin.end(),vPlane_Origin_H.begin(),vPlane_Origin_H.end());
    vPlane_Scale.insert(vPlane_Scale.end(),vPlane_Scale_H.begin(),vPlane_Scale_H.end());

    // Total Plane BBOX
    std::cout<<"vPlane_Origin size is: "<<vPlane_Origin.size()<<std::endl;
    visualization_msgs::MarkerArray plane_msg_marker;
    for (size_t i = 0; i < vPlane_Origin.size(); i++)
    {
        visualization_msgs::Marker bbx_marker;
        bbx_marker.header.frame_id = "map";
        bbx_marker.header.stamp = ros::Time::now();
        bbx_marker.ns = "plane" + i;
        bbx_marker.type = visualization_msgs::Marker::CUBE;
        bbx_marker.action = visualization_msgs::Marker::ADD;
        bbx_marker.pose.position.x =  vPlane_Origin[i][0];
        bbx_marker.pose.position.y =  vPlane_Origin[i][1];
        bbx_marker.pose.position.z =  vPlane_Origin[i][2];
        Eigen::Quaternionf quat (vPlane_Normal[i]);
        quat.normalize();
        bbx_marker.pose.orientation.x = quat.x();
        bbx_marker.pose.orientation.y = quat.y();
        bbx_marker.pose.orientation.z = quat.z();
        bbx_marker.pose.orientation.w = quat.w();
        bbx_marker.scale.x = std::fabs(vPlane_Scale[i][0]);
        bbx_marker.scale.y = std::fabs(vPlane_Scale[i][1]);
        bbx_marker.scale.z = std::fabs(vPlane_Scale[i][2]);
        bbx_marker.color.b = 0;
        bbx_marker.color.g = 255;
        bbx_marker.color.r = 0;
        bbx_marker.color.a = 0.7;
        plane_msg_marker.markers.push_back(bbx_marker);
    }

    // Segment Corner PointCloud
    pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr corner_points (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    std::vector<Eigen::Vector3f> vCorner_Origin;
    std::vector<Eigen::Matrix3f> vCorner_Normal;
    std::vector<Eigen::Vector3f> vCorner_Scale;
    visualization_msgs::MarkerArray cross_marker_array;
    corner_points = SegmentCornerCloud(cloud_corner, vPlane_Origin, vPlane_Normal, vPlane_Scale, 
                                       vCorner_Origin, vCorner_Normal, vCorner_Scale);
    // std::cout << "vPlane_Origin_H[0][2]: " << vPlane_Origin_H[0][2] << std::endl;
    // corner_points = SegmentCornerCloud2(vPlane_Origin_V, vPlane_Normal_V, vPlane_Scale_V, cross_marker_array,
    //                                 vCorner_Origin, vCorner_Normal, vCorner_Scale, vPlane_Origin_H[0][2]);
    // Corner BBOX
    visualization_msgs::MarkerArray corner_msg_marker;
    for (size_t i = 0; i < vCorner_Origin.size(); i++)
    {
        visualization_msgs::Marker bbx_marker;
        bbx_marker.header.frame_id = "map";
        bbx_marker.header.stamp = ros::Time::now();
        bbx_marker.ns = "corner" + i;
        bbx_marker.type = visualization_msgs::Marker::CUBE;
        bbx_marker.action = visualization_msgs::Marker::ADD;
        bbx_marker.pose.position.x =  vCorner_Origin[i][0];
        bbx_marker.pose.position.y =  vCorner_Origin[i][1];
        bbx_marker.pose.position.z =  vCorner_Origin[i][2];
        Eigen::Quaternionf quat (vCorner_Normal[i]);
        quat.normalize();
        bbx_marker.pose.orientation.x = quat.x();
        bbx_marker.pose.orientation.y = quat.y();
        bbx_marker.pose.orientation.z = quat.z();
        bbx_marker.pose.orientation.w = quat.w();
        bbx_marker.scale.x = std::fabs(vCorner_Scale[i][0]);
        bbx_marker.scale.y = std::fabs(vCorner_Scale[i][1]);
        bbx_marker.scale.z = std::fabs(vCorner_Scale[i][2]);
        bbx_marker.color.b = 0;
        bbx_marker.color.g = 0;
        bbx_marker.color.r = 255;
        bbx_marker.color.a = 0.7;
        corner_msg_marker.markers.push_back(bbx_marker);
    }

    for (size_t i = 0; i < cloud_zfilter_after->points.size(); i++)
    { 
        octomap::OcTreeNode * cloudNode=cloudAndUnknown.updateNode(cloud_zfilter_after->points[i].x,
                                                                   cloud_zfilter_after->points[i].y,
                                                                   cloud_zfilter_after->points[i].z,
                                                                   true);
        cloudNode->setValue(1);
    }
    cloudAndUnknown.updateInnerOccupancy();
    octomap_msgs::Octomap octomapMsg;
    octomapMsg.header.frame_id = "map";
    octomap_msgs::binaryMapToMsg(cloudAndUnknown, octomapMsg);
    static nav_msgs::OccupancyGrid OccupancyGrid_msg;
    OccupancyGrid_msg.header.frame_id = "map";
    OccupancyGrid_msg.header.stamp = ros::Time::now();
    octomapToOccupancyGrid2(cloudAndUnknown, OccupancyGrid_msg, 0, std::numeric_limits<double>::max());

    t2 = ros::Time::now();
    std::cout << "the total Spend Time is: " << (t2-t1).toSec() << std::endl;

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_room_pub.publish(output_pointcloud_room);
        pcl_wall_pub.publish(output_pointcloud_wall);
        octomap_pub.publish(octomapMsg);
        occupancy_pub_.publish(OccupancyGrid_msg);
        plane_markers_pub_.publish(plane_msg_marker);
        corner_markers_pub_.publish(corner_msg_marker);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
}