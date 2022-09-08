#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

class pcl_colored
{
private:
  ros::NodeHandle n;
  ros::Subscriber subCloud;
  ros::Publisher pubCloud;
  sensor_msgs::PointCloud2 msg;  //接收到的点云消息
  sensor_msgs::PointCloud2 colored_msg;  //等待发送的点云消息

public:
  pcl_colored():
    n("~"){
    subCloud = n.subscribe<sensor_msgs::PointCloud2>("/livox_scanlaser_livox_1", 1000, &pcl_colored::getcloud, this); //接收velodyne点云数据，进入回调函数getcloud()
    pubCloud = n.advertise<sensor_msgs::PointCloud2>("/livox/lidar", 1000);  //建立了一个发布器，主题是/adjustd_cloud，方便之后发布加入颜色之后的点云
  }

  //回调函数
  void getcloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
  {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr   raw_pcl_ptr (new pcl::PointCloud<pcl::PointXYZ>);   //VLP-16的点云消息包含xyz和intensity、ring的，这里没有加ring不知道为啥也可以，需要的话要自己定义一个点类型PointXYZIR
    sensor_msgs::PointCloud raw_pcl_ptr;
    sensor_msgs::convertPointCloud2ToPointCloud(*laserCloudMsg, raw_pcl_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr  colored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZI>);   //放在这里是因为，每次都需要重新初始化，舍弃了原有但没用的两个通道intensity、ring

    for (int i = 0; i <  raw_pcl_ptr.points.size(); i++)
    {
      pcl::PointXYZI  p;
      p.x=raw_pcl_ptr.points[i].x;
      p.y=raw_pcl_ptr.points[i].y;
      p.z=raw_pcl_ptr.points[i].z;
      p.intensity=1.0;
      // p.intensity=1.0f;
      colored_pcl_ptr->points.push_back(p);
    }
    colored_pcl_ptr->width = 1;
    colored_pcl_ptr->height = raw_pcl_ptr.points.size();
    pcl::toROSMsg( *colored_pcl_ptr,  colored_msg);  //将点云转化为消息才能发布
    colored_msg.header.frame_id = "livox_frame";//帧id改成和velodyne一样的
    colored_msg.header.stamp = laserCloudMsg->header.stamp;//转化之后时间戳不变
    pubCloud.publish(colored_msg); //发布调整之后的点云数据，主题为/adjustd_cloud
  }

  ~pcl_colored(){}

};

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "intensity");  //初始化了一个节点，名字为colored
  pcl_colored  pc;
  ros::spin();
  return 0;
}

