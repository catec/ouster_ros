#include <ouster_ros/MirrorIndicesMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

std::vector<int> indices;
ros::Publisher pub_mirror_points;
ros::Publisher pub_non_mirror_points;

void indicesCallBack(ouster_ros::MirrorIndicesMsgConstPtr indices_msg)
{
   std::vector<uint32_t> indices_u;
   indices_u = indices_msg->indices;

   indices.clear();
   for (uint i = 0; i < indices_u.size(); i++) indices.push_back(int(indices_u[i]));
}

void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
   pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr mirror_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   sensor_msgs::PointCloud2 mirror_msg;
   pcl::PointCloud<pcl::PointXYZ>::Ptr non_mirror_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   sensor_msgs::PointCloud2 non_mirror_msg;

   pcl::fromROSMsg(*cloud_msg, *original_cloud);

   pcl::ExtractIndices<pcl::PointXYZ> extractor;
   pcl::PointIndicesPtr point_indices(new pcl::PointIndices);

   extractor.setInputCloud(original_cloud);

   point_indices->indices = indices;
   extractor.setIndices(point_indices);

   extractor.setNegative(false);
   extractor.filter(*mirror_cloud);

   extractor.setNegative(true);
   extractor.filter(*non_mirror_cloud);

   pcl::toROSMsg(*mirror_cloud, mirror_msg);
   mirror_msg.header = cloud_msg->header;

   pcl::toROSMsg(*non_mirror_cloud, non_mirror_msg);
   non_mirror_msg.header = cloud_msg->header;

   pub_mirror_points.publish(mirror_msg);
   pub_non_mirror_points.publish(non_mirror_msg);

   // Mirrors markers representation
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "os_img_node");
   ros::NodeHandle nh("~");
   indices.clear();
   pub_mirror_points     = nh.advertise<sensor_msgs::PointCloud2>("/os_node/mirror_points", 10);
   pub_non_mirror_points = nh.advertise<sensor_msgs::PointCloud2>("/os_node/non_mirror_points", 10);

   ros::Subscriber sub_indices = nh.subscribe("/os_node/mirror_indices", 1, indicesCallBack);
   ros::Subscriber sub_poins   = nh.subscribe("/os_node/points", 1, cloudCallBack);

   ros::spin();
}