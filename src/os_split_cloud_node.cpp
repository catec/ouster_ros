#include <ouster_ros/MirrorIndicesMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>

#include "mirror_markers.h"

std::vector<int> indices;
ros::Publisher pub_mirror_points;
ros::Publisher pub_non_mirror_points;
ros::Publisher pub_markers;
std::shared_ptr<MirrorMarkers> mirror_markers;

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
   pcl::ExtractIndices<pcl::PointXYZ> extractor;
   pcl::PointIndicesPtr point_indices(new pcl::PointIndices);

   // Split full cloud in thwo clouds
   pcl::fromROSMsg(*cloud_msg, *original_cloud);
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

   // Mirrors markers representation for rviz
   visualization_msgs::MarkerArray mirror_array = mirror_markers->getMarkers();
   for (uint i = 0; i < mirror_array.markers.size(); i++)
   {
      mirror_array.markers[i].header.frame_id = cloud_msg->header.frame_id;
      mirror_array.markers[i].header.stamp    = cloud_msg->header.stamp;
   }

   pub_markers.publish(mirror_array);
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "os_split_cloud_node");
   ros::NodeHandle nh("~");

   //[[TODO]] Read data as parameters
   const double mirror_angle      = 50 * M_PI / 180;
   const double mirror_distance_m = 0.051;  // 0.0525
   const double w_mirror_m        = 0.080;  // 0.088
   const double h_mirror_m        = 0.110;  // 0.120
   //[[]]

   mirror_markers = std::make_shared<MirrorMarkers>(mirror_angle, mirror_distance_m, w_mirror_m, h_mirror_m);

   mirror_markers->computeMirror(MirrorType::UP);
   mirror_markers->addCornersMarkers();
   mirror_markers->addLinesMarkers();

   mirror_markers->computeMirror(MirrorType::DOWN);
   mirror_markers->addCornersMarkers();
   mirror_markers->addLinesMarkers();

   pub_mirror_points     = nh.advertise<sensor_msgs::PointCloud2>("/os_node/mirror_points", 10);
   pub_non_mirror_points = nh.advertise<sensor_msgs::PointCloud2>("/os_node/non_mirror_points", 10);
   pub_markers           = nh.advertise<visualization_msgs::MarkerArray>("/os_node/mirror_markers", 10);

   ros::Subscriber sub_indices =
       nh.subscribe("/os_node/mirror_indices", 1, indicesCallBack);           // /os_node/mirror_indices
   ros::Subscriber sub_poins = nh.subscribe("/cloud_pcd", 1, cloudCallBack);  // /os_node/points

   ros::spin();
}
