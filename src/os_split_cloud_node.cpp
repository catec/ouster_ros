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

#include <XmlRpcException.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>

#include "mirror_markers.h"

std::vector<int> indices;
ros::Publisher pub_mirror_points;
ros::Publisher pub_non_mirror_points;
ros::Publisher pub_markers;
std::shared_ptr<ouster_rviz::MirrorMarkers> mirror_markers;
bool publish_mirrors_marker {false};

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

   // Split full cloud in two clouds
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

   if (!publish_mirrors_marker) 
      return;

   // Mirrors markers representation for rviz
   visualization_msgs::MarkerArray mirror_array = mirror_markers->getMarkers();
   for (uint i = 0; i < mirror_array.markers.size(); i++)
   {
      mirror_array.markers[i].header.frame_id = cloud_msg->header.frame_id;
      mirror_array.markers[i].header.stamp    = cloud_msg->header.stamp;
   }

   pub_markers.publish(mirror_array);
}

void createMirrorsMarkerFromParams(const std::vector<Eigen::Vector3d> &up_points, const std::vector<Eigen::Vector3d> &down_points)
{
   mirror_markers = std::make_shared<ouster_rviz::MirrorMarkers>(up_points, down_points);

   mirror_markers->computeMirror(ouster_rviz::MirrorType::UP);
   mirror_markers->addCornersMarkers();
   mirror_markers->addLinesMarkers();
   mirror_markers->addPlanesMarkers();

   mirror_markers->computeMirror(ouster_rviz::MirrorType::DOWN);
   mirror_markers->addCornersMarkers();
   mirror_markers->addLinesMarkers();
   mirror_markers->addPlanesMarkers();
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "os_split_cloud_node");
   ros::NodeHandle nh("~");

   if (!(nh.hasParam("/up_mirror_points_x") || nh.hasParam("/up_mirror_points_y") || nh.hasParam("/up_mirror_points_z") ||
         nh.hasParam("/down_mirror_points_x") || nh.hasParam("/down_mirror_points_y") || nh.hasParam("/down_mirror_points_z")))
   {
      ROS_WARN("No mirror points found in parameters, no mirror markers will be published");
      publish_mirrors_marker = false;
   }
   else
   {
      publish_mirrors_marker = true;
      std::vector<Eigen::Vector3d> up_points;
      std::vector<Eigen::Vector3d> down_points;
      try
      {
         XmlRpc::XmlRpcValue up_x, up_y, up_z;
         nh.param("/up_mirror_points_x", up_x, up_x);
         nh.param("/up_mirror_points_y", up_y, up_y);
         nh.param("/up_mirror_points_z", up_z, up_z);

         XmlRpc::XmlRpcValue down_x, down_y, down_z;
         nh.param("/down_mirror_points_x", down_x, down_x);
         nh.param("/down_mirror_points_y", down_y, down_y);
         nh.param("/down_mirror_points_z", down_z, down_z);

         for (int i = 0; i < up_x.size(); i++) up_points.push_back(Eigen::Vector3d(up_x[i], up_y[i], up_z[i]));
         for (int i = 0; i < down_x.size(); i++) down_points.push_back(Eigen::Vector3d(down_x[i], down_y[i], down_z[i]));
      }
      catch (XmlRpc::XmlRpcException& ex) 
      { 
         ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str()); 
         return 0;
      }
      
      createMirrorsMarkerFromParams(up_points, down_points);
      pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/os_node/mirror_markers", 10);
   }

   pub_mirror_points     = nh.advertise<sensor_msgs::PointCloud2>("/os_node/mirror_points", 10);
   pub_non_mirror_points = nh.advertise<sensor_msgs::PointCloud2>("/os_node/non_mirror_points", 10);

   ros::Subscriber sub_indices = nh.subscribe("/os_node/mirror_indices", 1, indicesCallBack);
   ros::Subscriber sub_poins   = nh.subscribe("/os_node/points", 1, cloudCallBack);

   ros::spin();
}