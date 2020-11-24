#include "ouster_ros/ros.h"

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cassert>
#include <chrono>
#include <string>
#include <vector>

#include "ouster_ros/client/types.h"

namespace ouster_ros
{
namespace sensor = ouster::sensor;

bool read_imu_packet(const sensor::client& cli, PacketMsg& m, const sensor::packet_format& pf)
{
   m.buf.resize(pf.imu_packet_size + 1);
   return read_imu_packet(cli, m.buf.data(), pf);
}

bool read_lidar_packet(const sensor::client& cli, PacketMsg& m, const sensor::packet_format& pf)
{
   m.buf.resize(pf.lidar_packet_size + 1);
   return read_lidar_packet(cli, m.buf.data(), pf);
}

double packet_to_imu_msg(const PacketMsg& p, sensor_msgs::Imu& imu_msg, const std::string& frame,
                         const sensor::packet_format& pf, const double stamp_offset, const double max_sync_diff,
                         bool check_diff)
{
   const double standard_g = 9.80665;
   const uint8_t* buf      = p.buf.data();

   imu_msg.header.stamp.fromNSec(pf.imu_gyro_ts(buf));
   auto stamp_sec = imu_msg.header.stamp.toSec() + stamp_offset;

   auto now       = ros::Time::now();
   auto diff_time = now.toSec() - stamp_sec;

   if (check_diff && abs(diff_time) > max_sync_diff) return diff_time;

   imu_msg.header.frame_id = frame;

   imu_msg.orientation.x = 0;
   imu_msg.orientation.y = 0;
   imu_msg.orientation.z = 0;
   imu_msg.orientation.w = 0;

   imu_msg.linear_acceleration.x = pf.imu_la_x(buf) * standard_g;
   imu_msg.linear_acceleration.y = pf.imu_la_y(buf) * standard_g;
   imu_msg.linear_acceleration.z = pf.imu_la_z(buf) * standard_g;

   imu_msg.angular_velocity.x = pf.imu_av_x(buf) * M_PI / 180.0;
   imu_msg.angular_velocity.y = pf.imu_av_y(buf) * M_PI / 180.0;
   imu_msg.angular_velocity.z = pf.imu_av_z(buf) * M_PI / 180.0;

   for (int i = 0; i < 9; i++)
   {
      imu_msg.orientation_covariance[i]         = -1;
      imu_msg.angular_velocity_covariance[i]    = 0;
      imu_msg.linear_acceleration_covariance[i] = 0;
   }
   for (int i = 0; i < 9; i += 4)
   {
      imu_msg.linear_acceleration_covariance[i] = 0.01;
      imu_msg.angular_velocity_covariance[i]    = 6e-4;
   }

   return diff_time;
}

double cloud_to_cloud_msg(const Cloud& cloud, sensor_msgs::PointCloud2& cloud_msg, ns timestamp,
                          const std::string& frame, const double stamp_offset, const double max_sync_diff,
                          bool check_diff)
{
   cloud_msg.header.stamp.fromNSec(timestamp.count());
   auto stamp_sec = cloud_msg.header.stamp.toSec() + stamp_offset;

   auto now       = ros::Time::now();
   auto diff_time = now.toSec() - stamp_sec;

   if (check_diff && abs(diff_time) > max_sync_diff) return diff_time;

   pcl::toROSMsg(cloud, cloud_msg);
   cloud_msg.header.stamp.fromSec(stamp_sec);
   cloud_msg.header.frame_id = frame;
   return diff_time;
}

geometry_msgs::TransformStamped transform_to_tf_msg(const std::vector<double>& mat, const std::string& frame,
                                                    const std::string& child_frame, bool inverse)
{
   assert(mat.size() == 16);

   tf2::Transform tf{};

   tf.setOrigin({mat[3] / 1e3, mat[7] / 1e3, mat[11] / 1e3});
   tf.setBasis({mat[0], mat[1], mat[2], mat[4], mat[5], mat[6], mat[8], mat[9], mat[10]});

   geometry_msgs::TransformStamped msg{};
   msg.header.stamp = ros::Time::now();
   if (!inverse)
   {
      msg.header.frame_id = frame;
      msg.child_frame_id  = child_frame;
      msg.transform       = tf2::toMsg(tf);
   }
   else
   {
      msg.header.frame_id = child_frame;
      msg.child_frame_id  = frame;
      msg.transform       = tf2::toMsg(tf.inverse());
   }

   return msg;
}
}  // namespace ouster_ros
