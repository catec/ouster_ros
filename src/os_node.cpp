
#include <ros/console.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include <math.h>

#include "ouster_ros/client/lidar_scan.h"
#include "ouster_ros/client/packet.h"
#include "ouster_ros/client/types.h"

#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/packets_queue.h"
#include "ouster_ros/parameters.h"
#include "ouster_ros/ros.h"

using PacketMsg  = ouster_ros::PacketMsg;
using Cloud      = ouster_ros::Cloud;
using Point      = ouster_ros::Point;
namespace sensor = ouster::sensor;

void cloud_packages_processing(ros::NodeHandle& nh, std::shared_ptr<ouster_ros::PacketsQueue<PacketMsg>>& queue,
                               sensor::sensor_info& info, const std::string& lidar_frame, const double stamp_offset,
                               const double max_sync_diff, bool check_diff)
{
   auto lidar_pub          = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
   auto lidar_status_pub   = nh.advertise<std_msgs::Header>("lidar_status", 10);
   auto mirror_indices_pub = nh.advertise<ouster_ros::MirrorIndicesMsg>("mirror_indices", 10);

   uint32_t H = info.format.pixels_per_column;
   uint32_t W = info.format.columns_per_frame;
   auto pf    = sensor::get_format(info.format);

   auto xyz_lut = ouster::make_xyz_lut_mirror(info);

   Cloud cloud{W, H};
   auto it = cloud.begin();
   sensor_msgs::PointCloud2 msg{};

   std::vector<uint32_t> mirror_indices;
   ouster_ros::MirrorIndicesMsg mirror_indices_msg{};

   std_msgs::Header status_msg;

   auto batch_and_publish = sensor::batch_to_iter<Cloud::iterator>(
       W, pf, {}, Point::get_from_pixel(xyz_lut, W, H), [&](std::chrono::nanoseconds scan_ts) mutable {
          double delay =
              ouster_ros::cloud_to_cloud_msg(cloud, msg, scan_ts, lidar_frame, stamp_offset, max_sync_diff, check_diff);
          ouster_ros::indices_to_indices_msg(mirror_indices, mirror_indices_msg, scan_ts, lidar_frame, stamp_offset,
                                             max_sync_diff, check_diff);

          if (check_diff && fabs(delay) > max_sync_diff)
          {
             ROS_WARN_STREAM("OS clock is not sync with host. Delay is: " << delay << " secs");
             return;
          }
          mirror_indices_pub.publish(mirror_indices_msg);
          lidar_pub.publish(msg);
          mirror_indices.clear();

          status_msg = msg.header;
          lidar_status_pub.publish(status_msg);
       });

   while (ros::ok())
   {
      auto packet = queue->get();
      batch_and_publish(packet.buf.data(), it, mirror_indices);
   }
}

void imu_packages_processing(ros::NodeHandle& nh, std::shared_ptr<ouster_ros::PacketsQueue<PacketMsg>>& queue,
                             sensor::sensor_info& info, const std::string& imu_frame, const double stamp_offset,
                             const double max_sync_diff, bool check_diff)
{
   auto lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 10);

   auto pf = sensor::get_format(info.format);

   while (ros::ok())
   {
      auto packet = queue->get();

      sensor_msgs::Imu msg;
      double delay = ouster_ros::packet_to_imu_msg(packet, msg, imu_frame, pf, stamp_offset, max_sync_diff, check_diff);
      if (check_diff && fabs(delay) > max_sync_diff)
      {
         ROS_WARN_STREAM("OS clock is not sync with host. Delay is: " << delay << " secs");
         return;
      }

      lidar_pub.publish(msg);
   }
}

void populate_metadata_defaults(sensor::sensor_info& info, sensor::lidar_mode specified_lidar_mode)
{
   if (!info.hostname.size()) info.hostname = "UNKNOWN";

   if (!info.sn.size()) info.sn = "UNKNOWN";

   ouster::util::version v = ouster::util::version_of_string(info.fw_rev);
   if (v == ouster::util::invalid_version)
      ROS_WARN("Unknown sensor firmware version; output may not be reliable");
   else if (v < sensor::min_version)
      ROS_WARN("Firmware < %s not supported; output may not be reliable", to_string(sensor::min_version).c_str());

   if (!info.mode)
   {
      ROS_WARN("Lidar mode not found in metadata; output may not be reliable");
      info.mode = specified_lidar_mode;
   }

   if (!info.prod_line.size()) info.prod_line = "UNKNOWN";

   if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty())
   {
      ROS_WARN("Beam angles not found in metadata; using design values");
      info.beam_azimuth_angles  = sensor::gen1_azimuth_angles;
      info.beam_altitude_angles = sensor::gen1_altitude_angles;
   }

   if (info.imu_to_sensor_transform.empty() || info.lidar_to_sensor_transform.empty())
   {
      ROS_WARN("Frame transforms not found in metadata; using design values");
      info.imu_to_sensor_transform   = sensor::imu_to_sensor_transform;
      info.lidar_to_sensor_transform = sensor::lidar_to_sensor_transform;
   }
}

void write_metadata(const std::string& meta_file, const std::string& metadata)
{
   std::ofstream ofs;
   ofs.open(meta_file);
   ofs << metadata << std::endl;
   ofs.close();
   if (ofs)
      ROS_INFO("Wrote metadata to $ROS_HOME/%s", meta_file.c_str());
   else
      ROS_WARN("Failed to write metadata to %s; check that the path is valid", meta_file.c_str());
}

int connection_loop(sensor::client& cli, const sensor::data_format& df,
                    std::shared_ptr<ouster_ros::PacketsQueue<PacketMsg>>& lidar_queue,
                    std::shared_ptr<ouster_ros::PacketsQueue<PacketMsg>>& imu_queue, bool enable_imu)
{
   auto pf = sensor::get_format(df);

   PacketMsg lidar_packet;
   PacketMsg imu_packet;

   while (ros::ok())
   {
      auto state = sensor::poll_client(cli);
      if (state == sensor::EXIT)
      {
         ROS_INFO("poll_client: caught signal, exiting");
         return EXIT_SUCCESS;
      }
      if (state & sensor::CLIENT_ERROR)
      {
         ROS_ERROR("poll_client: returned error");
         return EXIT_FAILURE;
      }
      if (state & sensor::LIDAR_DATA)
      {
         lidar_packet.buf.resize(pf.lidar_packet_size + 1);
         if (sensor::read_lidar_packet(cli, lidar_packet.buf.data(), pf)) lidar_queue->add(lidar_packet);
      }
      if (enable_imu & state & sensor::IMU_DATA)
      {
         if (sensor::read_imu_packet(cli, imu_packet.buf.data(), pf)) imu_queue->add(imu_packet);
      }
      ros::spinOnce();
   }
   return EXIT_SUCCESS;
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "os_node");
   ros::NodeHandle nh("~");

   ouster_ros::Parameters parameters(nh);

   ROS_INFO("Connecting to %s; sending data to %s", parameters.sensor_hostname.c_str(), parameters.udp_dest.c_str());
   ROS_INFO("Waiting for sensor to initialize ...");

   auto cli = sensor::init_client(parameters.sensor_hostname, parameters.udp_dest, parameters.lidar_mode,
                                  parameters.timestamp_mode, parameters.lidar_port, parameters.imu_port);
   if (!cli)
   {
      ROS_ERROR("Failed to initialize sensor at: %s", parameters.sensor_hostname.c_str());
      return EXIT_FAILURE;
   }
   ROS_INFO("Sensor initialized successfully");

   auto metadata = sensor::get_metadata(*cli);
   write_metadata(parameters.metafile, metadata);
   auto info = sensor::parse_metadata(metadata);
   populate_metadata_defaults(info, sensor::MODE_UNSPEC);

   ROS_INFO("Using lidar_mode: %s", sensor::to_string(info.mode).c_str());
   ROS_INFO("%s sn: %s firmware rev: %s", info.prod_line.c_str(), info.sn.c_str(), info.fw_rev.c_str());

   bool check_diff = false;
   if (parameters.timestamp_mode == ouster::sensor::TIME_FROM_PTP_1588) check_diff = true;

   // Lidar
   tf2_ros::StaticTransformBroadcaster tf_bcast{};

   tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(info.lidar_to_sensor_transform, parameters.sensor_frame,
                                                          parameters.lidar_frame, parameters.tf_inverse));

   std::shared_ptr<ouster_ros::PacketsQueue<PacketMsg>> lidar_queue =
       std::make_shared<ouster_ros::PacketsQueue<PacketMsg>>("lidar_queue", 512);

   std::unique_ptr<std::thread> lidar_processing_thread = std::make_unique<std::thread>(
       cloud_packages_processing, std::ref(nh), std::ref(lidar_queue), std::ref(info), std::ref(parameters.lidar_frame),
       parameters.stamp_offset, parameters.max_sync_diff, check_diff);

   // Imu
   std::shared_ptr<ouster_ros::PacketsQueue<PacketMsg>> imu_queue;
   std::unique_ptr<std::thread> imu_processing_thread;

   if (parameters.imu_port)
   {
      tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(info.imu_to_sensor_transform, parameters.sensor_frame,
                                                             parameters.imu_frame, false));

      std::shared_ptr<ouster_ros::PacketsQueue<PacketMsg>> imu_queue =
          std::make_shared<ouster_ros::PacketsQueue<PacketMsg>>("imu_queue", 512);

      imu_processing_thread = std::make_unique<std::thread>(
          imu_packages_processing, std::ref(nh), std::ref(imu_queue), std::ref(info), std::ref(parameters.imu_frame),
          parameters.stamp_offset, parameters.max_sync_diff, check_diff);
   }

   auto res = connection_loop(*cli, info.format, lidar_queue, imu_queue, (bool)parameters.imu_port);

   lidar_processing_thread->join();
   if (parameters.imu_port) imu_processing_thread->join();

   return res;
}
