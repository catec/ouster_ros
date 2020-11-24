/*!
 *      @file  parameters.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  20/11/2020
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2020, FADA-CATEC
 */

#include "ouster_ros/parameters.h"

namespace ouster_ros
{
Parameters::Parameters(ros::NodeHandle& nh)
{
   sensor_hostname = nh.param("sensor_hostname", std::string{});
   udp_dest        = nh.param("udp_dest", std::string{});

   if (!sensor_hostname.size() || !udp_dest.size())
   {
      throw ros::Exception("Must specify both hostname and udp destination");
   }

   lidar_port = nh.param("lidar_port", 0);
   if (lidar_port <= 0) throw ros::Exception("Invalid lidar port");

   imu_port = nh.param("imu_port", 0);
   if (lidar_port < 0)
      throw ros::Exception("Invalid imu port");
   else if (lidar_port)
      ROS_WARN("IMU is disabled");

   std::string lidar_mode_arg = nh.param("lidar_mode", std::string{});
   if (lidar_mode_arg.size())
   {
      lidar_mode = sensor::lidar_mode_of_string(lidar_mode_arg);
      if (!lidar_mode) throw ros::Exception("Invalid lidar mode " + lidar_mode_arg);
   }

   std::string timestamp_mode_arg = nh.param("timestamp_mode", std::string{});
   if (timestamp_mode_arg.size())
   {
      timestamp_mode = sensor::timestamp_mode_of_string(timestamp_mode_arg);
      if (!timestamp_mode) throw ros::Exception("Invalid timestamp mode " + timestamp_mode_arg);
   }

   stamp_offset  = nh.param("stamp_offset", 0.0);
   max_sync_diff = nh.param("max_sync_diff", 1.5);

   tf_prefix  = nh.param("tf_prefix", std::string{});
   tf_inverse = nh.param("tf_inverse", true);

   if (!tf_prefix.empty() && tf_prefix.back() != '/') tf_prefix.append("/");
   sensor_frame = tf_prefix + "os_sensor";
   imu_frame    = tf_prefix + "os_imu";
   lidar_frame  = tf_prefix + "os_lidar";
}
}  // namespace ouster_ros