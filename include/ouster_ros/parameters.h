/*!
 *      @file  paremeters.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  20/11/2020
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2020, FADA-CATEC
 */

#pragma once

#include <ros/ros.h>

#include "ouster_ros/client/types.h"

namespace ouster_ros
{
namespace sensor = ouster::sensor;

class Parameters
{
  public:
   Parameters(ros::NodeHandle& nh);

   std::string sensor_hostname = "";
   std::string udp_dest        = "";

   int lidar_port = 0;
   int imu_port   = 0;

   sensor::lidar_mode lidar_mode         = sensor::MODE_UNSPEC;
   sensor::timestamp_mode timestamp_mode = sensor::TIME_FROM_UNSPEC;

   double stamp_offset  = 0.0;
   double max_sync_diff = 1.5;

   std::string tf_prefix = "";
   bool tf_inverse       = false;

   std::string sensor_frame = "";
   std::string imu_frame    = "";
   std::string lidar_frame  = "";

   std::string metafile = sensor_hostname + ".json";
};
}  // namespace ouster_ros
