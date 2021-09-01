/**
 * @file
 * @brief Higher-level functions to read data from the ouster sensors as ROS messages
 */

#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <chrono>
#include <string>

#include "ouster_ros/client/client.h"
#include "ouster_ros/client/types.h"

#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/point.h"

#include "ouster_ros/MirrorIndicesMsg.h"

namespace ouster_ros
{
namespace sensor = ouster::sensor;
using Cloud      = pcl::PointCloud<Point>;
using ns         = std::chrono::nanoseconds;

/**
 * Read an imu packet into a ROS message. Blocks for up to a second if no data
 * is available.
 * @param cli the sensor client
 * @param pm the destination packet message
 * @return whether reading was successful
 */
bool read_imu_packet(const sensor::client& cli, PacketMsg& pm, const sensor::packet_format& pf);

/**
 * Read a lidar packet into a ROS message. Blocks for up to a second if no data
 * is available.
 * @param cli the sensor client
 * @param pm the destination packet message
 * @return whether reading was successful
 */
bool read_lidar_packet(const sensor::client& cli, PacketMsg& pm, const sensor::packet_format& pf);

/**
 * Parse an imu packet message into a ROS imu message
 * @param pm packet message populated by read_imu_packet
 * @param frame the frame to set in the resulting ROS message
 * @return ROS sensor message with fields populated from the packet
 */
double packet_to_imu_msg(const PacketMsg& pm, sensor_msgs::Imu& imu_msg, const std::string& frame,
                         const sensor::packet_format& pf, const double stamp_offset, const double max_sync_diff,
                         bool check_diff);

/**
 * Serialize a PCL point cloud to a ROS message
 * @param cloud the PCL point cloud to convert
 * @param timestamp the timestamp to give the resulting ROS message
 * @param frame the frame to set in the resulting ROS message
 * @return a ROS message containing the point cloud
 */
double cloud_to_cloud_msg(const Cloud& cloud, sensor_msgs::PointCloud2& cloud_msg, ns timestamp,
                          const std::string& frame, const double stamp_offset, const double max_sync_diff,
                          bool check_diff);

double indices_to_indices_msg(const std::vector<uint32_t>& indices, MirrorIndicesMsg &indices_msg, ns timestamp,
                              const std::string& frame, const double stamp_offset, const double max_sync_diff,
                              bool check_diff);

/**
 * Convert transformation matrix return by sensor to ROS transform
 * @param mat transformation matrix return by sensor
 * @param frame the parent frame of the published transform
 * @param child_frame the child frame of the published transform
 * @return ROS message suitable for publishing as a transform
 */
geometry_msgs::TransformStamped transform_to_tf_msg(const std::vector<double>& mat, const std::string& frame,
                                                    const std::string& child_frame, bool inverse);
}  // namespace ouster_ros
