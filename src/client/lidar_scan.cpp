#include "ouster_ros/client/lidar_scan.h"
#include <iostream>
#include <vector>
namespace ouster
{
XYZLut make_xyz_lut(LidarScan::index_t w, LidarScan::index_t h, double range_unit,
                    double lidar_origin_to_beam_origin_mm, const std::vector<double>& azimuth_angles_deg,
                    const std::vector<double>& altitude_angles_deg)
{
   Eigen::ArrayXd azimuth(w * h);
   Eigen::ArrayXd altitude(w * h);
   const double azimuth_radians = M_PI * 2.0 / w;
   for (LidarScan::index_t v = 0; v < w; v++)
   {
      for (LidarScan::index_t u = 0; u < h; u++)
      {
         LidarScan::index_t i = u * w + v;
         azimuth(i)           = azimuth_angles_deg[u] * M_PI / 180.0 + (v + w / 2) * azimuth_radians;
         altitude(i)          = altitude_angles_deg[u] * M_PI / 180.0;
      }
   }
   XYZLut lut;
   lut.direction        = LidarScan::Points{w * h, 3};
   lut.direction.col(0) = altitude.cos() * azimuth.cos();
   lut.direction.col(1) = -altitude.cos() * azimuth.sin();
   lut.direction.col(2) = altitude.sin();

   lut.direction *= range_unit;

   const double lidar_origin_to_beam_origin_m = lidar_origin_to_beam_origin_mm / 1000.0;

   lut.offset        = LidarScan::Points{w * h, 3};
   lut.offset.col(0) = azimuth.cos() - lut.direction.col(0);  // - lut.direction.col(0)* lidar_origin_to_beam_origin_m;
   lut.offset.col(1) = azimuth.sin() - lut.direction.col(1);  // - lut.direction.col(0)* lidar_origin_to_beam_origin_m;
   lut.offset.col(2) = -lut.direction.col(2);                 // Eigen::ArrayXd::Zero(w * h);
   lut.offset *= lidar_origin_to_beam_origin_m;

   return lut;
}

XYZLut make_xyz_lut_mirror(LidarScan::index_t w, LidarScan::index_t h, double range_unit,
                           double lidar_origin_to_beam_origin_mm, const std::vector<double>& azimuth_angles_deg,
                           const std::vector<double>& altitude_angles_deg,
                           std::vector<Eigen::Vector3d> up_mirror_points,
                           std::vector<Eigen::Vector3d> down_mirror_points)
{
   // Aproximate mirror interval
   const double mirror_az_min = -45 * M_PI / 180;
   const double mirror_az_max =  45 * M_PI / 180;

   // const double mirror_angle      = 50 * M_PI / 180;
   // const double mirror_distance_m = 0.051;  // 0.0525
   // const double w_mirror_m        = 0.080;  // 0.088
   // const double h_mirror_m        = 0.110;  // 0.120

   // const double x_mirror_min = -w_mirror_m / 2.0;
   // const double x_mirror_max = w_mirror_m / 2.0;
   // const double y_mirror_min = -mirror_distance_m - h_mirror_m * cos(mirror_angle);
   // const double y_mirror_max = -mirror_distance_m;

   // Extract plane data
   Eigen::Vector3d v1_plane_up, v2_plane_up, n_plane_up, p_plane_up;
   v1_plane_up = up_mirror_points[1] - up_mirror_points[0];
   v2_plane_up = up_mirror_points[2] - up_mirror_points[1];
   n_plane_up  = v1_plane_up.cross(v2_plane_up);
   p_plane_up  = up_mirror_points[0];
   n_plane_up.normalize();

   Eigen::Vector3d v1_plane_down, v2_plane_down, n_plane_down, p_plane_down;
   v1_plane_down = down_mirror_points[1] - down_mirror_points[0];
   v2_plane_down = down_mirror_points[2] - down_mirror_points[1];
   n_plane_down  = v1_plane_down.cross(v2_plane_down);
   p_plane_down  = down_mirror_points[0];
   n_plane_down.normalize();

   XYZLut lut;
   lut.direction = LidarScan::Points{w * h, 3};
   lut.offset    = LidarScan::Points{w * h, 3};
   lut.mirror_reflexed.resize(w * h);

   for (LidarScan::index_t v = 0; v < w; v++)
   {
      for (LidarScan::index_t u = 0; u < h; u++)
      {
         double azimuth  = azimuth_angles_deg[u] * M_PI / 180.0 + (v + w / 2.0) * M_PI * 2.0 / w - 2.0 * M_PI;
         double altitude = altitude_angles_deg[u] * M_PI / 180.0;
         double lidar_origin_to_beam_origin_m = lidar_origin_to_beam_origin_mm / 1000.0;

         LidarScan::index_t i = u * w + v;

         // Check if there are mirrors
         if (azimuth > mirror_az_min && azimuth < mirror_az_max)
         {
            // Select mirror plane
            Eigen::Vector3d n_plane, p_plane;
            std::vector<Eigen::Vector3d> points;
            if (altitude < 0)
            {
               p_plane = p_plane_down;
               n_plane = n_plane_down;
               points  = down_mirror_points;
            }
            else
            {
               p_plane = p_plane_up;
               n_plane = n_plane_up;
               points  = up_mirror_points;
            }

            // First beam section - Horizontal stage
            Eigen::Vector3d v1(cos(azimuth), -sin(azimuth), 0);
            v1.normalize();
            v1 *= lidar_origin_to_beam_origin_m;

            // Second beam section - To mirror intersection
            Eigen::Vector3d d2(cos(altitude) * cos(azimuth), -cos(altitude) * sin(azimuth), sin(altitude));
            d2.normalize();
            Eigen::Vector3d v2;
            v2 = (n_plane.dot(p_plane - v1) / n_plane.dot(d2)) * d2;

            Eigen::Vector3d i_point;
            i_point = v1 + v2;

            // Check if i_point is in the mirror section ( (x,y) inside projection of 4 lines )
            // Make lineal clasifiers
            std::vector<Eigen::Vector3d> w_lines;
            points.push_back(points[0]);
            for (uint i = 0; i < points.size() - 1; i++)
            {
               Eigen::Vector2d p1(points[i](0), points[i](1));
               Eigen::Vector2d p2(points[i + 1](0), points[i + 1](1));

               Eigen::Vector2d v_line;
               v_line = p2 - p1;
               v_line.normalize();

               Eigen::Vector2d n_line(v_line(1), -v_line(0));
               double rho = p1.dot(n_line);
               Eigen::Vector3d w(n_line(0), n_line(1), -rho);

               w_lines.push_back(w);
            }

            // Clasify
            uint inside = 0;
            Eigen::Vector3d i_data(i_point(0), i_point(1), 1);

            for (uint i = 0; i < w_lines.size(); i++)
               if (w_lines[i].dot(i_data) > 0) inside++;

            // Mirrors section
            if (inside == w_lines.size())
            {
               // Third section - Reflex direction
               const double theta_r = acos(n_plane.dot(-d2));
               Eigen::Vector3d n_reflex_plane;
               Eigen::Vector3d d3;
               n_reflex_plane = n_plane.cross(d2);
               d3             = cos(theta_r) * n_plane + sin(theta_r) * n_reflex_plane.cross(n_plane);
               d3.normalize();
               d3 *= range_unit;

               // Offset
               Eigen::Vector3d v_offset;
               v_offset = i_point - d3 * lidar_origin_to_beam_origin_m / range_unit - d3 * v2.norm() / range_unit;

               // Store data
               lut.direction(i, 0)    = d3(0);
               lut.direction(i, 1)    = d3(1);
               lut.direction(i, 2)    = d3(2);
               lut.offset(i, 0)       = v_offset(0);
               lut.offset(i, 1)       = v_offset(1);
               lut.offset(i, 2)       = v_offset(2);
               lut.mirror_reflexed[i] = true;

               continue;
            }
         }

         // Without mirrors
         Eigen::Vector3d v_direction(cos(altitude) * cos(azimuth), -cos(altitude) * sin(azimuth), sin(altitude));
         v_direction.normalize();
         v_direction *= range_unit;

         lut.direction(i, 0) = v_direction(0);
         lut.direction(i, 1) = v_direction(1);
         lut.direction(i, 2) = v_direction(2);

         v_direction /= range_unit;
         Eigen::Vector3d v_offset(cos(azimuth) - v_direction(0), -sin(azimuth) - v_direction(1), -v_direction(2));
         v_offset *= lidar_origin_to_beam_origin_m;

         lut.offset(i, 0) = v_offset(0);
         lut.offset(i, 1) = v_offset(1);
         lut.offset(i, 2) = v_offset(2);
      }
   }
   return lut;
}

}  // namespace ouster
