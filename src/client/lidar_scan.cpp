#include "ouster_ros/client/lidar_scan.h"
#include <vector>
#include <iostream>
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
   lut.offset.col(0) = azimuth.cos() - lut.direction.col(0);   // - lut.direction.col(0)* lidar_origin_to_beam_origin_m;
   lut.offset.col(1) = azimuth.sin() - lut.direction.col(1);   // - lut.direction.col(0)* lidar_origin_to_beam_origin_m;
   lut.offset.col(2) = - lut.direction.col(2);                 // Eigen::ArrayXd::Zero(w * h);
   lut.offset *= lidar_origin_to_beam_origin_m;

   return lut;
}

XYZLut make_xyz_lut_mirror(LidarScan::index_t w, LidarScan::index_t h, double range_unit,
                           double lidar_origin_to_beam_origin_mm, const std::vector<double>& azimuth_angles_deg,
                           const std::vector<double>& altitude_angles_deg)
{
   // Const Mirror Data
   const double mirror_angle      = 45 * M_PI/ 180;
   const double mirror_distance_m = 0.0525;

   const double mirror_az_min  = 50  * M_PI / 180;
   const double mirror_az_max  = 130 * M_PI / 180;
   const double mirror_alt_min = -mirror_angle;
   const double mirror_alt_max = mirror_angle;
   

   const Eigen::Vector3d p_plane(0,-mirror_distance_m,0);

   XYZLut lut;
   lut.direction = LidarScan::Points{w * h, 3};
   lut.offset    = LidarScan::Points{w * h, 3};

   for (LidarScan::index_t v = 0; v < w; v++)
   {
      for (LidarScan::index_t u = 0; u < h; u++)
      {
         double azimuth  = azimuth_angles_deg[u] * M_PI/180.0 + (v + w / 2.0) * M_PI*2.0/w - 2.0*M_PI;
         double altitude = altitude_angles_deg[u] * M_PI / 180.0;
         double lidar_origin_to_beam_origin_m =  lidar_origin_to_beam_origin_mm / 1000.0;

         LidarScan::index_t i = u*w + v;

         // Without mirrors
         if ( (azimuth < mirror_az_min) || (azimuth > mirror_az_max) || (altitude < mirror_alt_min) || (altitude > mirror_alt_max)) 
         {
            Eigen::Vector3d v_direction(cos(altitude) * cos(azimuth),-cos(altitude) * sin(azimuth),sin(altitude));
            v_direction*= range_unit;
            
            lut.direction(i,0) = v_direction(0);
            lut.direction(i,1) = v_direction(1);
            lut.direction(i,2) = v_direction(2);

            Eigen::Vector3d v_offset(cos(azimuth) - v_direction(0),-sin(azimuth)- v_direction(1),- v_direction(2));
            v_offset*= lidar_origin_to_beam_origin_m;

            lut.offset(i,0) = v_offset(0);
            lut.offset(i,1) = v_offset(1);
            lut.offset(i,2) = v_offset(2);
         }
         else // With mirrors
         {
         
            // Select mirror plane
            double z_n_direction = cos(mirror_angle);
            if (altitude < 0)
               z_n_direction *= -1;
               
            Eigen::Vector3d n_plane(0, sin(mirror_angle), z_n_direction);
            n_plane.normalize();

            // First beam section - Horizontal stage
            Eigen::Vector3d v1(cos(azimuth),-sin(azimuth),0);
            v1.normalize();
            v1 *= lidar_origin_to_beam_origin_m;

            // Second beam section - To mirror intersection
            Eigen::Vector3d d2(cos(altitude)*cos(azimuth),-cos(altitude)*sin(azimuth),sin(altitude));
            d2.normalize();
            Eigen::Vector3d v2;
            v2 = (n_plane.dot(p_plane-v1)/n_plane.dot(d2))*d2;

            // Third section - Reflex direction
            const double theta_r = acos(n_plane.dot(-d2));
            Eigen::Vector3d n_reflex_plane;
            Eigen::Vector3d d3;
            n_reflex_plane = n_plane.cross(d2);
            d3 = cos(theta_r)*n_plane + sin(theta_r)*n_reflex_plane.cross(n_plane);
            d3.normalize();
            d3 *= range_unit;

            // Offset
            Eigen::Vector3d v_offset;
            v_offset = v1 + v2 - d3*lidar_origin_to_beam_origin_m - d3 * v2.norm();

            // Store data
            lut.direction(i,0) = d3(0);
            lut.direction(i,1) = d3(1);
            lut.direction(i,2) = d3(2);
            lut.offset(i,0)    = v_offset(0);
            lut.offset(i,1)    = v_offset(1);
            lut.offset(i,2)    = v_offset(2);
         }
      }  
   }   
   return lut;
}



// OLDER

// XYZLut make_xyz_lut_mirror(LidarScan::index_t w, LidarScan::index_t h, double range_unit,
//                     double lidar_origin_to_beam_origin_mm, const std::vector<double>& azimuth_angles_deg,
//                     const std::vector<double>& altitude_angles_deg)
// {
//    double new_range;
//    Eigen::ArrayXd a(w *h);
//    Eigen::ArrayXd azimuth(w * h);
//    Eigen::ArrayXd altitude(w * h);
//    Eigen::ArrayXd new_altitude(w * h);
//    const double azimuth_radians = M_PI * 2.0 / w;
//    for (LidarScan::index_t v = 0; v < w; v++)
//    {
//       for (LidarScan::index_t u = 0; u < h; u++)
//       {
//          // TODO: if altitude < 0, el valor de h es negativo, sacar h y r de sen y cos siendo conocido el angulo?
//          LidarScan::index_t i = u * w + v;
//          azimuth(i)           = azimuth_angles_deg[u] * M_PI / 180.0 + (v + w / 2) * azimuth_radians;
//          altitude(i)          = altitude_angles_deg[u] * M_PI / 180.0;
//          a(i)                 = ((0.525 - lidar_origin_to_beam_origin_mm / 1000) * 0.766) / sin(50 - std::abs(altitude(i)));
//          if (altitude(i) > 0)
//          {
//             new_altitude(i)   = altitude(i) + acos(((range_unit - a(i)) * sin(2 * (40 + altitude (i)))) / 
//                                 sqrt(2 * pow(a(i), 2) + pow(range_unit, 2) - 2 * a(i) * range_unit - 2 * a(i) * (range_unit - a(i) * cos(2 * (40 + altitude(i))))));
//          }
//          else
//          {
//             new_altitude(i)   = altitude(i) - acos(((range_unit - a(i)) * sin(2 * (40 + std::abs(altitude (i))))) / 
//                                 sqrt(2 * pow(a(i), 2) + pow(range_unit, 2) - 2 * a(i) * range_unit - 2 * a(i) * (range_unit - a(i) * cos(2 * (40 + std::abs(altitude(i)))))));
//          }
//          new_range = sqrt(2 * pow(a(i), 2) + pow(range_unit, 2) - 2 * a(i) * range_unit - 2 * a(i) * (range_unit - a(i) * cos(2 * (40 + std::abs(altitude(i)))))) * cos(new_altitude(i));
//       }
//    }
//    XYZLut lut;
//    lut.direction        = LidarScan::Points{w * h, 3};
//    lut.direction.col(0) = new_altitude.cos() * azimuth.cos();
//    lut.direction.col(1) = -new_altitude.cos() * azimuth.sin();
//    lut.direction.col(2) = new_altitude.sin();
//    lut.direction *= new_range;

//    const double lidar_origin_to_beam_origin_m = lidar_origin_to_beam_origin_mm / 1000.0;

//    lut.offset        = LidarScan::Points{w * h, 3};
//    lut.offset.col(0) = azimuth.cos() - lut.direction.col(0) * lidar_origin_to_beam_origin_m;
//    lut.offset.col(1) = azimuth.sin() - lut.direction.col(0) * lidar_origin_to_beam_origin_m;
//    lut.offset.col(2) = Eigen::ArrayXd::Zero(w * h);
//    lut.offset *= lidar_origin_to_beam_origin_m;

//    return lut;
// }

}  // namespace ouster
