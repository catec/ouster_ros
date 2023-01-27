#ifndef MIRRORS_MARKERS_H
#define MIRRORS_MARKERS_H

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>

#include "mirror_type.h"

namespace ouster_rviz
{
class MirrorMarkers
{
  public:
   MirrorMarkers(const double &mirror_angle, const double &mirror_distance_m, const double &w_mirror_m,
                 const double &h_mirror_m);
   MirrorMarkers(const std::vector<Eigen::Vector3d> &up_points, const std::vector<Eigen::Vector3d> &down_points);

   virtual ~MirrorMarkers();

   void computeMirror(MirrorType mirror_type);
   void addCornersMarkers();
   void addLinesMarkers();
   void addPlanesMarkers();

   visualization_msgs::MarkerArray getMarkers();

  private:
   double _mirror_angle{0};
   double _mirror_distance_m{0};
   double _w_mirror_m{0};
   double _h_mirror_m{0};
   std::vector<Eigen::Vector3d> _up_points;
   std::vector<Eigen::Vector3d> _down_points;
   MirrorType _mirror_type;
   int _id{0};
   bool _explicit_points{false};

   std::vector<Eigen::Vector3d> _corners;
   std::vector<geometry_msgs::Point> _lines_points;

   visualization_msgs::Marker _corners_markers;
   visualization_msgs::Marker _lines_markers;
   visualization_msgs::Marker _planes_markers;

   visualization_msgs::MarkerArray _markers;

   void computeCorners();
   void computePlanes();
   std::vector<geometry_msgs::Point> cornersToLines(std::vector<Eigen::Vector3d> face_corners);
   void appendTriangleFace(Eigen::Vector3d first_corner, Eigen::Vector3d second_corner, Eigen::Vector3d third_corner,
                           visualization_msgs::Marker &faces_marker);
   void initializeMarkers();
};

}  // namespace ouster_rviz

#endif