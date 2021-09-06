#ifndef MIRRORS_MARKERS_H
#define MIRRORS_MARKERS_H

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>

#include "mirror_type.h"

// namespace ouster
// {
class MirrorMarkers
{
  public:
   MirrorMarkers(const double &mirror_angle, const double &mirror_distance_m, const double &w_mirror_m,
                 const double &h_mirror_m);
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
   MirrorType _mirror_type;
   int _id{0};

   std::vector<Eigen::Vector3f> _corners;
   std::vector<geometry_msgs::Point> _lines_points;

   visualization_msgs::Marker _corners_markers;
   visualization_msgs::Marker _lines_markers;
   visualization_msgs::Marker _planes_markers;

   visualization_msgs::MarkerArray _markers;

   void computeCorners();
   void computePlanes();
   std::vector<geometry_msgs::Point> cornersToLines(const std::vector<Eigen::Vector3f> &face_corners);
   void appendTriangleFace(Eigen::Vector3f first_corner, Eigen::Vector3f second_corner, Eigen::Vector3f third_corner,
                           visualization_msgs::Marker &faces_marker);
};

// }  // namespace ouster

#endif