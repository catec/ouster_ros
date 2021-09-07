#include "mirror_markers.h"

namespace ouster_rviz
{
MirrorMarkers::MirrorMarkers(const double &mirror_angle, const double &mirror_distance_m, const double &w_mirror_m,
                             const double &h_mirror_m)
{
   _mirror_angle      = mirror_angle;
   _mirror_distance_m = mirror_distance_m;
   _w_mirror_m        = w_mirror_m;
   _h_mirror_m        = h_mirror_m;
   _explicit_points   = false;

   initializeMarkers();
}

MirrorMarkers::MirrorMarkers(std::vector<Eigen::Vector3d> &up_points, std::vector<Eigen::Vector3d> &down_points)
{
   _up_points       = up_points;
   _down_points     = down_points;
   _explicit_points = true;

   initializeMarkers();
}

MirrorMarkers::~MirrorMarkers() {}

void MirrorMarkers::initializeMarkers()
{
   _corners_markers.type               = visualization_msgs::Marker::POINTS;
   _corners_markers.action             = visualization_msgs::Marker::MODIFY;
   _corners_markers.ns                 = "/mirror_markers";
   _corners_markers.pose.orientation.w = 1.0;
   _corners_markers.scale.x            = 0.01;
   _corners_markers.scale.y            = 0.01;
   _corners_markers.color.a            = 1.0;
   _corners_markers.color.r            = 0.15;
   _corners_markers.color.g            = 0.15;
   _corners_markers.color.b            = 1;

   _lines_markers.type               = visualization_msgs::Marker::LINE_LIST;
   _lines_markers.action             = visualization_msgs::Marker::MODIFY;
   _lines_markers.ns                 = "/mirror_markers";
   _lines_markers.pose.orientation.w = 1.0;
   _lines_markers.scale.x            = 0.005;
   _lines_markers.color.a            = 1.0;
   _lines_markers.color.r            = 0.8;
   _lines_markers.color.g            = 0.8;
   _lines_markers.color.b            = 0.8;

   _planes_markers.type               = visualization_msgs::Marker::TRIANGLE_LIST;
   _planes_markers.action             = visualization_msgs::Marker::MODIFY;
   _planes_markers.ns                 = "/mirror_markers";
   _planes_markers.pose.orientation.w = 1.0;
   _planes_markers.scale.x            = 1.0;
   _planes_markers.scale.y            = 1.0;
   _planes_markers.scale.z            = 1.0;
   _planes_markers.color.a            = 1.0;
   _planes_markers.color.r            = 0.3;
   _planes_markers.color.g            = 0.3;
   _planes_markers.color.b            = 0.8;
}

void MirrorMarkers::computeMirror(MirrorType mirror_type)
{
   _mirror_type = mirror_type;

   _corners.clear();
   _corners_markers.points.clear();
   _lines_markers.points.clear();
   _planes_markers.points.clear();

   computeCorners();
   _lines_points = cornersToLines(_corners);

   for (uint i = 0; i < _lines_points.size(); i++) _corners_markers.points.push_back(_lines_points[i]);
   for (uint i = 0; i < _lines_points.size(); i++) _lines_markers.points.push_back(_lines_points[i]);

   computePlanes();
}

void MirrorMarkers::addCornersMarkers()
{
   _corners_markers.id = _id;
   _markers.markers.push_back(_corners_markers);
   _id++;
}
void MirrorMarkers::addLinesMarkers()
{
   _lines_markers.id = _id;
   _markers.markers.push_back(_lines_markers);
   _id++;
}
void MirrorMarkers::addPlanesMarkers()
{
   _planes_markers.id = _id;
   _markers.markers.push_back(_planes_markers);
   _id++;
}

visualization_msgs::MarkerArray MirrorMarkers::getMarkers() { return _markers; }

std::vector<geometry_msgs::Point> MirrorMarkers::cornersToLines(std::vector<Eigen::Vector3d> face_corners)
{
   std::vector<geometry_msgs::Point> lines_face;

   face_corners.push_back(face_corners[0]);

   for (uint i = 0; i < face_corners.size() - 1; i++)
   {
      auto pt_msg = geometry_msgs::Point();
      tf::pointEigenToMsg(face_corners[i].cast<double>(), pt_msg);
      lines_face.push_back(pt_msg);
      tf::pointEigenToMsg(face_corners[i + 1].cast<double>(), pt_msg);
      lines_face.push_back(pt_msg);
   }

   return lines_face;
}

void MirrorMarkers::appendTriangleFace(Eigen::Vector3d first_corner, Eigen::Vector3d second_corner,
                                       Eigen::Vector3d third_corner, visualization_msgs::Marker &faces_marker)
{
   geometry_msgs::Point triang_corner_1, triang_corner_2, triang_corner_3;
   tf::pointEigenToMsg(first_corner.cast<double>(), triang_corner_1);
   faces_marker.points.push_back(triang_corner_1);
   tf::pointEigenToMsg(second_corner.cast<double>(), triang_corner_2);
   faces_marker.points.push_back(triang_corner_2);
   tf::pointEigenToMsg(third_corner.cast<double>(), triang_corner_3);
   faces_marker.points.push_back(triang_corner_3);
}

void MirrorMarkers::computeCorners()
{
   if (_explicit_points)
   {
      if (_mirror_type == MirrorType::UP)

         _corners = _up_points;

      else if (_mirror_type == MirrorType::DOWN)

         _corners = _down_points;

      return;
   }

   Eigen::Vector3d x_vector(1, 0, 0);
   Eigen::Vector3d y_vector(0, 1, 0);
   Eigen::Vector3d z_vector(0, 0, 1);

   Eigen::Vector3d up_vector(0, -cos(_mirror_angle), sin(_mirror_angle));
   Eigen::Vector3d down_vector(0, -cos(_mirror_angle), -sin(_mirror_angle));

   if (_mirror_type == MirrorType::UP)
   {
      // Bottom-right / Bottom-left / Top-left / Top-right
      _corners.push_back(-y_vector * _mirror_distance_m - x_vector * _w_mirror_m / 2.0);
      _corners.push_back(-y_vector * _mirror_distance_m + x_vector * _w_mirror_m / 2.0);
      _corners.push_back(-y_vector * _mirror_distance_m + x_vector * _w_mirror_m / 2.0 + up_vector * _h_mirror_m);
      _corners.push_back(-y_vector * _mirror_distance_m - x_vector * _w_mirror_m / 2.0 + up_vector * _h_mirror_m);
   }
   else if (_mirror_type == MirrorType::DOWN)
   {
      // Top-left / Top-right / Bottom-left / Bottom-right
      _corners.push_back(-y_vector * _mirror_distance_m - x_vector * _w_mirror_m / 2.0);
      _corners.push_back(-y_vector * _mirror_distance_m + x_vector * _w_mirror_m / 2.0);
      _corners.push_back(-y_vector * _mirror_distance_m + x_vector * _w_mirror_m / 2.0 + down_vector * _h_mirror_m);
      _corners.push_back(-y_vector * _mirror_distance_m - x_vector * _w_mirror_m / 2.0 + down_vector * _h_mirror_m);
   }
}

void MirrorMarkers::computePlanes()
{
   if (_corners.size() == 4)
   {
      if (_mirror_type == MirrorType::UP)
      {
         appendTriangleFace(_corners[0], _corners[3], _corners[2], _planes_markers);
         appendTriangleFace(_corners[2], _corners[1], _corners[0], _planes_markers);
      }
      else if (_mirror_type == MirrorType::DOWN)
      {
         appendTriangleFace(_corners[0], _corners[1], _corners[2], _planes_markers);
         appendTriangleFace(_corners[2], _corners[3], _corners[0], _planes_markers);
      }
   }
}
}  // namespace ouster_rviz
