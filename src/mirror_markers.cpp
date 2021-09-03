#include "mirror_markers.h"
// namespace ouster
// {
MirrorMarkers::MirrorMarkers(const double &mirror_angle, const double &mirror_distance_m, const double &w_mirror_m,
                             const double &h_mirror_m)
{
   _mirror_angle      = mirror_angle;
   _mirror_distance_m = mirror_distance_m;
   _w_mirror_m        = w_mirror_m;
   _h_mirror_m        = h_mirror_m;

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
   _corners_markers.scale.y          = 0.005;
   _lines_markers.color.a            = 1.0;
   _lines_markers.color.r            = 0.8;
   _lines_markers.color.g            = 0.8;
   _lines_markers.color.b            = 0.8;
}

MirrorMarkers::~MirrorMarkers() {}

void MirrorMarkers::computeMirror(MirrorType mirror_type)
{
   _mirror_type = mirror_type;

   _corners_markers.points.clear();
   _lines_markers.points.clear();
   _planes_markers.points.clear();

   computeCorners();
   _lines_points = cornersToLines(_corners);

   for (uint i = 0; i < _lines_points.size(); i++) _corners_markers.points.push_back(_lines_points[i]);
   for (uint i = 0; i < _lines_points.size(); i++) _lines_markers.points.push_back(_lines_points[i]);
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

std::vector<geometry_msgs::Point> MirrorMarkers::cornersToLines(const std::vector<Eigen::Vector3f> &face_corners)
{
   std::vector<geometry_msgs::Point> lines_face;

   for (const auto &pt : face_corners)
   {
      auto pt_msg = geometry_msgs::Point();
      tf::pointEigenToMsg(pt.cast<double>(), pt_msg);
      lines_face.push_back(pt_msg);
   }

   std::vector<geometry_msgs::Point> up_line_aux;
   std::vector<geometry_msgs::Point> down_line_aux;
   for (uint32_t i = 0; i < face_corners.size(); ++i)
   {
      if (i % 2)
      {
         auto pt_msg = geometry_msgs::Point();
         tf::pointEigenToMsg(face_corners[i].cast<double>(), pt_msg);
         up_line_aux.push_back(pt_msg);
      }
      else
      {
         auto pt_msg = geometry_msgs::Point();
         tf::pointEigenToMsg(face_corners[i].cast<double>(), pt_msg);
         down_line_aux.push_back(pt_msg);
      }
   }

   lines_face.insert(lines_face.end(), up_line_aux.begin(), up_line_aux.end());
   lines_face.insert(lines_face.end(), down_line_aux.begin(), down_line_aux.end());

   return lines_face;
}

void MirrorMarkers::computeCorners()
{
   Eigen::Vector3f x_vector(1, 0, 0);
   Eigen::Vector3f y_vector(0, 1, 0);
   Eigen::Vector3f z_vector(0, 0, 1);

   Eigen::Vector3f up_vector(0, -cos(_mirror_angle), sin(_mirror_angle));
   Eigen::Vector3f down_vector(0, -cos(_mirror_angle), -sin(_mirror_angle));

   if (_mirror_type == MirrorType::UP)
   {
      // Top-left / Top-right / Bottom-left / Bottom-right
      _corners.push_back(-y_vector * _mirror_distance_m + x_vector * _w_mirror_m / 2.0 + up_vector * _h_mirror_m);
      _corners.push_back(-y_vector * _mirror_distance_m - x_vector * _w_mirror_m / 2.0 + up_vector * _h_mirror_m);
      _corners.push_back(-y_vector * _mirror_distance_m + x_vector * _w_mirror_m / 2.0);
      _corners.push_back(-y_vector * _mirror_distance_m - x_vector * _w_mirror_m / 2.0);
   }
   else if (_mirror_type == MirrorType::DOWN)
   {
      // Top-left / Top-right / Bottom-left / Bottom-right
      _corners.push_back(-y_vector * _mirror_distance_m + x_vector * _w_mirror_m / 2.0 + down_vector * _h_mirror_m);
      _corners.push_back(-y_vector * _mirror_distance_m - x_vector * _w_mirror_m / 2.0 + down_vector * _h_mirror_m);
      _corners.push_back(-y_vector * _mirror_distance_m + x_vector * _w_mirror_m / 2.0);
      _corners.push_back(-y_vector * _mirror_distance_m - x_vector * _w_mirror_m / 2.0);
   }
}
// }  // namespace ouster
