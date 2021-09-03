cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_ros)

# ==== Requirements ====
find_package(PkgConfig REQUIRED)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)

find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             std_msgs
             sensor_msgs
             geometry_msgs
             pcl_ros
             pcl_conversions
             roscpp
             tf2
             tf2_ros
             tf2_geometry_msgs
             eigen_conversions
             visualization_msgs)

# ==== Options ====
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra)

# ==== Catkin ====
add_message_files(FILES PacketMsg.msg MirrorIndicesMsg.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ouster_ros
  CATKIN_DEPENDS
  roscpp
  message_runtime
  pcl_ros
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  eigen_conversions)

# ==== Libraries ====
add_library(ouster_ros
  STATIC
  src/ros.cpp
  src/packets_queue.cpp
  src/parameters.cpp
  src/client/client.cpp
  src/client/compat.cpp
  src/client/lidar_scan.cpp
  src/client/types.cpp
  )
target_link_libraries(
  ouster_ros
  PUBLIC ${catkin_LIBRARIES})
set_target_properties(ouster_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ouster_ros PUBLIC include ${catkin_INCLUDE_DIRS})
target_compile_options(ouster_ros PRIVATE "$<$<CONFIG:DEBUG>:${FLAGS_DEBUG}>")
target_compile_options(ouster_ros
                       PRIVATE "$<$<CONFIG:RELEASE>:${FLAGS_RELEASE}>")
target_compile_options(ouster_ros PRIVATE "${FLAGS}")
add_dependencies(ouster_ros ${PROJECT_NAME}_gencpp)

file(GLOB_RECURSE MIR_SRCS "src/mirror_markers.cpp" "src/os_split_cloud_node.cpp")
file(GLOB_RECURSE MIR_HDRS "src/mirror_markers.h" "src/mirror_markers.h")
add_library(mirrors_markers ${MIR_SRCS} ${MIR_HDRS})
add_dependencies(mirrors_markers ${catkin_EXPORTED_TARGETS})
target_link_libraries(mirrors_markers  PUBLIC ${catkin_LIBRARIES})
set_target_properties(mirrors_markers PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mirrors_markers PUBLIC include ${catkin_INCLUDE_DIRS})

# ==== Executables ====
add_executable(os_node src/os_node.cpp)
target_link_libraries(os_node ouster_ros ${catkin_LIBRARIES})
target_include_directories(os_node PRIVATE include ${catkin_INCLUDE_DIRS})
add_dependencies(os_node ${PROJECT_NAME}_gencpp)

add_executable(os_split_cloud_node src/os_split_cloud_node.cpp)
target_link_libraries(os_split_cloud_node ${catkin_LIBRARIES} mirrors_markers)
target_include_directories(os_split_cloud_node PRIVATE include ${catkin_INCLUDE_DIRS})
add_dependencies(os_split_cloud_node ${catkin_EXPORTED_TARGETS})

add_executable(os_img_node src/os_img_node.cpp)
target_link_libraries(os_img_node ouster_ros ${catkin_LIBRARIES})
target_include_directories(os_img_node PRIVATE include ${catkin_INCLUDE_DIRS})
add_dependencies(os_img_node ${PROJECT_NAME}_gencpp)

# ==== Install ====
install(
  TARGETS os_node os_img_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES launch/ouster.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
