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
             tf2_geometry_msgs)

# ==== Options ====
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra)

# ==== Build type ====
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)
IF(DEFINED CMAKE_BUILD_TYPE)
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
ENDIF()

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
ENDIF()

IF(NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug" AND
   NOT ${CMAKE_BUILD_TYPE} STREQUAL "Release")
  MESSAGE(FATAL_ERROR "Only Release and Debug build types are allowed.")
ENDIF()

# ==== Catkin ====
add_message_files(FILES PacketMsg.msg)
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
  geometry_msgs)

# ==== Libraries ====
add_library(ouster_ros
  STATIC
  src/ros.cpp
  src/packets_queue.cpp
  src/parameters.cpp
  src/client/client.cpp
  src/client/compat.cpp
  src/client/lidar_scan.cpp
  src/client/types.cpp)

target_link_libraries(
  ouster_ros
  PUBLIC ${catkin_LIBRARIES} jsoncpp)
set_target_properties(ouster_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ouster_ros PUBLIC include ${catkin_INCLUDE_DIRS})
target_compile_options(ouster_ros PRIVATE "$<$<CONFIG:DEBUG>:${FLAGS_DEBUG}>")
target_compile_options(ouster_ros
                       PRIVATE "$<$<CONFIG:RELEASE>:${FLAGS_RELEASE}>")
target_compile_options(ouster_ros PRIVATE "${FLAGS}")
add_dependencies(ouster_ros ${PROJECT_NAME}_gencpp)

# ==== Executables ====
add_executable(os_node src/os_node.cpp)
target_link_libraries(os_node ouster_ros ${catkin_LIBRARIES})
target_include_directories(os_node PRIVATE include ${catkin_INCLUDE_DIRS})
add_dependencies(os_node ${PROJECT_NAME}_gencpp)

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
