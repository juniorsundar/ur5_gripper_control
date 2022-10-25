cmake_minimum_required(VERSION 3.1.3)
project(ur5_control)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  roslib
  sensor_msgs
  std_msgs
  message_generation
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  FilterWorkspace.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge roscpp std_msgs sensor_msgs geometry_msgs message_runtime
#   DEPENDS OpenCV
)

include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(
  ${OpenCV_LIBRARY_DIRS}
  )
add_definitions(
  ${OpenCV_DEFINITIONS}
  )

catkin_install_python(PROGRAMS scripts/publish_octomap_cloud.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)