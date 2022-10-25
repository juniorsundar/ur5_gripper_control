cmake_minimum_required(VERSION 3.1.3)
project(ur5_gripper_control)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
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
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

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
  CATKIN_DEPENDS cv_bridge roscpp std_msgs sensor_msgs geometry_msgs message_runtime moveit_core moveit_visual_tools moveit_ros_planning_interface
  DEPENDS 
    EIGEN3
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(
  ${OpenCV_LIBRARY_DIRS}
  )
add_definitions(
  ${OpenCV_DEFINITIONS}
  )

add_executable(pick_place src/pick_place.cpp)
target_link_libraries(pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_install_python(PROGRAMS scripts/publish_octomap_cloud.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)