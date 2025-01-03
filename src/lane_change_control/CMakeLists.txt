cmake_minimum_required(VERSION 3.0.2)
project(lane_change_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  pcl_ros
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs tf pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(lane_change_node 
  src/lane_change_node.cpp
  src/lane_change_controller.cpp
)

target_link_libraries(lane_change_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
) 