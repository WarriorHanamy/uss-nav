cmake_minimum_required(VERSION 2.8.3)
project(decomp_ros_utils)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs decomp_ros_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
