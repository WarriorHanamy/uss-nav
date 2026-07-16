cmake_minimum_required(VERSION 2.8.3)
project(decision_trace)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
