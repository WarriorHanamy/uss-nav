cmake_minimum_required(VERSION 2.8.3)
project(camera_fov)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES camera_fov
 CATKIN_DEPENDS 
 DEPENDS system_lib
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(camera_fov
  src/camera_fov.cpp 
  )
target_link_libraries(camera_fov 
  ${catkin_LIBRARIES}
  )


