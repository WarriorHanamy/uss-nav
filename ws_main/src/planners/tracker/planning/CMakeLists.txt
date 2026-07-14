cmake_minimum_required(VERSION 3.0.2)
project(planning)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  quadrotor_msgs
  mapping
  roscpp
  nodelet
  decomp_ros_utils
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS quadrotor_msgs mapping roscpp nodelet decomp_ros_utils
  LIBRARIES ${PROJECT_NAME} traj_opt
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${DECOMP_UTIL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Elastic tracker trajectory optimization (self-contained)
add_library(traj_opt
  src/traj_opt.cc
  src/traj_opt_fake.cc
)
target_link_libraries(traj_opt ${catkin_LIBRARIES})
add_dependencies(traj_opt ${catkin_EXPORTED_TARGETS})

# planning node
add_library(${PROJECT_NAME}_nodelet
src/${PROJECT_NAME}_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet
  traj_opt
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_nodelet
  ${catkin_EXPORTED_TARGETS}
)

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server traj_opt ${catkin_LIBRARIES})
add_dependencies(traj_server ${catkin_EXPORTED_TARGETS})

add_executable(play_bag_node src/play_bag_node.cpp)
target_link_libraries(play_bag_node ${catkin_LIBRARIES})
add_dependencies(play_bag_node ${catkin_EXPORTED_TARGETS})

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node traj_opt ${catkin_LIBRARIES})