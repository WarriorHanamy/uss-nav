cmake_minimum_required(VERSION 3.0.2)
project(fake_planning)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  quadrotor_msgs
  fake_mapping
  roscpp
  nodelet
  decomp_ros_utils
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS quadrotor_msgs fake_mapping roscpp nodelet decomp_ros_utils
  LIBRARIES ${PROJECT_NAME} fake_traj_opt
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${DECOMP_UTIL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Elastic tracker traj_opt (simulation, self-contained)
add_library(fake_traj_opt
  src/fake_traj_opt.cc
  src/fake_traj_opt_fake.cc
)
target_link_libraries(fake_traj_opt ${catkin_LIBRARIES})
add_dependencies(fake_traj_opt ${catkin_EXPORTED_TARGETS})

# fake_planning node
add_library(${PROJECT_NAME}_nodelet
src/${PROJECT_NAME}_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet
  fake_traj_opt
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_nodelet
  ${catkin_EXPORTED_TARGETS}
)

add_executable(fake_traj_server src/fake_traj_server.cpp)
target_link_libraries(fake_traj_server fake_traj_opt ${catkin_LIBRARIES})
add_dependencies(fake_traj_server ${catkin_EXPORTED_TARGETS})
