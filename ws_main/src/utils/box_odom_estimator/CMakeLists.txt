cmake_minimum_required(VERSION 3.0.2)
project(box_odom_estimator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  quadrotor_msgs
  roscpp
  scene_graph
  sensor_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS nav_msgs quadrotor_msgs roscpp scene_graph sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(bbox_lidar_target_estimator src/bbox_lidar_target_estimator.cpp)
add_dependencies(bbox_lidar_target_estimator ${catkin_EXPORTED_TARGETS})
target_link_libraries(bbox_lidar_target_estimator ${catkin_LIBRARIES})

catkin_install_python(
  PROGRAMS scripts/moge_target_estimator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS bbox_lidar_target_estimator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
