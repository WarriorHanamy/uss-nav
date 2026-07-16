cmake_minimum_required(VERSION 2.8.3)
project(mission_executive)

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -Wall")
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  scene_graph
  global_belief
  decision_trace
  quadrotor_msgs
  traj_utils
  cv_bridge
  ego_planner
  mavros
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS global_belief decision_trace traj_utils ego_planner mavros sensor_msgs cv_bridge scene_graph
 DEPENDS OpenCV
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(mission_executive_node
  src/mission_executive_node.cpp 
  src/mission_fsm.cpp
  src/vla_search_map.cpp
)
add_dependencies(mission_executive_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(mission_executive_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # -ldw
  )

add_executable(rc_replan_trigger
  src/rc_replan_trigger.cpp
)
add_dependencies(rc_replan_trigger ${catkin_EXPORTED_TARGETS})

target_link_libraries(rc_replan_trigger
  ${catkin_LIBRARIES}
)
