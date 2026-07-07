cmake_minimum_required(VERSION 3.5)
project(scene_graph)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -O3")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -Wall")

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    message_generation 
)


include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
)

add_message_files(
  FILES
  WordVector.msg
  EncodeMask.msg
  PromptMsg.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES scene_graph
  CATKIN_DEPENDS message_generation message_runtime 
  # DEPENDS system_lib
)
