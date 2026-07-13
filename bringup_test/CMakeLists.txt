cmake_minimum_required(VERSION 3.10)
project(bringup_test)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch params rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
