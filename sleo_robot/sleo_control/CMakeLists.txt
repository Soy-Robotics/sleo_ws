cmake_minimum_required(VERSION 2.8.3)
project(sleo_control)

find_package(catkin REQUIRED COMPONENTS roslaunch roscpp geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS
)

roslaunch_add_file_check(launch)


install(DIRECTORY config launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

