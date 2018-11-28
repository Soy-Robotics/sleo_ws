cmake_minimum_required(VERSION 2.8.3)
project(sleo_driver)

find_package(catkin REQUIRED COMPONENTS controller_manager sleo_msgs roscpp serial hardware_interface)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  hardware_interface sleo_msgs roscpp serial
  DEPENDS Boost
)

# Build and install binary driver.
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_subdirectory(src)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

