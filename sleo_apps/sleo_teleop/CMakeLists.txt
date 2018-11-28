cmake_minimum_required(VERSION 2.8.3)
project(sleo_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs joy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs joy
  DEPENDS
)

###########
## Build ##
###########

add_executable(sleo_teleop_joy src/sleo_joy.cpp)
target_link_libraries(sleo_teleop_joy ${catkin_LIBRARIES})

add_executable(sleo_xbox_joy src/sleo_xbox_joy.cpp)
target_link_libraries(sleo_xbox_joy ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/sleo_teleop_key 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS sleo_teleop_joy sleo_xbox_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch param scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
