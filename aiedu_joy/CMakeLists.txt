cmake_minimum_required(VERSION 2.8.3)
project(aiedu_joy)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aiedu_joy
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
 

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(PROGRAMS scripts/create_udev_rules
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(FILES 99-turtlebot3-cdc.rules
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
################################################################################
# Test
################################################################################
