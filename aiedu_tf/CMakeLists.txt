##############################################################################
# CMake
##############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(aiedu_tf)

##############################################################################
# Catkin
##############################################################################
## Compile as C++11, supported in ROS Kinetic and newer
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  tf2
)


################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aiedu_tf
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf tf2
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(aiedu_tf_node src/aiedu_tf_node.cpp)
target_link_libraries(aiedu_tf_node ${catkin_LIBRARIES})
add_dependencies(aiedu_tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

################################################################################
# Install
################################################################################
install(TARGETS aiedu_tf_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
