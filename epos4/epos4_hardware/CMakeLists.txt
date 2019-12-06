cmake_minimum_required(VERSION 2.8.3)
project(epos4_hardware)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  epos4_library
  geometry_msgs
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES epos4_hardware 
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(${PROJECT_NAME}_node
#  src/epos4_hardware_node.cpp
#  src/epos4_driver.cpp
#  include/${PROJECT_NAME}/epos4_driver.hpp
#  include/${PROJECT_NAME}/epos4_parm.hpp
#)

#add_executable(${PROJECT_NAME}_4wheel_node
#  src/epos4_hardware_4wheel_node.cpp
#  src/epos4_driver.cpp
#  include/${PROJECT_NAME}/epos4_driver.hpp
#  include/${PROJECT_NAME}/epos4_parm.hpp
#)

add_executable(${PROJECT_NAME}_2wheel_node
  src/epos4_hardware_2wheel_node.cpp
  src/epos4_driver.cpp
#  src/CLog.cpp
  include/${PROJECT_NAME}/epos4_driver.hpp
  include/${PROJECT_NAME}/epos4_parm.hpp
#  include/${PROJECT_NAME}/CLog.hpp
)

#target_link_libraries(${PROJECT_NAME}_node
##    -lEposCmd
#    ${catkin_LIBRARIES}
#)

#target_link_libraries(${PROJECT_NAME}_4wheel_node
##    -lEposCmd
#    ${catkin_LIBRARIES}
#)

target_link_libraries(${PROJECT_NAME}_2wheel_node
#    -lEposCmd
    ${catkin_LIBRARIES}
)
#target_link_libraries(${PROJECT_NAME}_node
#  -lEposCmd
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_epos4_hardware.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
