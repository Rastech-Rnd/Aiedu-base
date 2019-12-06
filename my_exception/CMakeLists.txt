cmake_minimum_required(VERSION 2.8.3)
project(my_exception)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	move_base_msgs 
        roscpp
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_exception
  CATKIN_DEPENDS roscpp move_base_msgs std_srvs  
  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

#add_executable(my_exception src/move_clor.cpp)
add_executable(move_clro src/move_clor.cpp)

target_link_libraries(move_clro ${catkin_LIBRARIES})
