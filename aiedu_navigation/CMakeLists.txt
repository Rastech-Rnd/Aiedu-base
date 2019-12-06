cmake_minimum_required(VERSION 2.8.3)
project(aiedu_navigation)


find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aiedu_navigation
#  CATKIN_DEPENDS amcl map_server move_base 
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

