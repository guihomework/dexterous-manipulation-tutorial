cmake_minimum_required(VERSION 2.8.3)
project(multifinger_planning_marker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multifinger_planning_marker
  CATKIN_DEPENDS geometry_msgs interactive_markers moveit_core moveit_msgs moveit_ros_planning roscpp rospy std_msgs tf visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(multifinger_planning_marker src/multifinger_planning_marker.cpp)
#find_package(Boost REQUIRED COMPONENTS thread)
#include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(multifinger_planning_marker  ${catkin_LIBRARIES}) #${Boost_LIBRARIES}

