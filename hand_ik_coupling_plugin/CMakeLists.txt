cmake_minimum_required(VERSION 2.8.3)
project(hand_ik_coupling_plugin)
find_package(catkin REQUIRED COMPONENTS roscpp rostest tf urdf kdl_parser kdl_coupling sensor_msgs tf_conversions moveit_core geometry_msgs pluginlib moveit_msgs moveit_ros_planning)
find_package(orocos_kdl REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

catkin_package(
    DEPENDS orocos_kdl
    CATKIN_DEPENDS roscpp tf urdf kdl_parser kdl_coupling sensor_msgs tf_conversions moveit_core geometry_msgs pluginlib moveit_msgs moveit_ros_planning
    INCLUDE_DIRS include
    LIBRARIES hand_kinematics hand_kinematics_coupling_lib
)

add_library(hand_ik_coupling_plugin src/hand_ik_coupling_plugin.cpp
    src/hand_ik_utils.cpp)
add_dependencies(hand_ik_coupling_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(hand_ik_coupling_plugin ${catkin_LIBRARIES} ${moveit_core_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(TARGETS hand_ik_coupling_plugin
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
 FILES
   hand_ik_coupling_plugin.xml
 DESTINATION
   ${CATKIN_PACKAGE_SHARE_DESTINATION})
