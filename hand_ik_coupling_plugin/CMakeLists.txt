cmake_minimum_required(VERSION 2.8.3)
project(hand_ik_coupling_plugin)
find_package(catkin REQUIRED COMPONENTS roscpp rostest tf urdf kdl_parser kdl_coupling_extension sensor_msgs tf_conversions moveit_core geometry_msgs pluginlib moveit_msgs moveit_ros_planning)
find_package(orocos_kdl REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

catkin_package(
    DEPENDS orocos_kdl
    CATKIN_DEPENDS roscpp tf urdf kdl_parser kdl_coupling_extension sensor_msgs tf_conversions moveit_core geometry_msgs pluginlib moveit_msgs moveit_ros_planning
    INCLUDE_DIRS include
    LIBRARIES libmoveit_kinematics_plugin_loader
)

add_library(hand_ik_coupling_plugin src/hand_ik_coupling_plugin.cpp
    src/hand_ik_utils.cpp)
add_dependencies(hand_ik_coupling_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(hand_ik_coupling_plugin ${catkin_LIBRARIES} ${moveit_core_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_rostest_gtest(test_hand_ik_as_plugin test/hand_kinematics_plugin_test.test test/test_kinematics_as_plugin.cpp)
target_link_libraries(test_hand_ik_as_plugin ${moveit_core_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(test_hand_ik_as_plugin libmoveit_kinematics_plugin_loader hand_kinematics_gencpp ${catkin_EXPORTED_TARGETS})

install(TARGETS hand_ik_coupling_plugin
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
 FILES
   hand_ik_coupling_plugin.xml
 DESTINATION
   ${CATKIN_PACKAGE_SHARE_DESTINATION})
