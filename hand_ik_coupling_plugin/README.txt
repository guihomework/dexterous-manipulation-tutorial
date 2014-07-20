Copyright UPMC 2012
Updated by Guillaume WALCK CITEC University Bielefeld 2014

DESCRIPTION
-----------
Hand_kinematics provides inverse kinematics for the Shadow Hand fingers and thumb, coping with coupled joints in the fingers. This simplified version does not handle the thumb
The IK services requires a 3D pose to be requested and not a 6D pose since 6D requests are most of the time unfeasible (only 3 to 5 DOF)
However, the request pose message must be 6D, only 3D translation part will be considered.

This code is based on some modified functions taken out of kdl and augmented with coupling possibilities. These functions are in package kdl_coupling, 
under the same KDL:: namespace and can work together with standard KDL functionnalities. Indeed all the functions created have different names.


PRE-REQUEST
-----------
This package provides a plugin for constraint aware kinematics that depends on moveit_core package.

INSTALL
-------
make sure you have kdl_coupling in the workspace
use catkin_make

USAGE
-----
No launch file provided, this is a simple plugin.

TEST
  * catkin_make tests
  * catkin_make run_tests_hand_ik_coupling_plugin