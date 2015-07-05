#!/usr/bin/env python
 
import rospy
import numpy
import logging
import rospkg, genpy
  
import sys
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander , roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState, Grasp
from moveit_msgs.msg import DisplayRobotState, DisplayTrajectory
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

from sr_grasp.utils import mk_grasp

import copy


def load_grasps():
    grasp_yaml = rospy.get_param("~grasps")
    grasps={}
    for g in grasp_yaml:
        grasp = Grasp()
        genpy.message.fill_message_args(grasp, g)
            
        if grasp.id is None or grasp.id == "":
            raise Exception("Grasp has no id")
        else:
            grasps[grasp.id] = grasp
    return grasps

if __name__ == "__main__":
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_regrasping_app', anonymous=True)
    rospack = rospkg.RosPack()
      
    resourcepath = rospack.get_path('regrasping_app')+"/../resources/objects/"
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    arm = MoveGroupCommander("arm")
    hand = MoveGroupCommander("hand")
    #group.set_planner_id("arm_RRTkConfigDefault")    
    """display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    DisplayTrajectory)
    """
    rospy.sleep(1)     
    # clean the scene
    scene.remove_world_object("ground")
    scene.remove_world_object("cup")
    scene.remove_world_object("pen")
    rospy.sleep(1)  
    
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    
    p.pose.position.x = 0
    p.pose.position.y = 0
    # offset such that the box is below ground (to prevent collision with the robot itself)
    p.pose.position.z = -0.05
    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1
    #scene.add_box("ground", p, (3, 3, 0.02))
    
    # add the cup and pen
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    p.pose.position.x = 0.7
    p.pose.position.y = 0.7
    p.pose.position.z = 0.1
    p.pose.orientation.w = 1.0
    scene.add_mesh("cup",p,resourcepath+'cup.dae')
    rospy.sleep(2)
    grasps = load_grasps()
    side_grasp = grasps['power_grasp_vertical']
    side_grasp.grasp_pose.header.frame_id = "cup"
    
    arm.set_named_target("gamma")
    arm.go()
    rospy.sleep(1)
    arm.set_start_state_to_current_state()

    #robot.arm.set_planner_id("RRTstarkConfigDefault") #RRTstarkConfigDefault
    #robot.arm.set_planning_time(15.0)
    #robot.arm.pick("cup",[side_grasp])
    
    hand.set_planner_id("RRTstarkConfigDefault") #RRTstarkConfigDefault
    hand.set_named_target("open")
    hand.plan()
    rospy.sleep(2)
    hand.set_named_target("three_finger_precision_grasp")
    hand.plan()
    
        
    rospy.spin()
    roscpp_shutdown()
