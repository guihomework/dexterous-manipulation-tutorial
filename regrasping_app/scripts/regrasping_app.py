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

def subgroup_posture_target(grasp, group):
    joint_names = group.get_joints()
    posture={}
    for joint_name  in joint_names:
        for name, value in zip(grasp.grasp_posture.joint_names, grasp.grasp_posture.points[0].positions):
            if name == joint_name:
                posture[joint_name]=value
    return posture
            
    

def modify_grasp_reference_frame(grasp, frame):
    grasp.grasp_pose.header.frame_id = frame
    grasp.pre_grasp_approach.direction.header.frame_id = frame
    grasp.post_grasp_retreat.direction.header.frame_id = frame
    grasp.post_place_retreat.direction.header.frame_id = frame
    return grasp

if __name__ == "__main__":
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_regrasping_app', anonymous=True)
    rospack = rospkg.RosPack()
      
    resourcepath = rospack.get_path('regrasping_app')+"/../resources/objects/"
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    arm = MoveGroupCommander("arm")
    hand = MoveGroupCommander("hand")
    ring_finger = MoveGroupCommander("ring_finger")
    little_finger = MoveGroupCommander("little_finger")
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
    scene.add_box("ground", p, (3, 3, 0.02))
    
    # add the cup and pen
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    p.pose.position.x = 0.7
    p.pose.position.y = 0.7
    p.pose.position.z = 0.1
    p.pose.orientation.w = 1.0
    scene.add_mesh("cup",p,resourcepath+'cup.dae')
    
    p.pose.position.z = 0.15
    scene.add_mesh("pen",p,resourcepath+'pen.dae')
    
    rospy.sleep(2)
    grasps = load_grasps()
    top_grasp = modify_grasp_reference_frame (grasps['power_grasp_vertical'], "pen")
    two_finger_grasp = modify_grasp_reference_frame (grasps['two_finger_precision_horizontal'], "pen")
    three_finger_grasp = modify_grasp_reference_frame (grasps['three_finger_precision_horizontal'], "pen")
    
    #arm.set_named_target("gamma")
    #arm.go()
    rospy.sleep(1)
    arm.set_start_state_to_current_state()
    print "picking pen with side grasp"
    arm.set_planner_id("RRTstarkConfigDefault") #RRTstarkConfigDefault
    arm.set_planning_time(5.0)
    ret = arm.pick("pen",[two_finger_grasp])
    if ret != -1:
        rospy.sleep(2)
        arm.detach_object("pen")
        print "detachig pen"
        rospy.sleep(1)
        #hand.set_planner_id("RRTstarkConfigDefault") #RRTstarkConfigDefault
        #hand.set_named_target("three_finger_precision_grasp")
        ring_target_posture = subgroup_posture_target(three_finger_grasp,ring_finger)
        print "ring_target_posture ",ring_target_posture
        ring_finger.set_start_state_to_current_state()
        rospy.sleep(1)
        ring_finger.set_joint_value_target(ring_target_posture)
        print "result two finger ring ",ring_finger.go()
        #print "result two finger ",hand.go()
        rospy.sleep(2)
        #hand.set_named_target("three_finger_precision_grasp")
        #print "result three finger ", hand.go()
        #print "result two finger little ",little_finger.go()
        
    rospy.spin()
    roscpp_shutdown()
