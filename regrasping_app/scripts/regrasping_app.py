#!/usr/bin/env python
 
import sys
import rospy
import rospkg, genpy
import yaml
import copy
  
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_commander import roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState, Grasp
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

def load_grasps(resourcepath):
    """
    load and fill a list of Grasp.msg from a yaml file 
    """
    grasp_yaml = yaml.load(open(resourcepath+'grasps/grasps.yaml'))
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
    """
    extracts the joint targets of a grasp for a particular subgroup
    """
    joint_names = group.get_joints()
    posture={}
    for joint_name  in joint_names:
        for name, value in zip(grasp.grasp_posture.joint_names, grasp.grasp_posture.points[0].positions):
            if name == joint_name:
                posture[joint_name]=value
    return posture

def modify_grasp_reference_frame(grasp, frame):
    """
    change the frame_id of every element of a Grasp.msg
    """
    grasp.grasp_pose.header.frame_id = frame
    grasp.pre_grasp_approach.direction.header.frame_id = frame
    grasp.post_grasp_retreat.direction.header.frame_id = frame
    grasp.post_place_retreat.direction.header.frame_id = frame
    return grasp

if __name__ == "__main__":
  
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_regrasping_app', anonymous=True)
    rospy.loginfo("Starting grasp app")
    
    # Access the planning scene
    scene = PlanningSceneInterface()
    
    rospy.sleep(1)     
    # clean the scene
    scene.remove_world_object("ground")
    scene.remove_world_object("cup")
    scene.remove_world_object("pen")
    
    rospy.sleep(1)  
    
    # create a pose
    p = PoseStamped()
    p.header.frame_id = "world" #robot.get_planning_frame()
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = -0.05
    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1
    
    # add a box there
    scene.add_box("ground", p, (3, 3, 0.02))
    
    # access some meshes
    rospack = rospkg.RosPack()
    resourcepath = rospack.get_path('regrasping_app')+"/../resources/"
    
    # modify the pose
    p.pose.position.x = 0.7
    p.pose.position.y = 0.7
    p.pose.position.z = 0.0
    # add the cup
    scene.add_mesh("cup",p,resourcepath+'objects/cup.dae')
    
    # modify the pose
    p.pose.position.x = 0.72
    p.pose.position.z = 0.05
    # add the pen
    scene.add_mesh("pen",p,resourcepath+'objects/pen.dae')
    rospy.sleep(1)
     
    # print the existing groups 
    robot = RobotCommander()
    print "Available groups: ",robot.get_group_names()
    
    # setup the arm group and its planner
    arm = MoveGroupCommander("arm")
    arm.set_start_state_to_current_state()
    arm.set_planner_id("RRTstarkConfigDefault") 
    arm.set_planning_time(5.0)
    arm.detach_object("pen")
    
    # set the arm to a safe target
    arm.set_named_target("gamma")
    # plan and execute the motion 
    arm.go()
    
    # setup the hand group and its planner
    hand = MoveGroupCommander("hand")
    hand.set_start_state_to_current_state()
    hand.set_planner_id("LBKPIECEkConfigDefault") 
    hand.set_planning_time(10.0)
    
    # set the hand to a safe target
    hand.set_named_target("open")
    # plan and execute the motion
    hand.go()

    # load grasps
    grasps = load_grasps(resourcepath)
    two_finger_grasp = modify_grasp_reference_frame (grasps['two_finger_precision_horizontal'], "pen")
    four_finger_grasp = modify_grasp_reference_frame (grasps['four_finger_precision_horizontal'], "pen") 
      
    # plan and pick the object with two fingers first
    ret = arm.pick("pen",[two_finger_grasp])
    rospy.sleep(1)

    if ret != -1:
        rospy.sleep(2)
        # do some in-hand movements there  
        
          
    rospy.spin()
    roscpp_shutdown()
    rospy.loginfo("Stopping grasp app")
