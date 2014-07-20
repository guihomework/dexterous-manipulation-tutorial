// BSD license 
// majority of this code comes from package urdf_tool/arm_kinematics
// written by David Lu!!
//
// Modified by Juan A. Corrales, ISIR, UPMC
// -Added support for coupled joints of Shadow Hand. Now, the coupling is a fixed 1:1 value
// Modified by Guillaume WALCK (UPMC) 2012 
// - Turned it into a kinematics_plugin

#include <cstring>
#include <ros/ros.h>
#include <hand_ik_coupling_plugin/hand_ik_coupling_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit_msgs/MoveItErrorCodes.h>

using std::string;

namespace hand_kinematics {
//register the plugin
PLUGINLIB_EXPORT_CLASS( hand_kinematics::HandIKCouplingPlugin, kinematics::KinematicsBase)

  // Constructor
  HandIKCouplingPlugin::HandIKCouplingPlugin(){}
 
  // Initialize the plugin by setting the robot description name to be loaded
  // the joint group to consider
  // the base or root frame
  // the tip frame
  // search discretization is not used in this plugin
  bool HandIKCouplingPlugin::initialize(const std::string& robot_description,
                                          const std::string& group_name,
                                          const std::string& base_frame,
                                          const std::string& tip_frame,
                                          double search_discretization)
  {
    // Store the given values in the plugin
    setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);
    urdf::Model robot_model;
    std::string xml_string;
    
    // Handle to access the parameters for this group
    ros::NodeHandle private_handle("~/"+group_name);
    
    // Load the robot model
    while(!loadRobotModel(private_handle,robot_model,base_frame_,tip_frame_,xml_string) && private_handle.ok())
    {
      ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
      ros::Duration(0.5).sleep();
    }

    // Loading KDL Tree
    if(!getKDLChain(xml_string,base_frame_,tip_frame_,kdl_chain_))
    {
      ROS_ERROR("Could not load kdl tree");
      return false;
    }
  
    // Define coupling matrix for fingers ff, mf, rf: 
    // their first two joints (J1 and J2) are coupled 
    // while J3 and J4 are independent.
    // The rows of coupling matrix correspond to all joints (unlocked ones)
    // while the columns correspond to independent joints (not coupled).
    
    // LF has 5 joints and specific update coupling function
    if(tip_frame_.find("lftip")!=string::npos)
    {
      // Assign update function for dynamic coupling
      kdl_chain_.setUpdateCouplingFunction(updateCouplingLF); 
      dimension_=5;
    }
    else
    {
      if(tip_frame_.find("fftip")!=string::npos||
      tip_frame_.find("mftip")!=string::npos ||
      tip_frame_.find("rftip")!=string::npos
      )
      {
        // Assign update function for dynamic coupling
        kdl_chain_.setUpdateCouplingFunction(updateCoupling); 
        dimension_=4;
      }
      else
      {
        ROS_ERROR("Cannot solve for %s",tip_frame_.c_str());
        return false;
      }
    }   

    // Task space weighting matrix: 
    // The IK only consider translation components, other are set to 0.
    Eigen::MatrixXd Mx(6,6); 
    for(unsigned int i=0; i < 6; i++)
    {
      for(unsigned int j=0; j < 6; j++)
      {
        Mx(i,j)= 0.0;
      }
    }
    
    Mx(0,0)= 1.0; // coordinate X
    Mx(1,1)= 1.0; // coordinate Y
    Mx(2,2)= 1.0; // coordinate Z
    Mx(3,3)= 0.0; // rotation X
    Mx(4,4)= 0.0; // rotation Y
    Mx(5,5)= 0.0; // rotation Z
      
    // Set solver parameters
    int maxIterations=1000; // solver max loops
    double epsilon=0.001; // loop finish condition
    double lambda=0.01; // used in WDLS algorithm;

    // Init the solver infos
    init_ik(robot_model,base_frame_,tip_frame_, joint_min_,joint_max_,ik_solver_info_);
    
    // Build the solvers, passing the solving parameters
    fk_solver = new KDL::ChainFkSolverPos_recursive(kdl_chain_); 
    // inverse kinematic for velocity solver, using wdls (parameters lambda and Mx)
    ik_solver_vel= new KDL::ChainIkSolverVel_wdls_coupling(kdl_chain_,epsilon,maxIterations);
    ik_solver_vel->setLambda(lambda);
    ik_solver_vel->setWeightTS(Mx);
    // inverse kinematic for position, passing the ik vel and other parameters
    ik_solver_pos= new KDL::ChainIkSolverPos_NR_JL_coupling(kdl_chain_,
       joint_min_,joint_max_,*fk_solver, *ik_solver_vel, maxIterations, epsilon);
        
    return true;
  }

  // Main function called by the planner for Inverse kinematics
  bool HandIKCouplingPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                             const std::vector<double> &ik_seed_state,
                                             std::vector<double> &solution,
                                             moveit_msgs::MoveItErrorCodes &error_code,
                                             const kinematics::KinematicsQueryOptions &options) const
  {
    // Get the pose in KDL format
    KDL::Frame pose_desired;
    tf::poseMsgToKDL(ik_pose, pose_desired);

    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(dimension_);
    
    // Get the seed state in KDL format
    for(int i=0; i < dimension_; i++)
    {
      jnt_pos_in(i) = ik_seed_state[i];
    }
    // Enforce the coupling in case the seed state given value do not
    hand_kinematics::maintainCoupling(jnt_pos_in, tip_frame_); 
    
    
    int ik_valid=-1;
    // restart 10 times with different rand if no success
    for(int i=0; i < 10 && ik_valid < 0; i++)
    {
      // DO the IK
      ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in, pose_desired, jnt_pos_out);
      if(!ik_valid)
      {
        // generate a new random seed
        generateRandomJntSeed(jnt_pos_in);
        // Enforce the coupling
        hand_kinematics::maintainCoupling(jnt_pos_in, tip_frame_);
      }
    }
    
    // Extract the result
    if(ik_valid >= 0)
    {
      solution.resize(dimension_);
      for(int i=0; i < dimension_; i++)
      {
        solution[i] = jnt_pos_out(i);
      }
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      return true;
    }
    else
    {
      error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }
  }

  bool HandIKCouplingPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
  {
    static IKCallbackFn solution_callback = 0;
    static std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
  }  
    
  bool HandIKCouplingPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limits,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
  {
    static IKCallbackFn solution_callback = 0;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
  }  
  
  bool HandIKCouplingPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
  {
    static std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
  }
    
  
  bool HandIKCouplingPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limits,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
  {
    return getPositionIK(ik_pose,ik_seed_state,solution,error_code,options);
  }
  
  bool HandIKCouplingPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses) const
  {
    KDL::Frame p_out;
    KDL::JntArray jnt_pos_in;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    jnt_pos_in.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      jnt_pos_in(i) = joint_angles[i];
    }

    poses.resize(link_names.size());

    bool valid = true;

    for(unsigned int i=0; i < poses.size(); i++)
    {
      ROS_DEBUG("End effector index: %d",hand_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i]));
      if(fk_solver->JntToCart(jnt_pos_in,p_out,hand_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i])) >=0)
      {
        tf::poseKDLToMsg(p_out,poses[i]);
      }
      else
      {
        ROS_ERROR("Could not compute FK for %s",link_names[i].c_str());
        valid = false;
      }
    }
    return valid;
  }

  const std::vector<std::string> &HandIKCouplingPlugin::getJointNames() const
  {
    return ik_solver_info_.joint_names;
  }

  const std::vector<std::string> &HandIKCouplingPlugin::getLinkNames() const
  {
    return fk_solver_info_.link_names;
  }
  
  void HandIKCouplingPlugin::generateRandomJntSeed(KDL::JntArray &jnt_pos_in) const
  {
    for(int i=0; i < dimension_; i++)
    {
      double min= ik_solver_info_.limits[i].min_position;
      double max= ik_solver_info_.limits[i].max_position;
      double r= min + ((double)rand()) / RAND_MAX *(max-min);
      jnt_pos_in(i)= r;
    }
  }


}//end namespace 


