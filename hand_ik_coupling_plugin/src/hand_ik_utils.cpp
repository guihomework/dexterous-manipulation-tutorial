/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2008, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Sachin Chitta */
/* Modified by Guillaume Walck for Shadow Hands */

#include <hand_ik_coupling_plugin/hand_ik_utils.h>

namespace hand_kinematics
{
	static const double IK_DEFAULT_TIMEOUT = 10.0;
  
  // Load the robot model from the URDF on the parameter server and the root and tip names
	bool loadRobotModel(ros::NodeHandle node_handle, urdf::Model &robot_model, 
        std::string &root_name, std::string &tip_name, std::string &xml_string)
  {
    std::string urdf_xml,full_urdf_xml;
    node_handle.param("urdf_xml",urdf_xml,std::string("robot_description"));
    node_handle.searchParam(urdf_xml,full_urdf_xml);
    TiXmlDocument xml;
    ROS_DEBUG("Reading xml file from parameter server\n");
    std::string result;
    if (node_handle.getParam(full_urdf_xml, result))
      xml.Parse(result.c_str());
    else
    {
      ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
      return false;
    }
    xml_string = result;
    TiXmlElement *root_element = xml.RootElement();
    TiXmlElement *root = xml.FirstChildElement("robot");
    if (!root || !root_element)
    {
      ROS_FATAL("Could not parse the xml from %s\n", urdf_xml.c_str());
      exit(1);
    }
    robot_model.initXml(root);
    
   	if(root_name.find("palm")==std::string::npos) {
			ROS_FATAL("HANDIK: Current solver can only resolve to root frame = palm");
			return false;
		}
	
    if(tip_name.find("tip")==std::string::npos) {
			ROS_FATAL("Current solver can only resolve to one of the tip frames");
			return false;
		}
		if(tip_name.find("fftip")==std::string::npos &&
       tip_name.find("mftip")==std::string::npos &&
       tip_name.find("rftip")==std::string::npos &&
       tip_name.find("lftip")==std::string::npos){
			ROS_FATAL("Name of distal frame does not match any finger");
			return false;	
		}
    return true;
  }

  // fill a KDL chain from the URDF xml file
	bool getKDLChain(const std::string &xml_string, const std::string &root_name, const std::string &tip_name, KDL::Chain &kdl_chain)
  {
    // create robot chain from root to tip
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(xml_string, tree))
    {
      ROS_ERROR("Could not initialize tree object");
      return false;
    }
    if (!tree.getChain(root_name, tip_name, kdl_chain))
    {
      ROS_ERROR_STREAM("Could not initialize chain object for base " << root_name << " tip " << tip_name);
      return false;
    }
    return true;
  }

  // Get the index of the segment in the kdl chain
	int getKDLSegmentIndex(const KDL::Chain &chain,
                         const std::string &name)
  {
    int i=0; // segment number
    while(i < (int)chain.getNrOfSegments())
    {
      if(chain.getSegment(i).getName() == name)
      {
        return i+1;
      }
      i++;
    }
    return -1;
  }
  
  // Coupling update function. 
  // Currently using a static matrix, 
  // but can be different depending on the current joint position
	Eigen::MatrixXd updateCoupling(const KDL::JntArray& q)
	{
		Eigen::MatrixXd cm(4,3);
		for (unsigned int i =0; i<4; i++)
			for (unsigned int j=0; j<3; j++)
							cm(i,j) = 0.0;

		cm(0,0) = 1.0; // J4
		cm(1,1) = 1.0; // J3
		cm(2,2) = 1.0; // J2
		cm(3,2) = 1.0; // J1

		return cm;
	}

  // Coupling update function for LF 
	Eigen::MatrixXd updateCouplingLF(const KDL::JntArray& q)
	{
		Eigen::MatrixXd cm(5,4);
		for (unsigned int i =0; i<5; i++)
				for (unsigned int j=0; j<4; j++)
							cm(i,j) = 0.0;

		cm(0,0) = 1.0; // J5
		cm(1,1) = 1.0; // J4
		cm(2,2) = 1.0; // J3
		cm(3,3) = 1.0; // J2
		cm(4,3) = 1.0; // J1

		return cm;
	}

  // Initialize the IK solver with joint limits, root and tip names
  bool init_ik(urdf::Model &robot_model, const std::string &root_name, const std::string &tip_name, KDL::JntArray &joint_min, KDL::JntArray &joint_max,  moveit_msgs::KinematicSolverInfo &info )
  {
    unsigned int num_joints = 0;
    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint;

    urdf::Vector3 length;

    while (link && link->name != root_name) 
    {
      joint = robot_model.getJoint(link->parent_joint->name);
      if (!joint) {
          ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
          return false;
      }
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
          ROS_DEBUG( "adding joint: [%s]", joint->name.c_str() );
          num_joints++;
      }
      link = robot_model.getLink(link->getParent()->name);
    }

    joint_min.resize(num_joints);
    joint_max.resize(num_joints);
    info.joint_names.resize(num_joints);
    info.link_names.resize(num_joints);
    info.limits.resize(num_joints);

    link = robot_model.getLink(tip_name);
    unsigned int i = 0;
    while (link && i < num_joints) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_DEBUG( "getting bounds for joint: [%s]", joint->name.c_str() );

            float lower, upper;
            int hasLimits;
            if ( joint->type != urdf::Joint::CONTINUOUS ) {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
                hasLimits = 1;
            } else {
                lower = -M_PI;
                upper = M_PI;
                hasLimits = 0;
            }
            int index = num_joints - i -1;
            joint_min.data[index] = lower;
            joint_max.data[index] = upper;
            info.joint_names[index] = joint->name;
            info.link_names[index] = link->name;
            info.limits[index].joint_name = joint->name;
            info.limits[index].has_position_limits = hasLimits;
            info.limits[index].min_position = lower;
            info.limits[index].max_position = upper;
            i++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }
    return true;

  }
  
  // enforce the coupling on the joint angles (used after randomizing the joint values)
  void maintainCoupling(KDL::JntArray &jnt_pos_in, const std::string &tip_name)
  {
    // maintain 1:1 coupling
    if(tip_name.find("thtip")==std::string::npos && tip_name.find("lftip")==std::string::npos)
    {
      jnt_pos_in(3)=jnt_pos_in(2);
    }
    else if(tip_name.find("lftip")!=std::string::npos )
      jnt_pos_in(4)=jnt_pos_in(3);
  }
  
}
