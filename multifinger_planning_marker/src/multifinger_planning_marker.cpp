/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
  Modified by Guillaume WALCK (UPMC) to control the shadow arm
  April 2012
	Modified to use it for shadow hand finger control
  Modified to control multifinger start/goal planning states
*/

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

using namespace visualization_msgs;
using namespace interactive_markers;

// Interactive marker server pointer to access the server from any function
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
float marker_pos = 0;

// publisher to connect to the planning markers
ros::Publisher moveit_marker_publisher;

// menu and related variables to connect and disconnect the marker from the fingers
MenuHandler menu_handler;
MenuHandler::EntryHandle h_attach;
bool control_goal = true;
bool attached = false;

std::string joint_prefix="rh_";
std::string base_frame = joint_prefix + "palm";

// The main processing function of the interactive markers.
// Called on events/incoming messages of our own marker
void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  // Process the different type of events
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_DEBUG_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;
      
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_DEBUG_STREAM( s.str() << ": mouse down." );
      break;

    // we react mainly on mouse up button
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_DEBUG_STREAM( s.str() << ": mouse up." );

      // if attached
      if(attached)
      {
        // check if start or goal is clicked, will move goal if both selected
        std::string name_prefix;
        if(control_goal)
          name_prefix="EE:goal_";
        else
          name_prefix="EE:start_";
          
        // Simulated a move action on the fftip planning interactive marker
        visualization_msgs::InteractiveMarkerFeedback markerFeedBack;
        markerFeedBack.marker_name=name_prefix+joint_prefix+"fftip";
        markerFeedBack.header = base_frame;
        markerFeedBack.control_name="move";
        markerFeedBack.event_type=visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE;
        
        // Copy the current info of our marker and 
        // add a positive offset for the planning marker to go to
        markerFeedBack.client_id=feedback->client_id;
        markerFeedBack.pose=feedback->pose;
        markerFeedBack.pose.position.z+=0.010;
        markerFeedBack.header.seq=feedback->header.seq;
        
        // Fake the mouse pointing        
        markerFeedBack.mouse_point.x=0.0388541817665;
        markerFeedBack.mouse_point.y=-0.0139917135239;
        markerFeedBack.mouse_point.z=0.444140136242;
        markerFeedBack.mouse_point_valid=true;
        
        // Publish the marker (will move the planning marker)
        moveit_marker_publisher.publish(markerFeedBack);
        
        // prepare a similar marker for the thumb with a negative offset
        // (from below)
        markerFeedBack.marker_name=name_prefix+joint_prefix+"thtip";
        markerFeedBack.pose.position.z-=0.020;
        // Publish the marker (will move the planning marker)
        moveit_marker_publisher.publish(markerFeedBack);
      }
        
      break;
  }
  server->applyChanges();
}

// Callback when the attach menu entry was called. Switches attached status
void attachCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(attached)
  {
    menu_handler.setCheckState(h_attach,MenuHandler::UNCHECKED);
    attached=false;
  }
  else
  {
    menu_handler.setCheckState(h_attach,MenuHandler::CHECKED);
    attached=true;
  }
  
  menu_handler.reApply( *server );
  server->applyChanges();
}

// A box marker creator, center of the interactive marker
Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.25;
  marker.scale.y = msg.scale * 0.25;
  marker.scale.z = msg.scale * 0.25;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

// Add control to the interactive marker
InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

// Create the actual marker with a name and a frame_id it gives position from
void make3DofMarker(std::string name, std::string frame_id)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/"+frame_id;

  // Initial position of the marker on the fftip initial pose.
  int_marker.pose.position.x = 0.033;
  int_marker.pose.position.y = 0.0;
  int_marker.pose.position.z = 0.191;
 
  int_marker.scale = 0.05;

  int_marker.name = name+"_control";
  int_marker.description = name+" 3-DOF Control";

  // Insert a box and a control
  makeBoxControl(int_marker);

  InteractiveMarkerControl control;
  // Relative control from the previous state
  control.orientation_mode = InteractiveMarkerControl::INHERIT;

  // x axis control
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  // stack this control to the list of controls
  int_marker.controls.push_back(control);

  // z axis control
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // y axis control
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // Insert the marker on the server
  server->insert(int_marker);
  // Set the callback function to process events/messages received
  server->setCallback(int_marker.name, &processFeedback);
}

// Create a menu marker
void makeMenuMarker( std::string name )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = base_frame;
  int_marker.name = name;
  // Place it at the back of the palm as a button.
  int_marker.pose.position.x = 0.0;
  int_marker.pose.position.y = 0.02;
  int_marker.pose.position.z = 0.05;
  int_marker.scale = 0.055;

  InteractiveMarkerControl control;

  // A menu is a specific control mode
  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.always_visible = true;
  // Add a box to this marker
  control.markers.push_back( makeBox( int_marker ) );
  // Stack this control to the list of controls
  int_marker.controls.push_back(control);

  // Insert the marker on the server
  server->insert(int_marker);
}

// Add an entry to the menu and set its callback function
void initMenu()
{
  h_attach = menu_handler.insert( "Attach ",&attachCb );
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_finger_control");
  ros::NodeHandle n;

  // Create an interactive marker server
  server.reset( new interactive_markers::InteractiveMarkerServer("interactive_finger_control","",false) );

  // Access the planning markers with its feedback and update topics
  moveit_marker_publisher = n.advertise<visualization_msgs::InteractiveMarkerFeedback>(
    "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",2);

  ros::Duration(0.1).sleep();

  // Create a joint TH and FF marker
  make3DofMarker("thff",base_frame);
  
  // Init and add a menu
  initMenu();
  makeMenuMarker( "FingerControlOptions" );
  menu_handler.apply( *server, "FingerControlOptions" );
  
  // apply changes to the server
  server->applyChanges();

  // start the ROS main loop
  ros::spin();

  server.reset();

}
