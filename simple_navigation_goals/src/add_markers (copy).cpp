#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include "simple_navigation_goals/pose.h"
#include <nav_msgs/Odometry.h>
#include <string>

visualization_msgs::Marker marker;
float goal1[3];
float goal2[3];
bool pick = false;
bool destination_received = false;

void marker_st_changed(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Package status: [%s]", msg->data.c_str());
  
  // Set variables to build the if conditional
  std::string package_status = msg->data.c_str();
  std::string pickup_string = "pickup";
 
  if (pickup_string == package_status) {
  	pick = true;
  }
}

// Receive the targets from the "marker_destination_pose" topic, published by pick_objects.cpp
void marker_destination_pose(const simple_navigation_goals::pose::ConstPtr& msg)
{
  goal1[0] = msg->points[0].x;
  goal1[1] = msg->points[0].y;
  goal1[2] = msg->points[0].z;
  
  goal2[0] = msg->points[1].x;
  goal2[1] = msg->points[1].y;
  goal2[2] = msg->points[1].z;
  destination_received = true;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber marker_st_sub = n.subscribe("marker_status", 10, marker_st_changed);
  ros::Subscriber marker_destination_sub = n.subscribe("marker_destination_pose", 10, marker_destination_pose);  
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

	//Wait untill receiving the first destination information
    while(destination_received==false) {
		ros::spinOnce();
	}
    
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	// The pose of the marker is equal to the goals set in pick_objects.cpp
	if (destination_received==true) {
		marker.pose.position.x = goal1[0];
		marker.pose.position.y = goal1[1];
		marker.pose.position.z = 0.1;
		marker.pose.orientation.w = goal1[2];
	}


    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    } 
    marker_pub.publish(marker);

	//Wait untill the robot reaches the first goal position
	while(pick==false) {
		ros::spinOnce();
	}
	
	// When the robot reaches the fist goal, delete the marker and publish it on the second position goal
	if (pick == true) {
		ros::Duration(2.0).sleep();
    	marker.action = visualization_msgs::Marker::DELETE;
    	marker_pub.publish(marker);
    	
    	marker.pose.position.x = goal2[0];
		marker.pose.position.y = goal2[1];
		marker.pose.position.z = 0.1;
		marker.pose.orientation.w = goal2[2];
    	marker.action = visualization_msgs::Marker::ADD;
    	marker_pub.publish(marker);
	}

/*
	//Wait untill the package is dropped off
	while(drop==false) {
		ros::spinOnce();
	}
	
	//DELETE the marker from Rviz
	if (drop == true) {
		ros::Duration(2.0).sleep();
    	marker.action = visualization_msgs::Marker::DELETE;
    	marker_pub.publish(marker);
	}
*/
	ros::spin();
    r.sleep();
  }
}
