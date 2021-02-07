#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "simple_navigation_goals/pose.h"
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <unistd.h>

bool marker_placed = false;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Calbback that only checks if the marker is placed on Rviz and then send the nav goal1
void marker_placed_callback(const std_msgs::Bool& msg) 
{
	marker_placed = true;
	ROS_INFO("Marker Placed");
}



int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Create a handle to the arm_mover node
  ros::NodeHandle n;
  // Create a publisher to send the information when the robot reaches the goals
  ros::Publisher marker_status_pub = n.advertise<std_msgs::String>("marker_status", 10);
  // Create a publisher to send the information about the goal position
  ros::Publisher destination_pose_pub = n.advertise<simple_navigation_goals::pose>("marker_destination_pose", 10);
  ros::Subscriber marker_placed_sub = n.subscribe("marker_placed", 1, marker_placed_callback);
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal1, goal2;

  // set up the frame parameters
  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();
  
  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to pickup the marker
  goal1.target_pose.pose.position.x = 1.0;
  goal1.target_pose.pose.position.y = 1.0;
  goal1.target_pose.pose.orientation.w = 1.0;
  
  // Define a position and orientation for the robot to dropoff the marker
  goal2.target_pose.pose.position.x = -5;
  goal2.target_pose.pose.position.y = -5;
  goal2.target_pose.pose.orientation.w = -1.0;

  //Declare a message named destination_pose as an array of geometry_msgs
  simple_navigation_goals::pose destination_pose;
  geometry_msgs::Point point1, point2;
  
  //Clear the destination_pose array
  destination_pose.points.clear();

  //Populate the array with both goal1 and goal2
  point1.x = goal1.target_pose.pose.position.x;
  point1.y = goal1.target_pose.pose.position.y;
  point1.z = goal1.target_pose.pose.orientation.w;
  destination_pose.points.push_back(point1);
  point2.x = goal2.target_pose.pose.position.x;
  point2.y = goal2.target_pose.pose.position.y;
  point2.z = goal2.target_pose.pose.orientation.w;
  destination_pose.points.push_back(point2);
  
  //Keep sending the destination_pose message untill the marker is placed on Rviz
  while (marker_placed == false)
  {
	ros::spinOnce();
	//publish the position set to the topic
  	destination_pose_pub.publish(destination_pose);
  }

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal 1");
  ac.sendGoal(goal1);
    
  // Wait an infinite time for the results
  ac.waitForResult();

  std_msgs::String pickup_msg;
  pickup_msg.data = "pickup";
  
  // Check if the robot reached its goal and print the status if it suceeded
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("SUCEED GOAL 1 POSITION");
    marker_status_pub.publish(pickup_msg);
  } else
    ROS_INFO("The base failed to reach goal 1 for some reason");
    
  sleep(5);
  


  
   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal 2");
  ac.sendGoal(goal2);
  destination_pose_pub.publish(destination_pose);
  
  // Wait an infinite time for the results
  ac.waitForResult();

  std_msgs::String dropoff_msg;
  dropoff_msg.data = "dropoff";
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("SUCEED GOAL 2 POSITION");
    marker_status_pub.publish(dropoff_msg); 
  } else
    ROS_INFO("The base failed to reach goal 2 for some reason");

	ros::spin();
		
  return 0;
}
