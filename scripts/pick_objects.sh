#!/bin/bash
xterm -e  " roslaunch my_robot turtlebot_world.launch " &
sleep 2
xterm -e  " roslaunch my_robot amcl_demo.launch " &
sleep 2
xterm -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
xterm -e  " roslaunch my_robot view_navigation.launch " &
sleep 5
xterm -e  " rosrun simple_navigation_goals pick_objects " &


