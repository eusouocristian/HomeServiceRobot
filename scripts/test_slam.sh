#!/bin/bash
xterm -e  " roslaunch my_robot turtlebot_world.launch " &
sleep 5
xterm -e  " roslaunch my_robot gmapping_demo.launch " &
sleep 5
xterm -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm -e  " roslaunch my_robot view_navigation.launch " &
sleep 5
xterm -e  " roslaunch my_robot keyboard_teleop.launch " &

