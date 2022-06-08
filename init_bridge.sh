#!/bin/bash

source /opt/ros/noetic/setup.bash
roscore &
roslaunch vrpn_client_ros sample.launch server:=192.168.1.6 --wait &

xterm -title "BRIDGE" -e "source /opt/ros/noetic/setup.bash;. ~/ros2_galactic/ros2-linux/setup.bash;ros2 run ros1_bridge dynamic_bridge --wait" 
