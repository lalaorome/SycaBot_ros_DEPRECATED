#!/bin/bash

cd home/jetbot/syca_ws
source setup_ROS.sh --pkg sycabot --srv 9
ros2 launch jetbot init.launch.py