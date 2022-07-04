#!/bin/bash

cd home/jetbot/syca_ws
source setup_ROS.sh --pkg sycabot --srv 5
ros2 launch jetbot init.launch.py