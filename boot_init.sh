#!/bin/bash

cd syca_ws
source docker/run.sh
source install/local_setup.bash
ros2 launch jetbot init.launch.py