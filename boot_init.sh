#!/bin/bash

cd syca_ws
source Docker/run.sh

ros2 launch jetbot init.launch.py
