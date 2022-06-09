#!/bin/bash

cd ../Syca_ws/src/jetbot/launch
read -p 'SYCABOT_ID : ' ID 
sed -i "s/SYCABOT_ID = 1/SYCABOT_ID = ${ID}/" init.launch.py
cd ~/SycaBot_ros