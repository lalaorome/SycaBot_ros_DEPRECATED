#!/bin/bash

cp robot_boot.service /lib/systemd/system
sudo systemctl daemon-reload
sudo docker build . -t syca_jb
cd ~/jetbot
source docker/disable.sh
cd ~/syca_ws
sudo systemctl start robot_boot
