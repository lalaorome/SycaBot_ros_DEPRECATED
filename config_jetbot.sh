#!/bin/bash

#Create the good directory
sudo cp -r ./. ../syca_ws/

# Copy service file and make systemctl recgonize it
echo 'Copying service and boot file'
sudo cp robot_boot.service /lib/systemd/system/
sudo cp boot_init.sh /usr/local/bin/
echo 'chown /usr/local/bin/boot_init.sh'
sudo chown root:root /usr/local/bin/boot_init.sh
echo 'chmod 755 /usr/local/bin/boot_init.sh'
sudo chmod 755 /usr/local/bin/boot_init.sh
echo 'daemon reload'
sudo systemctl daemon-reload

# Change SYCABOT_ID number : https://www.geeksforgeeks.org/sed-command-in-linux-unix-with-examples/
cd ../syca_ws/src/jetbot/launch
read -p 'SYCABOT_ID : ' ID 
echo 'changing SYCABOT_ID'
sudo sed -i "s/SYCABOT_ID = 1/SYCABOT_ID = ${ID}/" init.launch.py
cd ~/syca_ws

# Build the docker
echo 'Building docker'
sudo docker build . -t syca_jb

# disable jupyter docker
echo 'disabling Jupyter docker'
cd ~/jetbot
source docker/disable.sh
cd ~/syca_ws
sudo systemctl start robot_boot
