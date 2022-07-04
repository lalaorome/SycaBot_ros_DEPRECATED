# SycaBot_ros
Package to make decentralized multi-agent control and task assignment using jetbots and ros2. Use with https://github.com/lalaorome/Central_PC

Please also don't hesitate to read the links in RESOURCES.md and the report.pdf file for further informations.

Have a look at ROS2_Toolkit if you want a catch up on ROS2 basics.

# Configure and setup a JetBot to make it work with this package

## Step 1 : Flash the card image and initialize the Jetbot

- Flash card with **JetPack version 4.5** for **Jetson Nano 4Gb** using the card image **sdcard_ros2_copy.img.gz** (Initial image from : [https://jetbot.org/master/software_setup/sd_card.html](https://jetbot.org/master/software_setup/sd_card.html) and we installed ROS2 manually on it.)
- Boot the JetBot with the HDMI, Keyboard and mouse. **It is normal that there is no GUI it was disabled.**
- WARNING : If you see that only 24 Gb are available on PioLED enter these instructions :

```powershell
cd ~/jetcard
git pull
git checkout jetpack_4.5.1
./scripts/jetson_install_nvresizefs_service.sh
cd
rm -rf jetcard
sudo reboot now
```

- Connect the JetBot to Wifi using :
`sudo nmcli device wifi connect SycamoreNet password Sycam0re`
- Enter : `sudo nvpmodel -m1` to put the Nano in 5W mode
- OPTIONAL : clone the newest jetbot repo to have access to the examples see steps here :
    - Waveshare : [https://www.waveshare.com/wiki/JetBot_AI_Kit](https://www.waveshare.com/wiki/JetBot_AI_Kit)
    - Nvidia : [https://github.com/NVIDIA-AI-IOT/jetbot/wiki/software-setup](https://github.com/NVIDIA-AI-IOT/jetbot/wiki/software-setup)
- OPTIONAL : if you want to enable or disable GUI enter this commands in the terminal :
    - Enable GUI : `sudo systemctl set-default graphical.target`
    - Disable GUI : `sudo systemctl set-default multi-user.target`

## Step 2 : Configure the jetbot

We want the jetbot to directly launch the ROS2 nodes at startup, hence we need to tell the system which service files to enable at boot. 

- Clone this github repo :

```powershell
git clone https://github.com/lalaorome/SycaBot_ros
```

### Configure all the file for the docker to start correctly at startup

**Source the config file** from inside the cloned git repo SycaBot_ros :

 `source config_jetbot.sh`

### Build the package

```powershell
sudo -i # enter root user
source /opt/ros/foxy/setup.bash # source ros2
cd /home/jetbot/syca_ws/
colcon build --symlink-install # Build package
exit
```

### Test that the service boots properly and enable it :

```powershell
sudo systemctl start robot_boot
# wait for few seconds
sudo systemctl status robot_boot # This should not display errors
sudo systemctl enable robot_boot
```

- Reboot the system using `sudo reboot now`
