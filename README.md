# SycaBot_ros
Package to make decentralized multi-agent control and task assignment using jetbots and ros2. Use with https://github.com/lalaorome/Central_PC

# Configure and setup a JetBot to make it work with this package

## Step 1 : Install JetPack 4.5 and configure JetBot:

- Flash card with **JetPack version 4.5** for **Jetson Nano 4Gb** : [https://jetbot.org/master/software_setup/sd_card.html](https://jetbot.org/master/software_setup/sd_card.html)
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
    - Enable GUI : `sudo systemctl set-default graphical.target`
    - Disable GUI : `sudo systemctl set-default multi-user.target`

## Step 2 : Configure starting script

In order for the JetBot to directly launch the Docker at startup we need to tell the system which service files to enable at boot. 

- Clone this github repo :

```powershell
git clone https://github.com/lalaorome/SycaBot_ros
```

### Configure the sudo rights so that sudo doesn’t ask for password each time :

1. Open visudo using `sudo visudo` it opens a file that looks like that :

```powershell
#
# Some comments here
#

Defaults env_reset
Defaults mail_badpass
Defaults secure_path="a/path/here"

# Some commented lines without anything inbetween

# Some commented lines without anything inbetween

# Some commented lines without anything inbetween

# User privilege specification
root ALL=(ALL:ALL) ALL

# Members of the admin group may gain root privileges
%admin ALL=(ALL:ALL) ALL

# Allow members of group sudo to execute any command
%sudo ALL=(ALL:ALL) ALL

# includir /etc/sudoers.d
```

1. Replace all the `ALL=(ALL:ALL) ALL` by `ALL=(ALL:ALL) NOPASSWD ALL` 
To do so use `i` to pass in insert mode `esc` to echap insert mode and go in command mode. In command mode type `:wq` to save and close the file.

### Configure all the file for the docker to start correctly at startup

**Option 1 :** Source the config file from inside the cloned git repo SycaBot_ros

 `source config_jetbot.sh`

**Option 2 :** Do it manually 

- Copy the previously cloned workspace into a new workspace called syca_ws and enter it :

```powershell
cp -r SycaBot_ros/. syca_ws/
rm SycaBot_ros
cd syca_ws
```

- Copy the service file robot_boot.service inside /lib/systemd/system and test and the boot_init in /usr/local/bin/ and give mode 755 to allow execution:

```powershell
sudo cp robot_boot.service /lib/systemd/system/
sudo systemctl daemon-reload # To make systemctl see the service
sudo cp boot_init.sh /usr/local/bin/
sudo chown root:root /usr/local/bin/boot_init.sh
sudo chmod 755 /usr/local/bin/boot_init.sh
```

- Build the Docker on the jetbot using `sudo docker build . -t syca_jb`
- Change SYCABOT_ID in the launch file of jetbot package (syca_ws/src/jetbot/launch)
- Disable Jupyter Docker to save system memory consumption :

```powershell
cd ~/jetbot
source docker/disable.sh
cd ~/syca_ws
```

### Test that the service boots properly and enable it :

```powershell
# wait for few seconds
sudo systemctl status robot_boot # This should not display errors and finish by sourcing ...
sudo systemctl enable robot_boot
```

- Reboot the system using `sudo reboot now` and check that docker is running using 
`sudo docker ps`
