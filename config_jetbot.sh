#!/bin/bash
show_help() {
    echo " "
    echo "  usage: Configure the jetbot"
    echo "          Should always be launched from SycaBot_ros directory"
    echo "          ./configure_jetbot.sh --file LAUNCH_FILE_NAME"
    echo ""
    echo ""
    echo ""
    echo "  args :"
    echo ""
    echo "      --help          Show this help"
    echo ""
    echo "      --file Launch file you want to execute at boot. keys : motors, init"
    echo "             default : init"
    echo ""
    echo "      --id Id of the jetbot being configured. keys : integer"
    echo "             default : 1"
    echo ""
    echo "      --install Install necessary packages and ROS2 (should be used at first initialization)"
    echo ""
    echo ""
    echo ""

}
die() {
    printf '%s\n' "$1"
    show_help
    exit 1
}

motors=true
init=false
LAUNCH_FILE_NAME="init"
ID="1"

while :; do
    case $1 in
        -h|-\?|--help)
            show_help    # Display a usage synopsis.
            exit
            ;;
        --file)
            if [ "$2" == "motors" ];then
                LAUNCH_FILE_NAME="$2"
                echo $LAUNCH_FILE_NAME
                shift
            elif [ "$2" == "init" ];then
                LAUNCH_FILE_NAME="$2"
                echo $LAUNCH_FILE_NAME
                shift
            else
                die 'ERROR: "--pkg" wrong input argument.'
            fi
            ;;
        --id)
            if [ "$2" ];then
                ID="$2"
                shift
            else
                die 'ERROR: "--id" requires a non-empty option argument.'
            fi
            ;;
        --install)
            echo 'installing adafruit ...'
            sudo pip3 install Adafruit-MotorHAT Adafruit-SSD1306 pyserial sparkfun-qwiic --verbose

            echo 'install matplotlib ...'
            # https://forums.developer.nvidia.com/t/jetson-nano-how-can-install-matplotlib/75132/7
            sudo cp /etc/apt/sources.list /etc/apt/sources.list~
            sudo sed -Ei 's/^# deb-src /deb-src /' /etc/apt/sources.list
            sudo apt-get update
            cd ~/.local/lib/python3.6/site-packages
            git clone -b v3.3.4 --depth 1 https://github.com/matplotlib/matplotlib.git 22
            cd ~/.local/lib/python3.6/site-packages/matplotlib
            sudo apt-get build-dep python3-matplotlib -y
            # pip3 install . -v
            sudo mv /etc/apt/sources.list~ /etc/apt/sources.list
            sudo apt-get update

            echo 'installing control ...'
            sudo pip3 install control --verbose
            shift
            ;;
        --)              # End of all options.
            shift
            break
            ;;
        -?*)
            printf 'WARN: Unknown option (ignored): %s\n' "$1" >&2
            ;;
        *)               # Default case: No more options, so break out of the loop.
            break
    esac
    shift
done

#Create the good directory
cd ~/SycaBot_ros
sudo cp -r ./. ../syca_ws/

# Copy service file and make systemctl recgonize it
# Set the boot file

sudo sed -i -r "s/ros2 launch jetbot [a-z]+/ros2 launch jetbot ${LAUNCH_FILE_NAME}/" boot_init.sh
echo ""
echo ""
cat boot_init.sh
echo ""
echo ""
echo 'Copying service and boot file...'
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
sudo sed -i "s/SYCABOT_ID = ./SYCABOT_ID = ${ID}/" init.launch.py
sudo sed -i "s/SYCABOT_ID = ./SYCABOT_ID = ${ID}/" motors.launch.py
echo ""
echo ""
cat init.launch.py
echo ""
echo ""
cat motors.launch.py
cd ~/syca_ws