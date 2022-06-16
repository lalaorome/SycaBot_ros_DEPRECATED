#!/usr/bin/env bash

ROS_DISTRO="foxy"

# where the project resides inside docker
DOCKER_ROOT="/syca_ws/src"

# generate mount commands
DEV_VOLUME="--volume $PWD:$DOCKER_ROOT"

# check for V4L2 devices
V4L2_DEVICES=" "

CONTAINER_IMAGE="syca_jb:latest"


for i in {0..9}
do
	if [ -a "/dev/video$i" ]; then
		V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
	fi
done

MOUNTS="\
--device /dev/snd \
--device /dev/bus/usb \
--volume /etc/timezone:/etc/timezone:ro \
--volume /etc/localtime:/etc/localtime:ro \
--volume /dev/shm:/dev/shm \
$DEV_VOLUME \
$V4L2_DEVICES"


# run the container
sudo docker run --runtime nvidia --rm --name jetbot_ros \
    --network host \
    --privileged \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /etc/enctune.conf:/etc/enctune.conf \
    $MOUNTS $CONTAINER_IMAGE $USER_COMMAND
