#!/bin/bash
set -e

ros_env_setup="/opt/ros/$ROS_DISTRO/setup.bash"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"

ros_workspace_setup="$WORKSPACE_ROOT/install/local_setup.bash"
echo "sourcing   $ros_workspace_setup"
source "$ros_workspace_setup"

ros_workspace_setup="/syca_ws/install/local_setup.bash"
echo "sourcing   $ros_workspace_setup"
source "$ros_workspace_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

FASTRTPS_DEFAULT_PROFILES=scripts/ddsconfig.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch jetbot init.launch.py

exec "$@"
