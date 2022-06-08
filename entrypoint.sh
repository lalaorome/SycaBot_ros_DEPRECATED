#!/bin/bash
set -e

ros_workspace_setup="$WORKSPACE_ROOT/install/local_setup.bash"
echo "sourcing   $ros_workspace_setup"
source "$ros_workspace_setup"

exec "$@"