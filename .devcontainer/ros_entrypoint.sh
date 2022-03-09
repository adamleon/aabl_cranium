#!/bin/bash
set -e

source /opt/ros/galactic/setup.bash
source ~/workspaces/ros2_vscode_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "Hello this was running"

# export CYCLONEDDS_URI=file:///cyclonedds.xml