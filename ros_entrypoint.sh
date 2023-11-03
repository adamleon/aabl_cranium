#!/bin/bash

cd /dagros
wget https://raw.githubusercontent.com/KCL-BMEIS/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos.yml -P src
vcs import src < src/repos.yml

echo "Hello this was running"

tail -f /dev/null