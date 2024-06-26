#!/bin/bash

. /opt/ros/humble/setup.bash

#cd /dagros_ws/src
#git clone -b ros2 https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git

#wget https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos.yaml
#vcs import . < repos.yaml

#rosdep install --from-paths ./ --ignore-src -y
#cd ..
#colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release

#echo ". /dagros_ws/install/setup.bash" >> ~/.bashrc

echo -e "\033[32mÅBL Robot Arm is finished installed.\033[m"

tail -f /dev/null