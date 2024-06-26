FROM osrf/ros:humble-desktop

# General Utilities
RUN apt update
RUN apt install -y x11-apps && \
    apt install -y mesa-utils
RUN apt install -y net-tools
RUN apt install -y wget

# OpenGL bug fix
RUN apt install -y software-properties-common
RUN add-apt-repository ppa:kisak/kisak-mesa
RUN apt update
RUN apt upgrade -y

# ROS 2 installs
RUN apt install -y ros-humble-rviz2
RUN apt install -y ros-humble-ros2-control && \
    apt install -y ros-humble-ros2-controllers
RUN apt install -y ros-humble-moveit

RUN echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc

# ROS2 repos
COPY src /dagros_ws/src
RUN cd dagros_ws/src

#RUN git clone git@github.com:lbr-stack/lbr_fri_ros2_stack.git
#RUN git clone -b ros2 git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git
#RUN rosdep install --from-paths ./ --ignore-src -y
#RUN cd ..
#RUN colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release

# WSL
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib

COPY entrypoint.sh /setup/entrypoint.sh

ENTRYPOINT ["/setup/entrypoint.sh"]