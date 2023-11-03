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

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# WSL
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib

COPY ros_entrypoint.sh /setup/ros_entrypoint.sh
COPY src /dagros/src

ENTRYPOINT ["/setup/ros_entrypoint.sh"]