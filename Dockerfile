FROM osrf/ros:humble-desktop

RUN apt update
RUN apt install -y x11-apps && \
    apt install -y mesa-utils
RUN apt install -y net-tools

RUN apt install -y software-properties-common
RUN add-apt-repository ppa:kisak/kisak-mesa
RUN apt update
RUN apt upgrade -y

RUN apt install ros-humble-rviz2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib

ENTRYPOINT "./ros_entrypoint.sh"