FROM osrf/ros:humble-desktop

# General Utilities
RUN apt update
RUN apt install -y x11-apps
RUN apt install -y mesa-utils
RUN apt install -y net-tools
RUN apt install -y wget
RUN apt install -y python3-vcstool
RUN apt install -y net-tools iputils-ping
RUN apt install -y openssh-client git

# download public key for github.com
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Install VCPKG
RUN apt install -y curl zip unzip tar
RUN --mount=type=ssh git clone https://github.com/microsoft/vcpkg
RUN ./vcpkg/bootstrap-vcpkg.sh

# OpenGL bug fix
RUN apt install -y software-properties-common
RUN add-apt-repository ppa:kisak/kisak-mesa
RUN apt update
RUN apt upgrade -y

# ROS2 Installs
RUN apt install -y ros-humble-rviz2
RUN apt install -y ros-humble-ros2-control && \
    apt install -y ros-humble-ros2-controllers
RUN apt install -y ros-humble-moveit

# ROS2 Configurations
RUN rosdep update
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# ROS2 Repos
RUN mkdir -p /workspaces/3rd_party_ws/src
WORKDIR /workspaces/3rd_party_ws

## LBR FRI ROS2 Stack
RUN apt install -y ros-dev-tools
RUN vcs import src --input https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos-fri-1.15.yaml

## Cartesian Controllers
RUN --mount=type=ssh git clone -b ros2 git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git src/cartesian_controllers

RUN rosdep install --from-paths src --ignore-src --rosdistro humble -y 
RUN . /opt/ros/humble/setup.sh && colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo "source /workspaces/3rd_party_ws/install/setup.bash" >> ~/.bashrc

RUN mkdir -p /workspaces/dagros_ws/src
WORKDIR /workspaces/dagros_ws

# WSL
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib

COPY entrypoint.sh /workspaces/entrypoint.sh
RUN chmod +x /workspaces/entrypoint.sh
ENTRYPOINT ["/workspaces/entrypoint.sh"]
