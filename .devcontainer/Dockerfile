FROM ros:galactic-ros-core-focal

# Add sudo user
RUN apt-get update && \
      apt-get -y install sudo

ARG USERNAME
RUN adduser --disabled-login $USERNAME \
        && echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME \
        && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> ~/.bashrc

CMD /bin/bash