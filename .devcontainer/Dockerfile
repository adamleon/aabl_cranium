FROM ros:galactic

# Install packages
RUN apt-get update && \
      apt-get -y install sudo git gcc && \
      sudo apt -y install ros-galactic-moveit ros-galactic-ros2-control ros-galactic-ros2-controllers

COPY /cyclonedds.xml /

RUN export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file:///cyclonedds.xml
CMD /bin/bash